// cms_eye.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>
#include <windows.h>
#include "tchar.h"
#include "autofocus.h"
#include "ASICamera2.h"
#include <filesystem>
// #include <atlstr.h>

#include<thread>
#include <iomanip>
#include <sstream>
//#include "opencv\sources\include\opencv2\opencv.hpp"
#include <chrono>
#include <mutex>
#include <condition_variable>
#include "pid.h"

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

//Uncomment if using 480 * 640 image
double scale = 0.5; 
const long img_size = 640 * 480;
// Video capture. Accessed by T1.
unsigned char* img_data_next_read_buf = (unsigned char*)malloc(img_size);
// Transfer. Accessed by T1 and T2.
unsigned char* img_data_filled_read_buf = (unsigned char*)malloc(img_size);
// Calculation. Accessed by T2.
unsigned char* img_data_calc_buf = (unsigned char*)malloc(img_size);


//Some control flags for easy testing
bool Multihreading_f = 1; //set to 0 to turn off multithreading
bool SaveConsoleToFile_f = 0; //0 to have cout go to console, 1 to have cout go to a file
bool BestFocusFromFirstImage_f = 1; //0 to have the location of best-focus be the center, 1 to have the location of best-focus from the first images



cv::Mat img;
cv::Mat TempImg;
ASI_CONTROL_TYPE ControlType;
std::atomic<bool> bNewImage = 0;
std::atomic<bool> bMotorMoving = 0;
bool bAutofocusing = 0;
int imgCount = 0;
int ASI_ERROR_CODE = 0;

std::mutex mtx; 

////Uncomment if using 960 * 1280 image
//double scale = 1;
//const long img_size = 960 * 1280;
//unsigned char img_data[1228800]; //960 * 1280
//unsigned char img_data_temp[1228800]; //480 * 640


// part of MULTITHREADING
void CaptureVideo(void) {
    while (bAutofocusing) {
        if (bMotorMoving) {
        //do nothing
        }
        else {
            ASI_ERROR_CODE = ASIGetVideoData(0, img_data_next_read_buf, img_size, -1);  //pulls image data from camera and saves as img_data
            {
                std::lock_guard<std::mutex> lck{ mtx };
                auto temp = img_data_filled_read_buf;
                img_data_filled_read_buf = img_data_next_read_buf;
                img_data_next_read_buf = temp;
                bNewImage = 1;
            }
            imgCount++;

            if (ASI_ERROR_CODE != 0) {
                std::cout << "Error getting image from camera!\n";
        }
    }
}
}

int main()
{
    int iNumofConnectCameras = ASIGetNumOfConnectedCameras();
    if (iNumofConnectCameras > 0) {
        std::cout << "Connected to the camera(s)\n";
    }
    else std::cout << "Failed to connect to the camera(s)";

    //Initializes and opens the camera
    ASIOpenCamera(0);
    ASIInitCamera(0);

    //Initializes the COM port for the motor
    __init__();
    Sleep(1000);


    std::string FileName = "null";
    while (TRUE) {
        std::cout << "File name to store images:\n";
        std::cin >> FileName;
        std::string FilePath = "C:\\Users\\HVI\\Desktop\\HVI_Images\\" + FileName;

        CreateDirectoryA(FilePath.c_str(), NULL);

        if (FileName != "null") {
            break;
        }

    }

    std::string TextFile = "C:\\Users\\HVI\\Desktop\\HVI_Data\\" + FileName + "_FittedCurve.txt";
    std::string TextFile2 = "C:\\Users\\HVI\\Desktop\\HVI_Data\\" + FileName + "_RobertCross.txt";

    std::ofstream outputFile(TextFile);
    std::ofstream outputFile2(TextFile2);


    int time_autofocusing_s = 0;
    while (TRUE) {
        std::cout << "How long should the code run for (s)?\n";
        std::cin >> time_autofocusing_s;

        if (time_autofocusing_s != 0) {
            break;
        }

    }

    while (TRUE) {
        char ch;
        std::cout << "Ready and in focus? (y/n):\n";
        std::cin >> ch;

        if (ch == 'y') {
            break;
        }

    }


    if (scale == 0.5) {
        ASISetROIFormat(0, 640, 480, 2, ASI_IMG_RAW8); //Switches to 640*480 at hardware level
    }
    else if (scale == 1) {
        ASISetROIFormat(0, 1280, 960, 1, ASI_IMG_RAW8);
    }

    int width, height, bin;
    long lBuffSize;
    ASI_IMG_TYPE image_type;
    ASIGetROIFormat(0, &width, &height, &bin, &image_type);

    const long size = width * height;
    std::cout << width << "\n";
    std::cout << height << "\n";

    Sleep(1000);

    long lValue;
    ASI_BOOL bAuto;
    ASIGetControlValue(0, ControlType, &lValue, &bAuto);

    ASISetControlValue(0, ASI_HIGH_SPEED_MODE, 1, bAuto);
    //ASISetControlValue(0, ASI_AUTO_MAX_EXP, 15, bAuto);
    //ASISetControlValue(0,  ASI_EXPOSURE, 15, bAuto);
    //ASISetControlValue(0, ASI_BANDWIDTHOVERLOAD, 90, bAuto);

    ASIStartVideoCapture(0);

    Sleep(1000);

    //int center = 640 * scale; //PIXELS ARE ALL DOUBLED BECAUSE IMAGE WIDTH HAS GONE FROM 640 TO 1280
    int center = 368;
    int moved = 1;
    int previous = center;
    //int tol = 26 * scale; //Tolerance zone of pixels in the center of image where no lense movement is triggered
    int tol = 12 * scale;

    int blink = 0; //Becomes 1 when a blink is detected
    int blinkframes = 10; //number of frames to ignore when blink is detected

    double slope = 0.004/5.0; //0.0042 is the best-case slope, 0.0016 is the worst-case slope. Higher means more sensitive, but more overshooting.
    double intercept = 0; //Might be useful in the future to bias the movement one way or the other.
    double mmToMove;

    //// PID CONTROLLER
    double dt = 1.0 / 30.0; //time per frame
    double max = 3;  //maximum relative move the lens can be ordered to make. Set to 3mm
    double min = -3; 
    double Kp = 0.001;
    double Ki = 0.0001;
    double Kd = 0;
    PID pid = PID(dt, max, min, Kp, Ki, Kd);



    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;
    using std::chrono::seconds;

    //Outputs cout to text file
    if (SaveConsoleToFile_f) {
       std::string FilePath2 = "C:\\Users\\HVI\\Desktop\\HVI_ConsoleOutput\\" + FileName + ".txt";
       freopen(FilePath2.c_str(), "w", stdout);
    }


    ////Outputs image to window
    //std::string windowName = "Window1";
    //cv::namedWindow("Window1");

    int timeout = 0;
    auto t1 = std::chrono::steady_clock::now();
    Sleep(10);
    auto t2 = std::chrono::steady_clock::now();
    auto s_int = duration_cast<seconds>(t2 - t1);

    // MULTITHREADING 
    bAutofocusing = 1;


    std::thread tCaptureVideo(CaptureVideo);
    if (!Multihreading_f) {
        bAutofocusing = 0;
        tCaptureVideo.join();
        Sleep(10);
        bAutofocusing = 1;
    }

    int i = 0;

    while(bAutofocusing) {

        if (!Multihreading_f) {
            if (ASIGetVideoData(0, img_data_next_read_buf, size, -1) != ASI_SUCCESS)
            {
        	    std::cout << "Error getting image from camera!\n";
            }
            else {
                std::cout << "ERROR\n";
                bNewImage = 1;
            }
        }


        t2 = std::chrono::steady_clock::now();
        s_int = duration_cast<seconds>(t2 - t1);

        if (s_int.count() > time_autofocusing_s-1) {
            break; // breaks when code runs longer than duration
        }
        
        if (bNewImage) {
            {
                std::lock_guard<std::mutex> lck{ mtx };
                auto temp = img_data_filled_read_buf;
                img_data_filled_read_buf = img_data_calc_buf;
                img_data_calc_buf = temp;
                bNewImage = 0;
            }

            i++;
            img = cv::Mat(480, 640, CV_8U, img_data_calc_buf);

            int location = sharpness(img, 0.5, outputFile, outputFile2);
            std::cout << location << ", "; //print location to console


            cv::Point p1(location, 0), p2(location, 479);
            cv::line(img, p1, p2, cv::Scalar(0, 0, 255), 2);


            //cv::imwrite("C:\\Users\\HVI\\Desktop\\HVI_Images\\" + FileName + "\\img" + std::to_string(i) + "_" + std::to_string(location) + ".png", img);

            if (i == 1) {
                if (BestFocusFromFirstImage_f) {
                    center = location;
                    previous = center;
                }
            }
            
            //else if (i < 5) {
            //    //ignores the first few images which can sometimes be blank
            //}

            //// BLINK DETECTION

        //    else if (moved == 0 && blink == 0 && abs(location - previous) > (400 * scale)) { // if location of best focus changes by more than 200 pixels with no move, is a blink
        //        //Blink starts
        //        std::cout << "Frame ignored; blink detected\n";
        //        blink = 1;
        //    }

        //    else if (blink == 1) {
        //        std::cout << "Frame ignored; blink detected\n";
        //        blinkframes--;
        //        if (blinkframes == 0) {
        //            blinkframes = 10;
        //            blink = 0;
        //        }
        //    }





        //    else {

        //        if (abs(location - center) <= tol) { // tol
        //            std::cout << "No movement; centered\n";
        //            moved = 0;
        //        }

        //        else {
        //            bMotorMoved = 1;
        //            //// PID CONTROLLER
        //            //double inc = pid.calculate(center, location);
        //            //inc = inc * -1.0;
        //            //mov_rel(inc);
        //            //std::cout << "Moved " << inc << "mm (PID)";

        //            moved = 1;
        //            timeout = 0;



        //            //// PROPORTIONAL CONTROLLER


        //            /*
        //            //// Overshoot guard
        //            //else if (moved == 1 && abs(location - previous) < 10 && timeout < 6) { //best-focus hasn't moved 5 pixels from previous lens move, so still a delay
        //            //    //Picture hasn't updated from previous move
        //            //    std::cout << "Waiting for picture to update\n";
        //            //    timeout++;
        //            //}


        //            else {*/
        //            mmToMove = (location - center) * slope + intercept;
        //            //mov_rel(mmToMove);

        //            std::cout << "vs. " << mmToMove << "mm (P)\n";
        //            moved = 1;
        //            timeout = 0;
        //            //} 
        //        }

        //        //// ZONING SYSTEM 

        //        /*else if (abs(location - center) < (200 * scale)) {
        //            if (location > center) {
        //                mov_rel(0.05);
        //                std::cout << "Moved left; zone 1\n";
        //            }
        //            else {
        //                mov_rel(-0.05);
        //                std::cout << "Moved right; zone 1\n";
        //            }
        //            moved = 1;
        //            timeout = 0;
        //        }

        //        else if (abs(location - center) < (300 * scale)) {
        //            if (location > center) {
        //                mov_rel(0.2);
        //                std::cout << "Moved left; zone 2\n";
        //            }
        //            else {
        //                mov_rel(-0.2);
        //                std::cout << "Moved right; zone 2\n";
        //            }
        //            moved = 1;
        //            timeout = 0;
        //        }

        //        else if (abs(location - center) < (500 * scale)) {
        //            if (location > center) {
        //                mov_rel(0.3);
        //                std::cout << "Moved left; zone 3\n";
        //            }
        //            else {
        //                mov_rel(-0.3);
        //                std::cout << "Moved right; zone 3\n";
        //            }
        //            moved = 1;
        //            timeout = 0;
        //        }

        //        else {
        //            if (location > center) {
        //                mov_rel(0.5);
        //                std::cout << "Moved left; zone 4\n";
        //            }
        //            else {
        //                mov_rel(-0.5);
        //                std::cout << "Moved right; zone 4\n";
        //            }
        //            moved = 1;
        //            timeout = 0;
        //        }*/
        //    }
        //    bMotorMoved = 0;
        //    previous = location;
        
        } 
    }


    /* Getting number of milliseconds, seconds as an integer. */
    auto ms_int = duration_cast<milliseconds>(t2 - t1);

    std::cout << ms_int.count() << "ms\n";
    std::cout << s_int.count() << "s\n";

    std::cout << imgCount << " number of images \n";
    std::cout << i << " iterations of sharpness calculation \n";

    close_motor();
    
    bAutofocusing = 0;

    if (Multihreading_f) {
        tCaptureVideo.join();
    }
    

    ASIStopVideoCapture(0);
    if (ASICloseCamera(0) == ASI_SUCCESS)
    {
        std::cout << "closed\n";
    }

    outputFile.close();
    outputFile2.close();

    if (SaveConsoleToFile_f) {
        fclose(stdout);
    }
    

    return 0;
}

