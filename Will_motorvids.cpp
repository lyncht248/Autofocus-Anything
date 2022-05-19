//// autofocus_v4.cpp : This file contains the 'main' function. Program execution begins and ends there.
////
//
//#include <iostream>
//#include <windows.h>
//#include "tchar.h"
//#include "autofocus.h"
//#include "ASICamera2.h"
//#include <filesystem>
//// #include <atlstr.h>
//
//#include <iostream>
//#include <iomanip>
//#include <sstream>
////#include "opencv\sources\include\opencv2\opencv.hpp"
//#include <chrono>
//
//////Uncomment if using 480 * 640 image
////double scale = 0.5; 
////unsigned char img_data[307200]; //480 * 640
//
////Uncomment if using 960 * 1280 image
////double scale = 1;
////unsigned char img_data[1228800]; //960 * 1280
//
//int main()
//{
//    //int iNumofConnectCameras = ASIGetNumOfConnectedCameras();
//    //if (iNumofConnectCameras > 0) {
//        //std::cout << "Connected to the camera(s)\n";
//    //}
//    //else std::cout << "Failed to connect to the camera(s)";
//
//    //Initializes and opens the camera
//    //ASIOpenCamera(0);
//    //ASIInitCamera(0);
//
//
//    //Initializes the COM port for the motor
//    __init__();
//    Sleep(1000);
//
//
//    //std::string FileName = "null";
//    //while (TRUE) {
//        //std::cout << "File name to store images:\n";
//        //std::cin >> FileName;
//        //std::string FilePath = "C:\\Users\\HVI\\Desktop\\HVI_Images\\" + FileName;
//
//        //CreateDirectoryA(FilePath.c_str(), NULL);
//
//        //if (FileName != "null") {
//            //break;
//        //}
//
//    //}
//
//    //int frames = 0;
//    //while (TRUE) {
//        //std::cout << "How many frames?\n";
//        //std::cin >> frames;
//
//        //if (frames != 0) {
//            //break;
//        //}
//
//    //}
//
//    //while (TRUE) {
//        //char ch;
//        //std::cout << "Ready and in focus? (y/n):\n";
//        //std::cin >> ch;
//
//        //if (ch == 'y') {
//            //break;
//        //}
//
//    //}
//
//    //Test removing these and see how that affects the resulting images.
//
////	ASISetControlValue(0, ASI_HIGH_SPEED_MODE, 1, ASI_FALSE); //Sets to high-speed mode
//    //if (scale == 0.5) {
//        //ASISetROIFormat(0, 640, 480, 1, ASI_IMG_RAW8); //Switches to 640*480 at hardware level. Need to check ASI_IMG_RAW8 using GetROIFormat below.
//    //}
//    //else if (scale == 1) {
//    //    ASISetROIFormat(0, 1280, 960, 1, ASI_IMG_RAW8);
//    //}
//    //Sleep(1000);
//
//
//    //ASIStartVideoCapture(0);
//
//    //int width, height, bin;
//    //long lBuffSize;
//    //ASI_IMG_TYPE image_type;
//
//    //ASIGetROIFormat(0, &width, &height, &bin, &image_type);
//
//    //const long size = width * height;
//    //std::cout << width << "\n";
//    //std::cout << height << "\n";
//
//
//
//
//    //   char x[1];
//    //   strncpy(x,"n",1);
//       //char y[1] = {'n'};
//       //while (x == y) {
//       //	std::cout << "Ready and in focus? (y/n):  ";
//       //	std::cin >> x;
//       //}
//
//    //int center = 640 * scale; //PIXELS ARE ALL DOUBLED BECAUSE IMAGE WIDTH HAS GONE FROM 640 TO 1280
//    //int moved = 1;
//    //int previous = center;
//    //int tol = 100 * scale; //Tolerance zone of pixels in the center of image where no lense movement is triggered
//    
//    //vv("05");
//
//
//    using std::chrono::high_resolution_clock;
//    using std::chrono::duration_cast;
//    using std::chrono::duration;
//    using std::chrono::milliseconds;
//    using std::chrono::seconds;
//
//    auto t1 = high_resolution_clock::now();
//    for (int i = 0; i < 10; ++i) {
//        mov_rel(11);
//        mov_rel(-11);
//
//
//
//        //if (ASIGetVideoData(0, img_data, size, -1) != ASI_SUCCESS)
//        //{
//        //    std::cout << "Error getting image from camera!\n";
//        //};
//        //cv::Mat img = cv::Mat(height, width, CV_8U, img_data);
//        //int location = sharpness(img, 0.5);
//
//        ////This line produces the error messages; not sure how to stop this. 
//        //cv::Point p1(location, 0), p2(location, 979);
//        //cv::line(img, p1, p2, cv::Scalar(0, 0, 255), 2);
//        //cv::imwrite("C:\\Users\\HVI\\Desktop\\HVI_Images\\" + FileName + "\\img" + std::to_string(i) + "_" + std::to_string(location) + ".png", img);
//
//
//        //if (moved == 0 && abs(location - previous) > (400 * scale)) { //Using 400, double of 200, because more pixels are available
//        //    //Blinking
//        //    std::cout << "Frame ignored\n";
//        //}
//
//        //else {
//
//        //    if (abs(location - center) < tol) {
//        //        std::cout << "No movement, centered\n";
//        //        moved = 0;
//        //    }
//
//        //    else if (abs(location - center) < (200 * scale)) {
//        //        if (location > center) {
//        //            mov_rel(0.05);
//        //            std::cout << "Moved left, zone 1\n";
//        //        }
//        //        else {
//        //            mov_rel(-0.05);
//        //            std::cout << "Moved right, zone 1\n";
//        //        }
//        //        moved = 1;
//        //    }
//
//        //    else if (abs(location - center) < (300 * scale)) {
//        //        if (location > center) {
//        //            mov_rel(0.2);
//        //            std::cout << "Moved left, zone 2\n";
//        //        }
//        //        else {
//        //            mov_rel(-0.2);
//        //            std::cout << "Moved right, zone 2\n";
//        //        }
//        //        moved = 1;
//        //    }
//
//        //    else if (abs(location - center) < (500 * scale)) {
//        //        if (location > center) {
//        //            mov_rel(0.3);
//        //            std::cout << "Moved left, zone 3\n";
//        //        }
//        //        else {
//        //            mov_rel(-0.3);
//        //            std::cout << "Moved right, zone 3\n";
//        //        }
//        //        moved = 1;
//        //    }
//
//        //    else {
//        //        if (location > center) {
//        //            mov_rel(0.5);
//        //            std::cout << "Moved left, zone 4\n";
//        //        }
//        //        else {
//        //            mov_rel(-0.5);
//        //            std::cout << "Moved right, zone 4\n";
//        //        }
//        //        moved = 1;
//        //    }
//
//
//        //}
//        //previous = location;
//        ////       Sleep(30);
//
//    }
//    auto t2 = high_resolution_clock::now();
//
//    /* Getting number of milliseconds, seconds as an integer. */
//    auto ms_int = duration_cast<milliseconds>(t2 - t1);
//    auto s_int = duration_cast<seconds>(t2 - t1);
//
//    std::cout << ms_int.count() << "ms\n";
//    std::cout << s_int.count() << "s\n";
//
//    //ASIStopVideoCapture(0);
//    //if (ASICloseCamera(0) == ASI_SUCCESS)
//    //{
//    //    std::cout << "closed\n";
//    //}
//    close_motor();
//
//    return 0;
//}
//
//// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
//// Debug program: F5 or Debug > Start Debugging menu
//
//// Tips for Getting Started: 
////   1. Use the Solution Explorer window to add/manage files
////   2. Use the Team Explorer window to connect to source control
////   3. Use the Output window to see build output and other messages
////   4. Use the Error List window to view errors
////   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
////   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
