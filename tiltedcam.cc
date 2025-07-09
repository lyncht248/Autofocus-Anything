#include "tiltedcam.h"
#include "autofocus.h"

#include "ASICamera2.h"

#include <iostream>
#include <unistd.h>

#include <atomic>
#include <thread>

int tiltedcam::initcam() {
    int iNumofConnectCameras = ASIGetNumOfConnectedCameras();
    if (iNumofConnectCameras > 0) {
          std::cout << "Connected to the camera(s)\n";
     }
     else {
          std::cout << "Failed to connect to the camera(s)";
          return -1;
    }

     if (ASICloseCamera(0) == ASI_SUCCESS)
     {
         std::cout << "closed\n";
     }

    // ASIOpenCamera(0);
    // ASIInitCamera(0);

    //Where binning (potentially) occurs. I'd hard-code this, personally...
    // if (scale == 0.5) {
    //     ASISetROIFormat(0, 640, 480, 2, ASI_IMG_RAW8); //Switches to 640*480 at hardware level
    // }
    // else if (scale == 1) {
    //     ASISetROIFormat(0, 1280, 960, 1, ASI_IMG_RAW8);
    // }

    //Set to high-speed-mode, set gain, and set exposure. Only the first one needs to be uncommented... 
    //TODO: add function to change gain and exposure for ASI ZWO camera, if neccessary. 
    //ASISetControlValue(0, ASI_HIGH_SPEED_MODE, 1, bAuto);
    //ASISetControlValue(0, ASI_AUTO_MAX_EXP, 15, bAuto);
    //ASISetControlValue(0,  ASI_EXPOSURE, 15, bAuto);
    //ASISetControlValue(0, ASI_BANDWIDTHOVERLOAD, 90, bAuto);

    //ASI start video capture
    //ASIStartVideoCapture(0);

    // dummy functions for testing
    // std::cout << "TiltedCam connected" << "\n";
    // std::cout << "TiltedCam opened" << "\n";
    // std::cout << "TiltedCam settings set" << "\n";
    // std::cout << "TiltedCam video stream started" << "\n";
    return 1;
}

tiltedcam::settings tiltedcam::getsettings() {
  //DUMMY
  std::cout << "Got camera settings TEST" << "\n";
  settings VarSettings; //new variable of type settings
  VarSettings.imgHeight = 480;
  VarSettings.imgWidth = 640;
  VarSettings.bin = 2;
  return VarSettings;
}

//currently unused due to 'Segmentation fault (core dumped)' error when I call this function
unsigned char* tiltedcam::capturevideowrapper(const long img_size) {
    unsigned char* img_get_buf = (unsigned char*)malloc(img_size);
    //ASI_ERROR_CODE = ASIGetVideoData(0, img_get_buf, img_size, -1);  //pulls image data from camera and saves as img_data
    std::cout << "got an image" << "\n";
    return img_get_buf;

    // while (bAutofocusing) {
    //     if (bMotorMoving) {
    //     //do nothing
    //     }
    //     else {
    //         {
    //             std::lock_guard<std::mutex> lck{ mtx };
    //             auto temp = img_data_filled_read_buf;
    //             img_data_filled_read_buf = img_data_next_read_buf;
    //             img_data_next_read_buf = temp;
    //             bNewImage = 1;
    //         }
    //         imgCount++;

    //         if (ASI_ERROR_CODE != 0) {
    //             std::cout << "Error getting image from camera!\n";
    //     }
    // }

    //DUMMY 
    // while(bAutofocusing) {
    //     // Reading in an image for testing
    //     cv::Mat image;
    //     image = cv::imread("/home/tom/projects/autofocus_v9/test/NoLine/img2_364.png", 1 );
    //     if ( !image.data )
    //     {
    //         printf("No image data \n");
    //     }

    //     q.enqueue(image.data);
    //     std::cout << "got an image" << "\n";
    //     usleep(16670); //is in microseconds
    //     {
    //         std::lock_guard<std::mutex> lck{ mtx };
    //         // auto temp = img_transfer_buf;
    //         // img_transfer_buf = img_read_buf;
    //         // img_read_buf = temp;
    //         bNewImage = 1;
    //     }
    // }
    // return;
}


int tiltedcam::closecam() {
    // ASIStopVideoCapture(0);
    // if (ASICloseCamera(0) == ASI_SUCCESS)
    // {
    //     std::cout << "closed\n";
    // }

    //DUMMY
    std::cout << "Camera closed" << "\n";
    return 1;
}