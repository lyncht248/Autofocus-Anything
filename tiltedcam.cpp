#include "tiltedcam.hpp"
#include "autofocus.hpp"

#include <iostream>
#include <unistd.h>

#include <atomic>
#include <thread>

int tiltedcam::initcam() {
    int iNumofConnectCameras = ASIGetNumOfConnectedCameras();
    if (iNumofConnectCameras > 0) {
          std::cout << "Connected to the tilted camera\n";
     }
     else {
          //std::cout << "Failed to connect to the tilted camera\n";
          return 0;
    }


    if (ASIOpenCamera(0) == ASI_SUCCESS) {
         std::cout << "tilted camera opened\n";
    }
    else {
        //std::cout << "tilted camera failed to open\n";
        return 0;
    }

    if (ASIInitCamera(0) == ASI_SUCCESS) {
        std::cout << "Tilted camera initilized\n";
    }
    else {
        std::cout << "Failed to initialize the tilted camera\n";
        return 0;
    }    

    // Set to high-speed-mode, set gain, and set exposure. Only the first one needs to be uncommented... 
    // TODO: add function to change gain and exposure for ASI ZWO camera, if neccessary. 
    if (ASISetControlValue(0, ASI_HIGH_SPEED_MODE, 1, ASI_FALSE) == ASI_SUCCESS) {

        std::cout << "Set tilted camera to highspeed mode (and bAuto=FALSE) \n";

    }
    else {
        std::cout << "failed to set tilted camera to high speed mode\n";
        return 0;
    }
    // TODO: Potentially set these to auto (so ASI_TRUE), but then the value has to be the current value, so need to use ASIGetControlValue first.
    //ASISetControlValue(0, ASI_AUTO_MAX_EXP, 15, ASI_TRUE);
    //ASISetControlValue(0,  ASI_EXPOSURE, 15, ASI_TRUE);
    //ASISetControlValue(0, ASI_BANDWIDTHOVERLOAD, 90, ASI_TRUE);

    //Binning so that image recieved is 640x480. TODO: Try recieving images at 1280x960, as normal, since binning may cause data loss
    if (ASISetROIFormat(0, 1280, 960, 1, ASI_IMG_RAW8) == ASI_SUCCESS) {
    //if (ASISetROIFormat(0, 320, 240, 1, ASI_IMG_RAW8) == ASI_SUCCESS) {
        std::cout << "Set tilted camera image to Raw8 and 1 bin, so image recieved is 320x240\n";
    }
    else {
        std::cout << "Failed to set ROI format for tilted camera\n";
        return 0;
    }; 

    usleep(100000);

    
    if (ASIStartVideoCapture(0) == ASI_SUCCESS) {
        std::cout << "Started tilted camera video capture\n";
    }
    else {
        std::cout << "Failed to start video capture\n";
        return 0;
    }; 

    return 1;
}

tiltedcam::settings tiltedcam::getsettings() {
  //DUMMY
  settings VarSettings; //new variable of type settings
  ASI_BOOL asiAuto = ASI_FALSE;
  ASIGetControlValue(0, ASI_GAIN, &VarSettings.gain, &asiAuto);
  if(asiAuto) {std::cout << "WARNING: tilted camera gain is set to Automatically adjust\n";};
  ASIGetControlValue(0, ASI_EXPOSURE, &VarSettings.exposure, &asiAuto);
  if(asiAuto) {std::cout << "WARNING: tilted camera exposure is set to Automatically adjust\n";};
  ASIGetControlValue(0, ASI_GAMMA, &VarSettings.gamma, &asiAuto);
  if(asiAuto) {std::cout << "WARNING: tilted camera gamma is set to Automatically adjust\n";};
  ASIGetControlValue(0, ASI_HIGH_SPEED_MODE, &VarSettings.highspeedmode, &asiAuto);
  if(asiAuto) {std::cout << "WARNING: tilted camera high speed mode is set to Automatically adjust\n";};

  ASIGetROIFormat(0, &VarSettings.imgWidth, &VarSettings.imgHeight, &VarSettings.bins, &VarSettings.imagetype);
  std::cout << "Got camera settings" << "\n";
  std::cout << VarSettings.imgWidth << "\n";
  std::cout << VarSettings.imgHeight << "\n";

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
    if (ASICloseCamera(0) == ASI_SUCCESS) {
        std::cout << "Tilted camera is closed\n";
    } 
    else {
        std::cout << "Failed to close tilted camera\n";
        return -1;
    }
    std::cout << "Camera closed" << "\n";
    return 1;
}