#include "tiltedcam.hpp"
#include "autofocus.hpp"
#include "logfile.hpp"

#include <iostream>
#include <unistd.h>

#include <atomic>
#include <thread>

tiltedcam::tiltedcam() {
    std::cout << "tiltedcam object created" << "\n";

    int iNumofConnectCameras = ASIGetNumOfConnectedCameras();

    if (iNumofConnectCameras > 0) {
        hvigtk_logfile << "Connected to the tilted camera \n";
     }
    else {
        hvigtk_logfile << "Failed to conect to the tilted camera \n";
    }


    if (ASIOpenCamera(0) == ASI_SUCCESS) {
        hvigtk_logfile << "Opened the tilted camera \n";
    }
    else {
        hvigtk_logfile << "Failed to open the tilted camera \n";
    }

    if (ASIInitCamera(0) == ASI_SUCCESS) {
        hvigtk_logfile << "Initialized the tilted camera \n";
    }
    else {
        hvigtk_logfile << "Failed to initialize the tilted camera\n";
    }    

    // Set to high-speed-mode, set gain, and set exposure. Only the first one needs to be uncommented... 
    // TODO: add function to change gain and exposure for ASI ZWO camera, if neccessary. 
    if (ASISetControlValue(0, ASI_HIGH_SPEED_MODE, 1, ASI_FALSE) == ASI_SUCCESS) {
        hvigtk_logfile << "Set tilted camera to highspeed mode (and bAuto=FALSE) \n";

    }
    else {
        hvigtk_logfile << "failed to set tilted camera to high speed mode\n";
    }
    // TODO: Potentially set these to auto (so ASI_TRUE), but then the value has to be the current value, so need to use ASIGetControlValue first.
    //ASISetControlValue(0, ASI_AUTO_MAX_EXP, 15, ASI_TRUE);
    //ASISetControlValue(0,  ASI_EXPOSURE, 15, ASI_TRUE);
    //ASISetControlValue(0, ASI_BANDWIDTHOVERLOAD, 90, ASI_TRUE);

    //Binning so that image recieved is 640x480. TODO: Try recieving images at 1280x960, as normal, since binning may cause data loss
    if (ASISetROIFormat(0, 1280, 960, 1, ASI_IMG_RAW8) == ASI_SUCCESS) {
    //if (ASISetROIFormat(0, 320, 240, 1, ASI_IMG_RAW8) == ASI_SUCCESS) {
        hvigtk_logfile << "Set tilted camera image to Raw8 and 1 bin, with image size 1280x960 \n";
    }
    else {
        hvigtk_logfile << "Failed to set ROI format for tilted camera\n";
    }; 

    usleep(100000);
    
    if (ASIStartVideoCapture(0) == ASI_SUCCESS) {
        hvigtk_logfile << "Started tilted camera video capture\n";
    }
    else {
        hvigtk_logfile << "Failed to start video capture\n";
    }; 

}


// tiltedcam::settings tiltedcam::getsettings() {
//   //DUMMY
//   settings VarSettings; //new variable of type settings
//   ASI_BOOL asiAuto = ASI_FALSE;
//   ASIGetControlValue(0, ASI_GAIN, &VarSettings.gain, &asiAuto);
//   if(asiAuto) {std::cout << "WARNING: tilted camera gain is set to Automatically adjust\n";};
//   ASIGetControlValue(0, ASI_EXPOSURE, &VarSettings.exposure, &asiAuto);
//   if(asiAuto) {std::cout << "WARNING: tilted camera exposure is set to Automatically adjust\n";};
//   ASIGetControlValue(0, ASI_GAMMA, &VarSettings.gamma, &asiAuto);
//   if(asiAuto) {std::cout << "WARNING: tilted camera gamma is set to Automatically adjust\n";};
//   ASIGetControlValue(0, ASI_HIGH_SPEED_MODE, &VarSettings.highspeedmode, &asiAuto);
//   if(asiAuto) {std::cout << "WARNING: tilted camera high speed mode is set to Automatically adjust\n";};

//   ASIGetROIFormat(0, &VarSettings.imgWidth, &VarSettings.imgHeight, &VarSettings.bins, &VarSettings.imagetype);
//   std::cout << "Got camera settings" << "\n";
//   std::cout << VarSettings.imgWidth << "\n";
//   std::cout << VarSettings.imgHeight << "\n";

//   return VarSettings;
// }

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


tiltedcam::~tiltedcam() {
    if (ASIStopVideoCapture(0) == ASI_SUCCESS) {
        hvigtk_logfile << "Stopped tilted camera video capture\n";
    }
    else {
        hvigtk_logfile << "Failed to stop video capture\n";
    }; 

    if (ASICloseCamera(0) == ASI_SUCCESS) {
        hvigtk_logfile << "Tilted camera is closed\n";
    } 
    else {
        hvigtk_logfile << "Failed to close tilted camera\n";
    }
}



// Getters
long tiltedcam::getGain() {
    long gain; //new variable of type settings
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetControlValue(0, ASI_GAIN, &gain, &asiAuto);
    hvigtk_logfile << "Got gain = " << gain << "\n";
    return gain;
}
long tiltedcam::getExposure() {
    long exposure; //new variable of type settings
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetControlValue(0, ASI_EXPOSURE, &exposure, &asiAuto);
    hvigtk_logfile << "Got exposure = " << exposure << "\n";
    return exposure;
}
long tiltedcam::getGamma() {
    long gamma; //new variable of type settings
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetControlValue(0, ASI_GAMMA, &gamma, &asiAuto);
    hvigtk_logfile << "Got gamma = " << gamma << "\n";
    return gamma;
}
long tiltedcam::getHighSpeedMode() {
    long highSpeedMode; //new variable of type settings
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetControlValue(0, ASI_HIGH_SPEED_MODE, &highSpeedMode, &asiAuto);
    hvigtk_logfile << "Got high speed mode = " << highSpeedMode << "\n";
    return highSpeedMode;
}
int tiltedcam::getImageWidth() {
    int width; //new variable of type settings
    int height;
    int bins;
    ASI_IMG_TYPE imageType;
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetROIFormat(0, &width, &height, &bins, &imageType);
    hvigtk_logfile << "Got tiltedcam image width = " << width << "\n";
    return width;
}
int tiltedcam::getImageHeight() {
    int width; //new variable of type settings
    int height;
    int bins;
    ASI_IMG_TYPE imageType;
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetROIFormat(0, &width, &height, &bins, &imageType);
    hvigtk_logfile << "Got tiltedcam image height = " << height << "\n";
    return height;
}
int tiltedcam::getBins() {
    int width; //new variable of type settings
    int height;
    int bins;
    ASI_IMG_TYPE imageType;
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetROIFormat(0, &width, &height, &bins, &imageType);
    hvigtk_logfile << "Got tiltedcam image bins = " << bins << "\n";
    return bins;
}
ASI_IMG_TYPE tiltedcam::getImageType() {
    int width; //new variable of type settings
    int height;
    int bins;
    ASI_IMG_TYPE imageType;
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetROIFormat(0, &width, &height, &bins, &imageType);
    hvigtk_logfile << "Got tiltedcam image type = " << imageType << "\n";
    return imageType;
}


//Setters
void tiltedcam::setGain(long newGain) {
    if (ASISetControlValue(0, ASI_GAIN, newGain, ASI_FALSE) == ASI_SUCCESS) {
        hvigtk_logfile << "Set tilted camera's gain (and bAuto=FALSE) \n";
    }
    else {
        hvigtk_logfile << "failed to change setting of tilted camera \n";
    }
}
void tiltedcam::setExposure(long newExposure) {
    if (ASISetControlValue(0, ASI_EXPOSURE, newExposure, ASI_FALSE) == ASI_SUCCESS) {
        hvigtk_logfile << "Set tilted camera's gain (and bAuto=FALSE) \n";
    }
    else {
        hvigtk_logfile << "failed to change setting of tilted camera \n";
    }
}
void tiltedcam::setGamma(long newGamma) {
    if (ASISetControlValue(0, ASI_GAMMA, newGamma, ASI_FALSE) == ASI_SUCCESS) {
        hvigtk_logfile << "Set tilted camera's gain (and bAuto=FALSE) \n";
    }
    else {
        hvigtk_logfile << "failed to change setting of tilted camera \n";
    }
}
void tiltedcam::setHighSpeedMode(long newHighSpeedMode) {
    if (ASISetControlValue(0, ASI_HIGH_SPEED_MODE, newHighSpeedMode, ASI_FALSE) == ASI_SUCCESS) {
        hvigtk_logfile << "Set tilted camera's gain (and bAuto=FALSE) \n";
    }
    else {
        hvigtk_logfile << "failed to change setting of tilted camera \n";
    }
}


