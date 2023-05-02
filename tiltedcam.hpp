#ifndef TILTEDCAM_H
#define TILTEDCAM_H

#include "ASICamera2.h"


class tiltedcam { //This object handles the tilted camera
  public:

  // Connects to, initializes, and opens a ZWO ASI camera. Then it sets relevant properties and starts a video stream. Returns 1 if successful, 0 if not.
  int initcam();


  struct settings {
    long gain;
    long exposure;
    long gamma;
    long highspeedmode;
    int imgWidth;
    int imgHeight;
    int bins;
    ASI_IMG_TYPE imagetype;
  };

  // Gets the settings (ie. imgWidth, imgHeight, bins) from the camera
  settings getsettings();

  // Pulls images from camera
  unsigned char* capturevideowrapper(const long img_size);

  // Stops video stream and closes the camera
  int closecam();
};

#endif // TILTEDCAM_H