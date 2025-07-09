#ifndef TILTEDCAM_H
#define TILTEDCAM_H


class tiltedcam { //This object handles the tilted camera
  public:

  // Initializes and opens the camera
  int initcam();


  struct settings {
    int imgWidth;
    int imgHeight;
    int bin;
    //ASI_IMG_TYPE image_type;
  };

  // Gets the settings (ie. imgWidth, imgHeight, bins) from the camera
  settings getsettings();

  // Pulls images from camera
  unsigned char* capturevideowrapper(const long img_size);

  // Stops video stream and closes the camera
  int closecam();
};

#endif // TILTEDCAM_H