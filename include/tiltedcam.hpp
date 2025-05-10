#ifndef TILTEDCAM_H
#define TILTEDCAM_H

#include "ASICamera2.h"
#include <mutex>
#include <atomic>
#include <thread>


class tiltedcam { //This object handles the tilted camera
  public:

  tiltedcam(); // Add constructor
  bool initialize(); //Connects to, initializes, and opens a ZWO ASI camera. Then it sets relevant properties and starts a video stream. Returns 1 if successful, 0 if not.
  ~tiltedcam(); // Stops video stream and closes the camera

  // Getters
  long getGain();
  long getExposure();
  long getGamma();
  long getHighSpeedMode ();
  int getImageWidth();
  int getImageHeight();
  int getBins();
  ASI_IMG_TYPE getImageType();

  // Setters
  void setGain(long newGain);
  void setExposure(long newExposure);
  void setGamma(long newGamma);
  void setHighSpeedMode(long newHighSpeedMode);

  // // Gets all the settings at once (ie. imgWidth, imgHeight, bins) from the camera
  // settings getsettings();

  // Pulls images from camera
  unsigned char* capturevideowrapper(const long img_size);

  void startCaptureThread(); // Start a thread to capture video
  void stopCaptureThread();  // Stop the thread
  bool getLatestFrame(unsigned char* destination, long size); // Get the most recent frame

  private: 
  
  // struct settings {
  //   long gain;
  //   long exposure;
  //   long gamma;
  //   long highspeedmode;
  //   int imgWidth;
  //   int imgHeight;
  //   int bins;
  //   ASI_IMG_TYPE imagetype;
  // };

  void captureThreadFunc(); // Thread function for video capture
  
  // Buffer management
  unsigned char* m_captureBuffer = nullptr;
  unsigned char* m_processingBuffer = nullptr;
  std::mutex m_bufferMutex;
  std::atomic<bool> m_newFrameAvailable{false};
  std::atomic<bool> m_stopThread{false};
  std::thread m_captureThread;
  long m_bufferSize = 0;
  bool m_threadRunning = false;
};

#endif // TILTEDCAM_H