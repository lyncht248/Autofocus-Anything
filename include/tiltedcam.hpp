#ifndef TILTEDCAM_H
#define TILTEDCAM_H

#include "ASICamera2.h"
#include <mutex>
#include <atomic>
#include <thread>

/**
 * @class tiltedcam
 * @brief Handles the tilted camera operations and video capture
 * 
 * This class provides an interface to control a ZWO ASI camera, which is mounted in a tilted position.
 * It manages camera initialization, configuration, and video capture through a dedicated thread.
 */
class tiltedcam { //This object handles the tilted camera
  public:

  /**
   * @brief Default constructor
   * 
   * Initializes member variables to their default values.
   */
  tiltedcam(); // Add constructor
  
  /**
   * @brief Initializes the camera
   * 
   * Connects to, initializes, and opens a ZWO ASI camera. Then it sets relevant 
   * properties and starts a video stream.
   * 
   * @return true if initialization was successful, false otherwise
   */
  bool initialize(); 
  
  /**
   * @brief Destructor
   * 
   * Stops video stream, terminates capture thread, and closes the camera.
   */
  ~tiltedcam();
  // Getters
  /**
   * @brief Get the current gain setting
   * @return Current gain value
   */
  long getGain();
  
  /**
   * @brief Get the current exposure setting
   * @return Current exposure value in microseconds
   */
  long getExposure();
  
  /**
   * @brief Get the current gamma setting
   * @return Current gamma value
   */
  long getGamma();
  
  /**
   * @brief Returns whether the camera is in high speed mode
   * @return Current high speed mode value (0 = off, 1 = on)
   */
  long getHighSpeedMode();
  
  /**
   * @brief Get the current image width
   * @return Width of the image in pixels
   */
  int getImageWidth();
  
  /**
   * @brief Get the current image height
   * @return Height of the image in pixels
   */
  int getImageHeight();
  
  /**
   * @brief Get the current binning setting
   * @return Current binning value
   */
  int getBins();
  
  /**
   * @brief Get the current image type
   * @return Current image type (ASI_IMG_RAW8, etc.)
   */
  ASI_IMG_TYPE getImageType();

  // Setters
  /**
   * @brief Set the camera gain
   * @param newGain The gain value to set
   */
  void setGain(long newGain);
  
  /**
   * @brief Set the camera exposure in microseconds
   * @param newExposure The exposure value to set in microseconds
   */
  void setExposure(long newExposure);
  
  /**
   * @brief Set the camera gamma
   * @param newGamma The gamma value to set
   */
  void setGamma(long newGamma);
  
  /**
   * @brief Set the camera high speed mode value(0 = off, 1 = on)
   * @param newHighSpeedMode The high speed mode value (0 = off, 1 = on)
   */
  void setHighSpeedMode(long newHighSpeedMode);

  /**
   * @brief Start the video capture thread
   * 
   * Allocates necessary buffers and starts a background thread
   * that continuously captures frames from the camera.
   */
  void startCaptureThread(); // Start a thread to capture video
  
  /**
   * @brief Stop the video capture thread
   * 
   * Signals the capture thread to stop, joins it, and frees allocated buffers.
   */
  void stopCaptureThread();  // Stop the thread
  
  /**
   * @brief Gets the most recent captured frame from the background capture thread
   * 
   * Copies the latest frame to the provided destination buffer if available.
   * 
   * @param destination Pointer to the destination buffer
   * @param size Size of the destination buffer in bytes
   * @return true if a new frame was available and copied, false otherwise
   */
  bool getLatestFrame(unsigned char* destination, long size); // Get the most recent frame

  private: 


  /**
   * @brief Thread function for continuous video capture
   * 
   * This function runs in a separate thread and continuously captures
   * frames from the camera, making them available for retrieval.
   */
  void captureThreadFunc(); // Thread function for video capture
  
  // Buffer management
  unsigned char* m_captureBuffer = nullptr;   ///< Buffer for capturing frames directly from camera
  unsigned char* m_processingBuffer = nullptr; ///< Buffer for storing the latest frame for processing
  std::mutex m_bufferMutex;                   ///< Mutex for thread-safe buffer access
  std::atomic<bool> m_newFrameAvailable{false}; ///< Flag indicating if a new frame is available
  std::atomic<bool> m_stopThread{false};       ///< Flag to signal thread termination
  std::thread m_captureThread;                ///< Thread handle for the capture thread
  long m_bufferSize = 0;                      ///< Size of the image buffers in bytes
  bool m_threadRunning = false;               ///< Flag indicating if the capture thread is running
};

#endif // TILTEDCAM_H