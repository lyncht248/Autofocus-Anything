#ifndef AUTOFOCUS_H
#define AUTOFOCUS_H

#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include "mainwindow.hpp"
#include "lens.hpp"
#include "tiltedcam.hpp"
#include <atomic>
#include <thread>

// extern unsigned char* img_read_buf; // Video capture. Accessed by T1.
// extern unsigned char* img_transfer_buf; // Transfer. Accessed by T1 and T2.
// extern unsigned char* img_calc_buf; // Calculation. Accessed by T2.
//extern std::mutex mtx; 
extern int imgcount;
extern bool bHoldFocus;
extern bool bFindFocus;
extern bool bResetLens;
extern bool bNewMoveRel;
extern int desiredLocBestFocus;
extern std::atomic<double> mmToMove;

class autofocus { //This class handles autofocusing
  public:
  //Called by int main()
  autofocus();
  ~autofocus();
  bool initialize(); //This was the constructor, but it needs to be called after the GUI is initialized

  void run(); 

  //thread for capturing video from the tilted camera
  void capturevideo();

  //static void crapGUI();

  //computes the location of best-focus
  int computeBestFocus (cv::Mat image, int imgHeight, int imgWidth);
  int computeBestFocusReduced(cv::Mat image, int imgHeight, int imgWidth);
  int computeBestFocusVeryReduced(cv::Mat image, int imgHeight, int imgWidth);
  void adjust_bestFocus(int val);

  void setPGain(double gain);
  double getPGain() const;

  // Add a method to get the lens object
  lens& getLens() { return lens1; }
  tiltedcam& getTiltedCam() { return tiltedcam1; }

  friend class AutofocusTest;
  friend class DeviceCalibrationTest;

  private:
  //computes the sharpness curve along the horizontal of the image using a sharpness algorithm
  std::vector<double> computesharpness(cv::Mat image, int imgHeight, int imgWidth, int kernel);
 
  //computes the sharpness score for a given portion of imagedata
  cv::Scalar robertscross(cv::Mat imagedata); 
  cv::Scalar tenengrad(cv::Mat imagedata);
  cv::Scalar vollath(cv::Mat imagedata);
  cv::Scalar canny(cv::Mat imagedata);

 
  //fits a normal curve to the given curve, which avoids local maxima
  std::vector<double> fitnormalcurve(std::vector<double> sharpnesscurve, double amplitude, double offset, double std_dev_factor);
  std::vector<double> fitnormalcurveBruteForce(std::vector<double> sharpnesscurve, double amplitude, double offset, double std_dev_factor);
  double calculateErrorWithAmplitudeAndOffset(const std::vector<double>& sharpnesscurve, double mean, double amplitude, double offset, double sigma);
  double normpdf(double x, double u, double s); //helper function

  std::atomic<bool> stop_thread; //Controls the autofocus, tilted camera, and lens threads
  std::thread tAutofocus;

  lens lens1;
  tiltedcam tiltedcam1;

  // Pre-allocated matrices for performance optimization
  cv::Mat blurred_preallocated;
  cv::Mat img_x_preallocated;
  cv::Mat img_y_preallocated;
  cv::Mat img_x_squared_preallocated;
  cv::Mat img_y_squared_preallocated;
  cv::Mat sum_xy_preallocated;
  cv::Mat sharpness_float_preallocated;
  cv::Mat imageofinterest_preallocated;
  
  // Pre-allocated Roberts Cross kernels
  cv::Mat roberts_kernelx;
  cv::Mat roberts_kernely;

  // CSV logging
  std::ofstream csvFile;
  std::string csvFilename;

};

#endif // AUTOFOCUS_H