/**
 * @file autofocus.hpp
 * @brief Defines the autofocus class for managing the autofocusing process
 * using the lens and tilted camera.
 */

#ifndef AUTOFOCUS_H
#define AUTOFOCUS_H

#include "lens.hpp"
#include "mainwindow.hpp"
#include "tiltedcam.hpp"
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <sstream>
#include <thread>

#include "settings.hpp"

// extern unsigned char* img_read_buf; // Video capture. Accessed by T1.
// extern unsigned char* img_transfer_buf; // Transfer. Accessed by T1 and T2.
// extern unsigned char* img_calc_buf; // Calculation. Accessed by T2.
// extern std::mutex mtx;
extern int imgcount;
extern bool bHoldFocus;
extern bool bFindFocus;
extern bool bResetLens;
extern bool bNewMoveRel;
extern int desiredLocBestFocus;
extern std::atomic<double> mmToMove;

/**
 * @class autofocus
 * @brief Handles the autofocusing process using the lens and tilted camera.
 *
 * The autofocus class manages the initialization, running, and computation of
 * the best focus using various sharpness algorithms and optimization
 * techniques.
 */
class autofocus { // This class handles autofocusing
public:
  /**
   * @brief Constructor for the autofocus class.
   *
   * Initializes the lens and tilted camera objects.
   */
  autofocus();

  /**
   * @brief Destructor for the autofocus class.
   *
   * Cleans up resources, stops threads, and ensures proper shutdown.
   */
  ~autofocus();

  /**
   * @brief Initializes the autofocus system.
   *
   * Sets up the lens and tilted camera, pre-allocates matrices, and initializes
   * benchmark files.
   * @return True if initialization is successful, false otherwise.
   */
  bool initialize(); // This was the constructor, but it needs to be called
                     // after the GUI is initialized

  /**
   * @brief Runs the autofocus process.
   *
   * Starts the camera capture thread, computes focus, and handles PID control.
   */
  void run();

  /**
   * @brief Captures video from the tilted camera.
   *
   * This method is used by the autofocus thread to continuously capture video
   * frames.
   */
  void capturevideo();

  /**
   * @brief Reloads settings from the settings file
   *
   * Updates the derivative gain with the current
   * value from the settings file. This allows runtime updates without
   * restarting.
   */
  void reloadSettings();




  double computeBestFocusGSL(cv::Mat image, int imgHeight, int imgWidth);
  double computeBestFocusGaussian(cv::Mat image, int imgHeight, int imgWidth);



  /**
   * @brief Adjusts the best focus location.
   *
   * Updates the desired focus location based on the given value.
   * @param val Adjustment value.
   */
  void adjust_bestFocus(int val);

  /**
   * @brief Sets the proportional gain for the PID controller.
   *
   * @param gain Proportional gain value.
   */
  void setPGain(double gain);

  /**
   * @brief Gets the proportional gain for the PID controller.
   *
   * @return Proportional gain value.
   */
  double getPGain() const;

  // Peak locator selection
  enum class PeakLocator {
    CenterOfMass,
    PowerCOM,
    TruncatedGaussian,
    GaussianFitBrute
  };




  // Set the assumed Gaussian sigma in **full-resolution pixels**. If <= 0, we
  // auto-estimate from σ_a.
  void setTruncGaussSigmaFullRes(double sigmaPx);
  // How many points to average at each edge to estimate f(a), f(b). (>=1)
  void setTruncGaussEdgeAveraging(int count);

  // Exponential smoothing of the *returned* peak (0<beta<=1). beta=1 disables
  // smoothing.
  void setPeakSmoothing(double beta);

  void SaveSharpnessTxt(const cv::Mat &resized, const cv::Mat& sharpness_float, 
    const std::vector<double> &columnMeans, const std::vector<double>& sharpnessCurve, int& increment);
  void SaveImagesPnG(cv::Mat &resized, double locBestFocusDouble, double A, double C, int& increment2);

  /**
   * @brief Gets the lens object.
   *
   * Provides access to the lens object for external use.
   * @return Reference to the lens object.
   */
  lens &getLens() { return lens1; }

  /**
   * @brief Gets the tilted camera object.
   *
   * Provides access to the tilted camera object for external use.
   * @return Reference to the tilted camera object.
   */
  tiltedcam &getTiltedCam() { return tiltedcam1; }

  friend class AutofocusTest;
  friend class DeviceCalibrationTest;

private:
  /**
   * @brief Computes the sharpness curve along the horizontal of the image.
   *
   * Uses a sharpness algorithm to calculate the sharpness curve.
   * @param image Input image.
   * @param imgHeight Height of the image.
   * @param imgWidth Width of the image.
   * @param kernel Kernel size for the sharpness algorithm.
   * @return Sharpness curve as a vector of doubles.
   */
  std::vector<double> computesharpness(cv::Mat image, int imgHeight,
                                       int imgWidth, int kernel);

  /**
   * @brief Computes the sharpness score using the Roberts Cross operator.
   *
   * @param imagedata Input image data.
   * @return Sharpness score as a scalar.
   */
  cv::Scalar robertscross(cv::Mat imagedata);

  /**
   * @brief Computes the sharpness score using the Tenengrad method.
   *
   * @param imagedata Input image data.
   * @return Sharpness score as a scalar.
   */
  cv::Scalar tenengrad(cv::Mat imagedata);

  /**
   * @brief Computes the sharpness score using the Vollath method.
   *
   * @param imagedata Input image data.
   * @return Sharpness score as a scalar.
   */
  cv::Scalar vollath(cv::Mat imagedata);

  /**
   * @brief Computes the sharpness score using the Canny edge detection method.
   *
   * @param imagedata Input image data.
   * @return Sharpness score as a scalar.
   */
  cv::Scalar canny(cv::Mat imagedata);

  /**
   * @brief Fits a normal curve to the given sharpness curve.
   *
   * Avoids local maxima and smooths the sharpness curve.
   * @param sharpnesscurve Input sharpness curve.
   * @param amplitude Amplitude of the curve.
   * @param offset Offset of the curve.
   * @param std_dev_factor Standard deviation factor for the curve.
   * @return Fitted sharpness curve as a vector of doubles.
   */
  std::vector<double> fitnormalcurve(std::vector<double> sharpnesscurve,
                                     double amplitude, double offset,
                                     double std_dev_factor);


  /**
   * @brief Calculates the error with amplitude and offset for the sharpness
   * curve.
   *
   * @param sharpnesscurve Input sharpness curve.
   * @param mean Mean value of the curve.
   * @param amplitude Amplitude of the curve.
   * @param offset Offset of the curve.
   * @param sigma Standard deviation of the curve.
   * @return Error value.
   */
  double calculateErrorWithAmplitudeAndOffset(
      const std::vector<double> &sharpnesscurve, double mean, double amplitude,
      double offset, double sigma);

  /**
   * @brief Computes the normal probability density function.
   *
   * @param x Input value.
   * @param u Mean value.
   * @param s Standard deviation.
   * @return Probability density value.
   */
  double normpdf(double x, double u, double s); // helper function

  /**
   * @brief Calculates the center of mass for the given curve.
   *
   * @param curve Input curve.
   * @return Center of mass value.
   */
  double findCenterOfMass(const std::vector<double> &curve);

  /**
   * @brief Stops the autofocus thread.
   *
   * Controls the autofocus, tilted camera, and lens threads.
   */
  std::atomic<bool>
      stop_thread; // Controls the autofocus, tilted camera, and lens threads

  /**
   * @brief Thread for running the autofocus process.
   */
  std::thread tAutofocus;

  /**
   * @brief Lens object for controlling the lens.
   */
  lens lens1;

  /**
   * @brief Tilted camera object for capturing images.
   */
  tiltedcam tiltedcam1;

  /**
   * @brief Pre-allocated matrices for performance optimization.
   */
  cv::Mat blurred_preallocated;
  cv::Mat img_x_preallocated;
  cv::Mat img_y_preallocated;
  cv::Mat img_x_squared_preallocated;
  cv::Mat img_y_squared_preallocated;
  cv::Mat sum_xy_preallocated;
  cv::Mat sharpness_float_preallocated;
  cv::Mat imageofinterest_preallocated;

  /**
   * @brief Pre-allocated Roberts Cross kernels.
   */
  cv::Mat roberts_kernelx;
  cv::Mat roberts_kernely;


  /**
   * @brief Stores the center of mass for visualization.
   */
  double lastCenterOfMass = -1.0; // Store center of mass for visualization

  /**
   * @brief CSV file for logging.
   */
  std::ofstream csvFile;

  /**
   * @brief Filename for the CSV file.
   */
  std::string csvFilename;

  /**
   * @brief Mutex for thread-safe CSV operations.
   */
  std::mutex csvMutex; // Thread-safe CSV operations

  /**
   * @brief Proportional gain for the PID controller.
   */
  double Kp;

  /**
   * @brief Derivative gain for the PID controller.
   */
  double Kd;

  PeakLocator m_peakLocator = PeakLocator::GaussianFitBrute;

  // Power-COM parameters
  int m_powerExponent = 4;          // default ^4
  bool m_powerCOMQuadRefine = true; // parabolic refine on by default

  // TruncatedGaussian parameters
  double m_sigmaPxFullRes =
      -1.0;               // <=0 => auto-estimate σ from σ_a (apparent var)
  int m_edgeAvgCount = 5; // points to average at each edge

  // Optional output smoothing (shared)
  double m_peakEmaBeta = 1.0; // 1.0 => smoothing disabled
  double m_prevMuReduced = std::numeric_limits<double>::quiet_NaN();



  Settings settings; ///< Settings object to manage configuration
};

#endif // AUTOFOCUS_H