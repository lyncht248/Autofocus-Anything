#include "autofocus.hpp"
// #include "tiltedcam.hpp"
// #include "lens.hpp"
#include "ASICamera2.h" //TODO: Remove this when you move capturevideo() to tiltedcam.cc
#include "logfile.hpp"
#include "main.hpp"
#include "mainwindow.hpp"
// #include "pid.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <mutex>
#include <numeric>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sched.h>
#include <sstream>
#include <thread>
#include <unistd.h>
// #include <gtk/gtk.h>

#include <stdlib.h>
#include <stdio.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlinear.h>

#include <Eigen/Dense>
#include <unsupported/Eigen/LevenbergMarquardt>


// #include <stdlib.h>
// #include <stdio.h>
// #include <gsl/gsl_vector.h>
// #include <gsl/gsl_matrix.h>
// #include <gsl/gsl_blas.h>
// #include <gsl/gsl_multilarge_nlinear.h>
// #include <gsl_gsl_spblas.h>
// #include <gsl/gsl_spmatrix.h>

#include "opencv2/highgui/highgui.hpp"

#include "settings.hpp"

// Global variables

// 0.0057 mm per pixel is the average!!!!

bool bAutofocusLogFlag = 0; // Flag that is 1 for when the autofocus log is
                            // being written to, 0 when it is not

std::atomic<bool> bNewImage = 0; // Flag that is 1 for when the buffer image is
                                 // new, 0 when buffer image is old

const long img_size = 640 * 480; // Replace with actual image size
bool bSaveImages = 0; // Saves images from the tilted camera to output folder. WARNING: will
                      // produce enormous number of images and slow down the system!
bool bSaveSharpnessCurves = 0; // Saves text files with the sharpness curve data, similar to above
bool bRunContinuous = 0;          // Runs autofocus method all the time (not just FindFocus/HoldFocus)


bool bBlinking = 0;

unsigned char *img_buf =
    (unsigned char *)malloc(img_size); // Accessed by thread1 and thread2
unsigned char *img_get_buf =
    (unsigned char *)malloc(img_size); // Used by by image pulling thread
unsigned char *img_calc_buf =
    (unsigned char *)malloc(img_size); // Used by image analysis thread
std::mutex mtx;

using namespace std;
using namespace cv;

int imgcount;
bool bHoldFocus;
bool bFindFocus = 0;
bool bResetLens = 0;
bool bNewMoveRel = 0;
int desiredLocBestFocus;

std::atomic<double> mmToMove = 0.0;
int increment = 0; // For saving sharpness curves (text files)
int increment2 = 0; // For saving images (png files)

// Initialise variable to store previous valid B value (see computeBestFocusEigenLM)
double lastValidB = 0.0;

std::vector<double> lastSharpnessCurve;
std::vector<double> lastXIndices;
// std::vector<double> lastFittedCurve;

static int totalFramesCaptured = 0;
static int framesProcessed = 0;

std::string csvFilename;
std::ofstream csvFile;

// Add P gain as a member variable with default value
// double Kp = 0.0022; // HAS BEEN 0.005 FOR A LONG TIME

std::atomic<double> currentMeasuredFocus{
    0.0}; // Current actual measured focus position

autofocus::autofocus()
    : lens1(), tiltedcam1(), stop_thread(false), settings("") {

  debugLogFile.open("debug_log.csv", std::ios::out | std::ios::app);
  if (debugLogFile.is_open()) {
    debugLogFile << "Timestamp,Event,Details\n";
  }
  try {
    // Load settings during construction
    settings.load();
    Kp = settings.getKp();
    Kd = settings.getKd();

  } catch (const std::exception &e) {
    std::cerr << "[lens::lens] Error loading settings: " << e.what()
              << std::endl;
    throw; // Rethrow exception to indicate critical failure
  }
}

bool autofocus::initialize() {
  bool bLensInit = lens1.initialize();
  bool bTiltedCamInit = tiltedcam1.initialize();
  std::cout << "bLensInit: " << bLensInit
            << " bTiltedCamInit: " << bTiltedCamInit << "\n";

  // Initialize benchmark CSV files with headers
  std::ofstream cpuBenchmarkFile("../output/focus_benchmark.csv");
  if (cpuBenchmarkFile.is_open()) {
    cpuBenchmarkFile
        << "timestamp,total_time_us,clahe_time_us,blur_time_us,tenengrad_time_"
           "us,column_time_us,sliding_time_us,fitting_time_us"
        << std::endl;
    cpuBenchmarkFile.close();
  }


  std::ofstream GSLBenchmarkFile("../output/focus_benchmark_GSL.csv");
  if (GSLBenchmarkFile.is_open()) {
    GSLBenchmarkFile
        // << "timestamp,total_time_us,resize_time_us,"
         << "timestamp,total_time_us, gaussian_time_us,"
           "roberts_time_us,column_time_us,offset_time_us,fit_time_us"
        << std::endl;
    GSLBenchmarkFile.close();
  }

  std::ofstream EigenLMBenchmarkFile("../output/focus_benchmark_EigenLM.csv");
  if (EigenLMBenchmarkFile.is_open()) {
    EigenLMBenchmarkFile
        << "timestamp,total_time_us, gaussian_time_us,"
           "roberts_time_us,column_time_us,offset_time_us,moving_average_time_us, fit_time_us, no. of iterations"
        << std::endl;
    EigenLMBenchmarkFile.close();
  }


  // Pre-allocate matrices for computeBestFocus to avoid runtime allocation
  int imWidth = tiltedcam1.getImageWidth();
  int imHeight = tiltedcam1.getImageHeight();

  
  blurred_preallocated = cv::Mat::zeros(imHeight, imWidth, CV_8UC1);
  img_x_preallocated = cv::Mat::zeros(imHeight, imWidth, CV_16S);
  img_y_preallocated = cv::Mat::zeros(imHeight, imWidth, CV_16S);
  img_x_squared_preallocated = cv::Mat::zeros(imHeight, imWidth, CV_32F);
  img_y_squared_preallocated = cv::Mat::zeros(imHeight, imWidth, CV_32F);
  sum_xy_preallocated = cv::Mat::zeros(imHeight, imWidth, CV_32F);
  imageofinterest_preallocated = cv::Mat::zeros(imHeight, 16, CV_8UC1);

  // Pre-allocate Roberts Cross kernels
  roberts_kernelx = (cv::Mat_<double>(2, 2) << 1, 0, 0, -1);
  roberts_kernely = (cv::Mat_<double>(2, 2) << 0, 1, -1, 0);


  if (bLensInit && bTiltedCamInit) {
    tAutofocus = std::thread(&autofocus::run, this);

    // Set high priority for autofocus thread
    pthread_t handle = tAutofocus.native_handle();
    struct sched_param params;
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (pthread_setschedparam(handle, SCHED_FIFO, &params) != 0) {
      // Fallback to nice priority if RT scheduling fails
      pthread_setschedprio(handle, -10); // Higher priority
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset); // Pin to CPU core 2 (adjust based on your system)
    CPU_SET(3, &cpuset); // And core 3
    pthread_setaffinity_np(tAutofocus.native_handle(), sizeof(cpu_set_t),
                           &cpuset);

    if (bAutofocusLogFlag) {
      logger->info("[autofocus::autofocus] autofocus::run thread started");
    }

    // Initialize CSV logging with better error handling
    std::string outputDir = "../output";
    try {
      if (!std::filesystem::create_directories(outputDir) &&
          !std::filesystem::exists(outputDir)) {
        logger->error(
            "[autofocus::initialize] Failed to create output directory: " +
            outputDir);
        return false;
      }

      csvFilename = outputDir + "/autofocus_data.csv";
      csvFile.open(csvFilename, std::ios::out | std::ios::trunc);

      if (csvFile.is_open() && csvFile.good()) {
        // Always write CSV header since we're overwriting the file
        csvFile << "time, timestamp_ms,imgcountfile,desiredLocBestFocus,locBestFocus,"
                   "pSignal,filteredLocBestFocus,dSignal,totalPdSignal,Kp\n";

        csvFile.flush();

        if (csvFile.fail()) {
          logger->error("[autofocus::initialize] Failed to write CSV header: " +
                        csvFilename);
          csvFile.close();
          return false;
        }

        logger->info("[autofocus::initialize] CSV logging started: " +
                     csvFilename);
      } else {
        logger->error("[autofocus::initialize] Failed to open CSV file: " +
                      csvFilename);
        return false;
      }
    } catch (const std::filesystem::filesystem_error &ex) {
      logger->error("[autofocus::initialize] Filesystem error: " +
                    std::string(ex.what()));
      return false;
    } catch (const std::exception &ex) {
      logger->error("[autofocus::initialize] Error setting up CSV logging: " +
                    std::string(ex.what()));
      return false;
    }

    return true;
  } else {
    return false;
  }
}

autofocus::~autofocus() {
  // Stops the autofocus thread
  stop_thread.store(true);

  // Make sure the tiltedcam capture thread is stopped
  tiltedcam1.stopCaptureThread();

  if (tAutofocus.joinable()) {
    tAutofocus.join();
  }

  // Free the globally allocated buffers
  if (img_buf != nullptr) {
    free(img_buf);
    img_buf = nullptr;
  }

  if (img_get_buf != nullptr) {
    free(img_get_buf);
    img_get_buf = nullptr;
  }

  if (img_calc_buf != nullptr) {
    free(img_calc_buf);
    img_calc_buf = nullptr;
  }

  if (bAutofocusLogFlag) {
    logger->info("[autofocus::~autofocus] destructor completed");
  }

  // Close CSV file with proper synchronization
  {
    std::lock_guard<std::mutex> lock(csvMutex);
    if (csvFile.is_open()) {
      csvFile.flush(); // Ensure all data is written before closing
      csvFile.close();
      logger->info("[autofocus::~autofocus] CSV file closed: " + csvFilename);
    }
  }
}

void autofocus::run() {
  auto t1 = std::chrono::steady_clock::now();
  usleep(10000); // 10ms
  auto t2 = std::chrono::steady_clock::now();
  auto s_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

  // For detecting oscillations
  std::deque<int>
      locBestFocusHistory; // use deque for easy push/pop at front and back

  // Start the camera capture thread
  logger->info("[autofocus::run] Starting camera capture thread");
  tiltedcam1.startCaptureThread();

  int moved = 1;
  int previous = desiredLocBestFocus; // TODO: should be in a mutex
  int tol = 6;          // Tolerance zone where no movement is made
  int blink = 0;        // Becomes 1 when a blink is detected
  int blinkframes = 15; // number of frames to ignore when blink is detected
  imgcount =
      0; // Keeps track of the number of images recieved and analyzed from the
         // camera, but resets when switching between FindFocus and HoldFocus
  int imgcountfile =
      0; // Keeps track of TOTAL number of images, for filenaming and debugging

  bHoldFocus = 0;

  int imWidth = tiltedcam1.getImageWidth();
  int imHeight = tiltedcam1.getImageHeight();

  //// PID CONTROLLER
  double dt = 1.0 / 60.0; // time per frame on the TILTED CAMERA! Assumes 60Hz.
  double max =
      3; // maximum relative move the lens can be ordered to make. Set to +-3mm
  double min = -3;
  // double Kp = 0.0012;   this has to be set as a member variable now
  double Ki = 0.0;
  // double Kd = 0.00005;

  //  double Kd = 0.00005; // Changed from 0.0 to add a small derivative term

  // PD variables for manual calculation and logging
  double filteredPreviousError = 0.0;
  double filteredLocBestFocus = 0.0; // Will be initialized on first frame
  bool isFirstFrame = true;
  const double derivativeFilterAlpha = 0.1; // Low-pass filter for measurement
  
  // Calculate movement using PID
  // PID pid(dt, max, min, Kp, Kd, Ki);

  if (bAutofocusLogFlag) {
    logger->info("[autofocus::run] while thread loop about to start");
  }

  while (!stop_thread.load()) {

    // Sophia - debugging
    if (bRunContinuous) {
      // Check if we have a new frame
      if (tiltedcam1.getLatestFrame(img_calc_buf, img_size)) {
        framesProcessed++;
        bNewImage = true;
        imgcount++;
        imgcountfile++;

        // Convert to OpenCV Mat - use reduced resolution for all processing
        cv::Mat image(imHeight, imWidth, CV_8UC1, img_calc_buf);


        // double locBestFocusDouble = computeBestFocusGSL(
        //     image, imHeight, imWidth); //  returns double

        double locBestFocusDouble = computeBestFocusEigenLM(
            image, imHeight, imWidth); //  returns double

        // std::cout << "locBestFocus (GSL): " << locBestFocusDouble << std::endl;
        // std::cout << "locBestFocus (EigenLM): " << locBestFocusEigenLM << std::endl;

        // Store the current measured focus position globally
        currentMeasuredFocus.store(locBestFocusDouble);
        
      };

    }





    // Only perform autofocus when either HoldFocus or FindFocus is active
    if (bHoldFocus || bFindFocus) {
      // Check if we have a new frame
      if (tiltedcam1.getLatestFrame(img_calc_buf, img_size)) {
        framesProcessed++;
        bNewImage = true;
        imgcount++;
        imgcountfile++;
        // Convert to OpenCV Mat - use reduced resolution for all processing
        cv::Mat image(imHeight, imWidth, CV_8UC1, img_calc_buf);

        // double locBestFocusDouble = computeBestFocusReduced(
        //     image, imHeight, imWidth); //  returns double

        double locBestFocusDouble = computeBestFocusEigenLM(
            image, imHeight, imWidth); //  returns double

        // double locBestFocusDouble = computeBestFocusGSL(
        //     image, imHeight, imWidth); //  returns double


        // Store the current measured focus position globally
        currentMeasuredFocus.store(locBestFocusDouble);

        static auto lastTime = std::chrono::high_resolution_clock::now();
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(
                            currentTime - lastTime)
                            .count();

        // Calculate FPS
        double fps = timeDiff > 0 ? 1000.0 / timeDiff : 0.0;

        // // Output to console for every frame
        // std::cout << "locBestFocus: " << locBestFocusDouble
        //           << ", desiredLocBestFocus: " << desiredLocBestFocus
        //           << ", FPS: " << std::fixed << std::setprecision(1) << fps
        //           << std::endl;

        lastTime = currentTime;

        // If imgcount==1, then the user has just turned on FindFocus or
        // HoldFocus
        if (imgcount == 1) {
          if (bHoldFocus) {
            desiredLocBestFocus =
                static_cast<int>(std::round(locBestFocusDouble));
            previous = desiredLocBestFocus;
          } else if (bFindFocus) {
            desiredLocBestFocus = 300;
            previous = static_cast<int>(std::round(locBestFocusDouble));
            if (bAutofocusLogFlag) {
              logger->info("[autofocus::run] Set desiredLocBestFocus back to "
                           "320 in autofocus.cc");
            }
          }
        }

        // (Shan testing) Use PID directly to calculate movement
        // double moveAmount = pid.calculate(desiredLocBestFocus, locBestFocusDouble) * -1.0;
        // mmToMove = moveAmount;
        // bNewMoveRel = 1;
        // moved = (std::abs(moveAmount) > 1e-12);

        // Filter the measurement for derivative term only
        if (isFirstFrame) {
          filteredLocBestFocus = static_cast<int>(
              std::round(locBestFocusDouble)); // Initialize on first frame
          isFirstFrame = false;
        } else {
          filteredLocBestFocus = static_cast<int>(
              std::round((1.0 - derivativeFilterAlpha) * filteredLocBestFocus +
                         derivativeFilterAlpha * locBestFocusDouble));
        }

        // Calculate PD components - P uses raw measurement with double
        // precision
        double currentError =
            desiredLocBestFocus -
            locBestFocusDouble; // Use double for higher precision

        // Scale P gain based on error magnitude
        double errorMagnitude = abs(currentError);
        double pScaleFactor = 1.0;
        if (errorMagnitude <= 3.0) {
          pScaleFactor = 0.13;
        } else if (errorMagnitude >= 100.0) {
          pScaleFactor = 1.0;
        } else {
          // Linear interpolation between 3 pixels (0.1x) and 100 pixels (1.0x)
          pScaleFactor =
              0.13 + (errorMagnitude - 3.0) * (1.0 - 0.13) / (100.0 - 3.0);
        }

        // Apply directional multiplier for downward moves (positive errors)
        double effectiveKp = Kp;
        double effectiveKd = Kd;
        if (currentError > 0) {
          // Moving toward more positive values (340→320, 320→300) - these were
          // slower
          effectiveKp = 2.24e-3;
          effectiveKd = 1.06e-4;
        }

        double pSignal = effectiveKp * currentError * pScaleFactor;

        // Calculate derivative using filtered measurement
        double filteredCurrentError =
            desiredLocBestFocus - filteredLocBestFocus;

        // Calculate derivative using filtered error
        double rawDerivative =
            (filteredCurrentError - filteredPreviousError) / dt;
        // double dSignal = Kd * rawDerivative;
        double dSignal = effectiveKd * rawDerivative;

        // Total PD signal
        double totalPdSignal = pSignal + dSignal;

        // Apply limits
        if (totalPdSignal > max)
          totalPdSignal = max;
        else if (totalPdSignal < min)
          totalPdSignal = min;

        // Store for next iteration
        // previousError = currentError; not actually used
        filteredPreviousError = filteredCurrentError;

        // Log data to CSV with improved error handling and thread safety
        {
          std::lock_guard<std::mutex> lock(csvMutex);
          if (csvFile.is_open() && csvFile.good()) {
            auto currentTime =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();
            // Timestamp in human readable form
            auto now = std::chrono::system_clock::now();
            auto now_time_t = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

            csvFile << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S")
                    << "." << std::setfill('0') << std::setw(3) << ms.count() << ",";


            // Use higher precision for double values
            csvFile << std::fixed << std::setprecision(6);
            csvFile << currentTime << "," << imgcountfile << ","
                    << desiredLocBestFocus << "," << locBestFocusDouble << ","
                    << pSignal << "," << filteredLocBestFocus << "," << dSignal
                    << "," << totalPdSignal << "," << Kp << "\n";

            // More frequent flushing for data safety - every 5 frames
            if (imgcountfile % 5 == 0) {
              csvFile.flush();
            }

            // Check for write errors
            if (csvFile.fail()) {
              if (bAutofocusLogFlag) {
                logger->error("[autofocus::run] CSV write error detected, "
                              "attempting to recover");
              }

              // Clear error flags and try to recover
              csvFile.clear();

              // If still failing, close and try to reopen
              if (csvFile.fail()) {
                csvFile.close();
                csvFile.open(csvFilename, std::ios::out | std::ios::app);
                if (!csvFile.is_open()) {
                  if (bAutofocusLogFlag) {
                    logger->error(
                        "[autofocus::run] Failed to recover CSV file: " +
                        csvFilename);
                  }
                }
              }
            }
          } else if (csvFile.is_open()) {
            // File is open but in bad state, try to recover
            if (bAutofocusLogFlag) {
              logger->warn("[autofocus::run] CSV file in bad state, attempting "
                           "recovery");
            }
            csvFile.clear();
          }
        }

        //// BLINK DETECTION. TODO: try removing 'moved' variable
        if (moved == 0 && blink == 0 &&
            abs(static_cast<int>(std::round(locBestFocusDouble)) - previous) >
                (50) &&
            bBlinking) { // if the location of best focus changes by more than
                         // 50 pixels with no move, it is a blink
          // Blink starts
          if (bAutofocusLogFlag)
            logger->info("[autofocus::run] Frame ignored; blink detected");
          std::cout << "Blink detected" << std::endl;
          blink = 1;
        } else if (blink == 1) {
          if (bAutofocusLogFlag)
            logger->info("[autofocus::run] Frame ignored; blink detected");
          blinkframes--;
          if (blinkframes == 0) {
            blinkframes = 15; // resets blinkframes
            blink = 0;
          }
        } else {
          // Use double precision measurement for tolerance checking
          if (abs(locBestFocusDouble - desiredLocBestFocus) <= tol) {
            // std::cout << ", in TOL band\n";
            moved = 0;
          } else {
            // Use the double precision PD signal - NO CASTING!
            mmToMove = totalPdSignal * -1.0;
            bNewMoveRel = 1;
            moved = 1;
            // std::cout << "mmToMove: " << mmToMove << "\n";

            // //// OSCILLATION DETECTION
            // //Adding to locBestFocusHistory when outside TOL band
            // locBestFocusHistory.push_back(locBestFocus);
            // // If we have more than 20 values, remove the oldest
            // if (locBestFocusHistory.size() > 20) {
            //   locBestFocusHistory.pop_front();
            // }
            // // Check if history contains values both above and below
            // desiredLocBestFocus if(locBestFocusHistory.size() == 20) {
            //   bool hasAbove = std::any_of(locBestFocusHistory.begin(),
            //   locBestFocusHistory.end(), [](int x) { return x >
            //   desiredLocBestFocus; }); bool hasBelow =
            //   std::any_of(locBestFocusHistory.begin(),
            //   locBestFocusHistory.end(), [](int x) { return x <
            //   desiredLocBestFocus; }); if (hasAbove && hasBelow) {
            //     // Find the value closest to desiredLocBestFocus
            //     auto closestIt =
            //     std::min_element(locBestFocusHistory.begin(),
            //     locBestFocusHistory.end(), [](int a, int b) {
            //         return std::abs(a - desiredLocBestFocus) < std::abs(b -
            //         desiredLocBestFocus);
            //     });
            //     desiredLocBestFocus = *closestIt;
            //     std::cout << "DETECTED OSCILLATION!! Adjusting
            //     desiredLocBestFocus to " << desiredLocBestFocus << "\n";
            //     locBestFocusHistory.clear(); // Clear history after adjusting
            //     desiredLocBestFocus
            //   }
            // }
          }
        }
        previous = static_cast<int>(std::round(locBestFocusDouble));

        // Periodically report frame drop rate
        if (framesProcessed % 300 ==
            0) { // Every 5 seconds at 60fps
                 // std::cout << "Frames processed: " << framesProcessed
                 //<< ", Estimated drops: " << (framesProcessed * 17/16.67 -
                 // framesProcessed) << std::endl;
        }
      }
    } else {
      // Reset counter when autofocus is not active
      imgcount = 0;

      // Sleep a bit to reduce CPU usage when not actively focusing
      usleep(50000); // 50ms
    }
  }
  tiltedcam1.stopCaptureThread();
}






double autofocus::computeBestFocusGaussian(cv::Mat image, int imgHeight,
                                           int imgWidth) {
              
  auto startTime = std::chrono::high_resolution_clock::now();

  // Resize to 1/2 x 1/2
  auto resizeStart = std::chrono::high_resolution_clock::now();
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), 0.5, 0.5);
  auto resizeEnd = std::chrono::high_resolution_clock::now();

  // Roberts Cross gradients -> sharpness image (float)
  auto robertsStart = std::chrono::high_resolution_clock::now();
  cv::Mat img_x, img_y;
  // cv::filter2D(resized, img_x, CV_32F, roberts_kernelx);
  // cv::filter2D(resized, img_y, CV_32F, roberts_kernely);
  cv::filter2D(resized, img_x, CV_16S, roberts_kernelx);
  cv::filter2D(resized, img_y, CV_16S, roberts_kernely);
  cv::Mat img_x_squared, img_y_squared, sum_xy;
  cv::multiply(img_x, img_x, img_x_squared);
  cv::multiply(img_y, img_y, img_y_squared);
  cv::add(img_x_squared, img_y_squared, sum_xy);
  cv::Mat sharpness_float;
  sum_xy.convertTo(sharpness_float, CV_32F);
  auto robertsEnd = std::chrono::high_resolution_clock::now();

  // Print largest value in sharpness_float for debugging
  // std::cout << "Max sharpness value: ";
  // double minVal, maxVal;
  // cv::minMaxLoc(sharpness_float, &minVal, &maxVal);
  // std::cout << maxVal << std::endl;

 


  // Column means
  auto columnStart = std::chrono::high_resolution_clock::now();
  cv::Mat columnMeansMatrix;
  cv::reduce(sharpness_float, columnMeansMatrix, 0, cv::REDUCE_AVG, CV_64F);
  std::vector<double> columnMeans;
  columnMeansMatrix.copyTo(columnMeans);
  auto columnEnd = std::chrono::high_resolution_clock::now();


  // std::cout << "Max column mean value : ";
  // double minColVal, maxColVal;
  // cv::Mat columnMeansMat(columnMeans);
  // cv::minMaxLoc(columnMeansMat, &minColVal, &maxColVal);
  // std::cout << maxColVal << std::endl;

  // Compute offset
  auto offsetStart = std::chrono::high_resolution_clock::now();
  const int offsetWindow = 50;
  double offset_left = 0.0, offset_right = 0.0;
  if (columnMeans.size() >= offsetWindow) {
      offset_left = std::accumulate(columnMeans.begin(), columnMeans.begin() + offsetWindow, 0.0) / offsetWindow;
      offset_right = std::accumulate(columnMeans.end() - offsetWindow, columnMeans.end(), 0.0) / offsetWindow;
  } else if (!columnMeans.empty()) {
      offset_left = std::accumulate(columnMeans.begin(), columnMeans.end(), 0.0) / columnMeans.size();
      offset_right = std::accumulate(columnMeans.begin(), columnMeans.end(), 0.0) / columnMeans.size();
  } else {
      offset_left = 0.0; // or handle empty case as needed
      offset_right = 0.0;
  }

  // Choose smallest offset and subtract, omitting negative values
  double offset = std::min(offset_left, offset_right);
  // std::vector<double> sharpnesscurve;
  // sharpnesscurve.reserve(columnMeans.size());
  // for (const double& v : columnMeans) {
  //   double val = v - offset;
  //   if (val >= 0.0) {
  //     sharpnesscurve.push_back(val);
  //   }
  // }

  std::vector<std::pair<double, double>> xy_pairs;
  xy_pairs.reserve(columnMeans.size());
  for (size_t i = 0; i < columnMeans.size(); ++i) {
    double x = static_cast<double>(i);
    double y = columnMeans[i] - offset;
    if (y >= 0.0) {
      xy_pairs.emplace_back(x, y);
    }
  }

  auto offsetEnd = std::chrono::high_resolution_clock::now();

  

  // Rescale x to range [-0.5, 0.5], (and maybe y to [0, 1]?)
  auto rescaleStart = std::chrono::high_resolution_clock::now();
  // std::vector<double> y_values;
  // y_values.reserve(xy_pairs.size());
  // for (const auto& p : xy_pairs) {
  //     y_values.push_back(p.second);
  // }
  double max_sharpness = xy_pairs.empty() ? 1.0 :
      std::max_element(xy_pairs.begin(), xy_pairs.end(),
          [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
              return a.second < b.second;
          })->second;

  // std::cout << "Max sharpness after offset: " << max_sharpness << std::endl;
  // std::cout << "Offset value subtracted: " << offset << std::endl;

  int N = static_cast<int>(xy_pairs.size());

  std::vector<double> x_values(N);

  // Rescale y to [0,1]
  for (int i = 0; i < N; ++i) {
    xy_pairs[i].second /= max_sharpness; // y in [0, 1]
  }


  // Save values for plotting
  if (bSaveImages) {
    lastSharpnessCurve.clear();
    lastXIndices.clear();
    for (const auto& p : xy_pairs) {
      lastSharpnessCurve.push_back(p.second);
      lastXIndices.push_back(p.first);
    }
  }

  // Rescale x in [-0.5, 0.5]
  for (int i = 0; i < N; ++i) {
    xy_pairs[i].first = static_cast<double>(i) / (N - 1) - 0.5; // x in [-0.5, 0.5]
  }

  auto rescaleEnd = std::chrono::high_resolution_clock::now();


  // Print min and max x value
  // std::cout << "X range after rescaling: ";
  // if (!xy_pairs.empty()) {
  //   double min_x = xy_pairs.front().first;
  //   double max_x = xy_pairs.back().first;
  //   std::cout << min_x << " to " << max_x << std::endl;
  // }

  // Max sharpness
  // std::cout << "Max sharpness after offset and rescale: ";
  // double max_sharpness_final = xy_pairs.empty() ? 0.0 :
  //     std::max_element(xy_pairs.begin(), xy_pairs.end(),
  //         [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
  //             return a.second < b.second;
  //         })->second;
  // std::cout << max_sharpness_final << std::endl;

  // Iterative Gaussian fit (weighted least squares)
  auto gaussianFitStart = std::chrono::high_resolution_clock::now();

  int maxIterations = 10;

  // List to store mu values
  std::vector<double> muValues;
  double mu_prev = std::numeric_limits<double>::quiet_NaN();


  double coeff_a = 0.0, coeff_b = 0.0, coeff_c = 0.0;

  for (int iter = 0; iter < maxIterations; ++iter) {

    double a11 = 0.0, a12 = 0.0, a13 = 0.0;
    double a21 = 0.0, a22 = 0.0, a23 = 0.0;
    double a31 = 0.0, a32 = 0.0, a33 = 0.0;
    double b_11 = 0.0, b_12 = 0.0, b_13 = 0.0;

    

    for (int i = 0; i < N; ++i) {

      double ln_y = std::log(xy_pairs[i].second);
      // Not normalized
      // double x = static_cast<double>(i);

      // Normalized to [-0.5, 0.5]
      double x_val = xy_pairs[i].first;

      double y;

      if (iter == 0) {
        y = xy_pairs[i].second;
      } else {
        y = std::exp(coeff_a + coeff_b * x_val + coeff_c * x_val * x_val);
      }

      
      double y2 = y * y;
      double x2 = x_val * x_val;
      double x3 = x2 * x_val;
      double x4 = x2 * x2;

      a11 += y2;
      a12 += x_val * y2;
      a13 += x2 * y2;
      a23 += x3 * y2;
      a33 += x4 * y2;

      b_11 += y2 * ln_y;
      b_12 += x_val * y2 * ln_y;
      b_13 += x2 * y2 * ln_y;
    }
  

    a21 = a12; // Symmetry
    a31 = a13; // Symmetry
    a22 = a13; // Symmetry
    a32 = a23; // Symmetry


    // Define A and b
    cv::Mat A = (cv::Mat_<double>(3, 3) << a11, a12, a13, 
                  a21, a22, a23,
                  a31, a32, a33);

    cv::Mat b = (cv::Mat_<double>(3, 1) << b_11, b_12, b_13);

    // // Calculate inverse
    // cv::Mat A_inv;
    // cv::invert(A, A_inv, cv::DECOMP_LU);
    // cv::Mat x = A_inv * b;
    // double coeff_a = x.at<double>(0, 0);
    // double coeff_b = x.at<double>(1, 0);
    // double coeff_c = x.at<double>(2, 0);
    // double mu = -coeff_b / (2 * coeff_c);
    // muValues.push_back(mu);

    // // // or hard-code inverse
    // double det = cv::determinant(A);

    // double det =
    //     a11 * (a22 * a33 - a23 * a32)
    //   - a12 * (a21 * a33 - a23 * a31)
    //   + a13 * (a21 * a32 - a22 * a31);

    // cv::Mat A_inv = (1.0 / det) * (cv::Mat_<double>(3, 3) <<
    //     (a22 * a33 - a23 * a32),
    //     (a13 * a32 - a12 * a33),
    //     (a12 * a23 - a13 * a22),

    //     (a23 * a31 - a21 * a33),
    //     (a11 * a33 - a13 * a31),
    //     (a13 * a21 - a11 * a23),

    //     (a21 * a32 - a22 * a31),
    //     (a12 * a31 - a11 * a32),
    //     (a11 * a22 - a12 * a21)
    // );
    // cv::Mat x;
    // x = A_inv * b;
    // double coeff_a = x.at<double>(0, 0);
    // double coeff_b = x.at<double>(1, 0);
    // double coeff_c = x.at<double>(2, 0);
    // double mu = -coeff_b / (2 * coeff_c);
    // muValues.push_back(mu);

    // or Solve for parameters
    cv::Mat x;
    cv::solve(A, b, x, cv::DECOMP_NORMAL);

    double coeff_a = x.at<double>(0, 0);
    double coeff_b = x.at<double>(1, 0);
    double coeff_c = x.at<double>(2, 0);
    double mu = -coeff_b / (2 * coeff_c);

    // Scale back to original index to save
    double mu_index = (mu + 0.5) * (N - 1);
    muValues.push_back(mu_index);

    // Early stopping: check relative change in mu
    // if (iter > 0 && std::abs(mu - mu_prev) / std::abs(mu_prev) < 0.001) {
    //   //std::cout << "Early stopping at iteration " << iter << std::endl;
    //   break; // Change less than 0.1%, stop early
    // }
    mu_prev = mu;


  }
  auto gaussianFitEnd = std::chrono::high_resolution_clock::now();


  //Print min and max x value after rescaling
  // std::cout << "X range after rescaling back: ";
  // if (!xy_pairs.empty()) {
  //   double min_x = xy_pairs.front().first; 
  //   double max_x = xy_pairs.back().first;
  //   std::cout << min_x << " to " << max_x << std::endl;
  // }


  auto estEnd = std::chrono::high_resolution_clock::now();


  // --- Timing log unchanged footprint
  // ---
  auto totalTime =
      std::chrono::duration_cast<std::chrono::microseconds>(estEnd - startTime);
  auto resizeTime = std::chrono::duration_cast<std::chrono::microseconds>(
      resizeEnd - resizeStart);
  auto robertsTime = std::chrono::duration_cast<std::chrono::microseconds>(
      robertsEnd - robertsStart);
  auto columnTime = std::chrono::duration_cast<std::chrono::microseconds>(
      columnEnd - columnStart);
  auto offsetTime = std::chrono::duration_cast<std::chrono::microseconds>(
      offsetEnd - offsetStart);
  auto rescaleTime = std::chrono::duration_cast<std::chrono::microseconds>(
      rescaleEnd - rescaleStart);
  auto gaussianFitTime = std::chrono::duration_cast<std::chrono::microseconds>(
      gaussianFitEnd - gaussianFitStart);
  
  try {
    std::ofstream iterativeBenchmarkFile("../output/focus_benchmark_iterative.csv",
                                       std::ios::app);
    if (iterativeBenchmarkFile.is_open() && iterativeBenchmarkFile.good()) {
      auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
      // keep CSV header compatibility: write estimator time in the
      // "com_time_us" column
      iterativeBenchmarkFile << timestamp << "," << totalTime.count() << ","
                          << resizeTime.count() << "," << robertsTime.count()
                          << "," << columnTime.count() << ","
                          << offsetTime.count() << "," << rescaleTime.count() << ","
                          << gaussianFitTime.count() << ",";

      // Write all mu values for each iteration, separated by semicolon
      for (size_t i = 0; i < muValues.size(); ++i) {
          iterativeBenchmarkFile << muValues[i];
          if (i < muValues.size() - 1) iterativeBenchmarkFile << ";";
      }
      iterativeBenchmarkFile << std::endl;
    }
  } catch (...) { /* ignore */
  }


  //std::cout << "Mu values over iterations: ";
  // for (const auto& mu : muValues) {
  //     std::cout << mu << " ";
  // }
  // std::cout << std::endl;


    if (bSaveSharpnessCurves) {

    // UNCOMMENT TO SAVE TXT FILES
    std::string FileName = "TESTING_GAUSSIAN_" + std::to_string(increment2);
    std::string TextFile1 = "../output/SharpnessCurves_steps_Gaussian/" + FileName + "_resized.txt";
    std::string TextFile2 = "../output/SharpnessCurves_steps_Gaussian/" + FileName + "_robertscross.txt";
    // std::string TextFile3 = "../output/SharpnessCurves_steps_Gaussian/" + FileName + "_reduced_robertscross.txt";
    // std::string TextFile4 = "../output/SharpnessCurves_steps_Gaussian/" + FileName + "_column_means.txt";
    // std::string TextFile5 = "../output/SharpnessCurves_steps_Gaussian/" + FileName + "_reduced_column_means.txt";
    // std::string TextFile6 = "../output/SharpnessCurves_steps_Gaussian/" + FileName + "_sharpnesscurve.txt";
    std::ofstream outputFile1(TextFile1);
    std::ofstream outputFile2(TextFile2);
    // std::ofstream outputFile3(TextFile3);
    // std::ofstream outputFile4(TextFile4);
    // std::ofstream outputFile5(TextFile5);
    // std::ofstream outputFile6(TextFile6);


    std::ostream_iterator<double> output_iterator(outputFile1, ", ");
    for (int i = 0; i < resized.rows; ++i) {
      const uchar* row_ptr = resized.ptr<uchar>(i);
      for (int j = 0; j < resized.cols; ++j) {
        outputFile1 << static_cast<double>(row_ptr[j]) << ", ";
      }
    }
    outputFile1 << "\n";

    std::ostream_iterator<double> output_iterator2(outputFile2, ", ");
    for (int i = 0; i < sharpness_float.rows; ++i) {
      const float* row_ptr = sharpness_float.ptr<float>(i);
      for (int j = 0; j < sharpness_float.cols; ++j) {
        outputFile2 << static_cast<double>(row_ptr[j]) << ", ";
      }
    }
    outputFile2 << "\n";

    outputFile1.close();
    outputFile2.close();

    // SaveSharpnessTxt(resized, sharpness_float, columnMeans, lastSharpnessCurve, increment2);
    increment2++;
  }


  return muValues.back();


} 









cv::Scalar autofocus::robertscross(cv::Mat imagedata) {
  // // Creating Roberts Cross matrices
  // cv::Mat matrixx = (cv::Mat_<double>(2, 2) << 1, 0, 0, -1);
  // cv::Mat matrixy = (cv::Mat_<double>(2, 2) << 0, 1, -1, 0);

  // //convolving imagedata with the matrices
  // cv::Mat img_x, img_y;
  // cv::filter2D(imagedata, img_x, -1, matrixx);
  // cv::filter2D(imagedata, img_y, -1, matrixy);

  // //squaring and summing the resultant matrices, which gives us the sharpness
  // (or, gradient) for each pixel in imagedata, and taking the average score
  // over the portion of image cv::Scalar mean_sharpness = mean(img_x.mul(img_x)
  // + img_y.mul(img_y)); return mean_sharpness;

  // OPENCL-FRIENDLY VERSION
  // Creating Roberts Cross matrices
  cv::Mat temp_matrixx = (cv::Mat_<double>(2, 2) << 1, 0, 0, -1);
  cv::Mat temp_matrixy = (cv::Mat_<double>(2, 2) << 0, 1, -1, 0);

  // cv::UMat matrixx = temp_matrixx.getUMat(cv::ACCESS_READ);
  // cv::UMat matrixy = temp_matrixy.getUMat(cv::ACCESS_READ);

  // convolving imagedata with the matrices. WARNING, REPLACE TEMP_ WITH ACTUAL
  cv::Mat img_x, img_y;
  cv::filter2D(imagedata, img_x, -1, temp_matrixx);
  cv::filter2D(imagedata, img_y, -1, temp_matrixy);

  // squaring and summing the resultant matrices, which gives us the sharpness
  // (or, gradient) for each pixel in imagedata, and taking the average score
  // over the portion of image
  cv::Mat img_x_squared, img_y_squared;
  cv::multiply(img_x, img_x, img_x_squared);
  cv::multiply(img_y, img_y, img_y_squared);
  cv::Mat sum_xy;
  cv::add(img_x_squared, img_y_squared, sum_xy);
  cv::Scalar mean_sharpness = cv::mean(sum_xy);
  return mean_sharpness;
}

cv::Scalar autofocus::tenengrad(cv::Mat img) {
  // Two sobel operations //
  cv::Mat Gx, Gy;
  cv::Sobel(img, Gx, CV_64F, 1, 0, 3);
  cv::Sobel(img, Gy, CV_64F, 0, 1, 3);
  cv::Scalar mean_grad = mean(Gx.mul(Gx) + Gy.mul(Gy));

  // One Sobel operation //
  // Mat Gxy;
  // Sobel(img, Gxy, CV_64F, 1, 1, 3);
  // Scalar mean_grad = mean(abs(Gxy));
  return mean_grad;
}
cv::Scalar autofocus::vollath(cv::Mat img) {
  // Vollath's F4
  int sum1 = 0;
  int sum2 = 0;
  int h = img.rows; // 480
  int w = img.cols; // 16
  for (int i = 0; i < h - 1; i++) {
    for (int j = 0; j < w; j++) {
      sum1 = sum1 + static_cast<int>(img.at<uchar>(i, j)) *
                        static_cast<int>(img.at<uchar>(
                            i + 1, j)); // might need to flip i and j
    }
  }
  for (int i = 0; i < h - 2; i++) {
    for (int j = 0; j < w; j++) {
      sum2 = sum2 + static_cast<int>(img.at<uchar>(i, j)) *
                        static_cast<int>(img.at<uchar>(i + 2, j));
    }
  }
  cv::Scalar mean_grad_vol = sum1 - sum2;
  return mean_grad_vol;
}
cv::Scalar autofocus::canny(cv::Mat img) {
  // Canny //
  cv::Mat edges;
  cv::Canny(img, edges, 25, 55, 3);
  cv::Scalar mean_grad_canny = mean(edges.mul(edges));
  return mean_grad_canny;
}

std::vector<double>
autofocus::fitnormalcurve(std::vector<double> sharpnesscurve, double amplitude,
                          double offset, double std_dev_factor) {
  if (sharpnesscurve.empty())
    return sharpnesscurve;

  // Find initial guess from sharpness curve maximum
  auto maxIt = std::max_element(sharpnesscurve.begin(), sharpnesscurve.end());
  double currentMean = std::distance(sharpnesscurve.begin(), maxIt);

  // Gradient descent parameters - try smaller learning rate and more iterations
  double learningRate = 0.1; // Reduced from 0.5
  double tolerance = 1e-6;
  int maxIterations = 200; // Increased from 100

  double sigma = sharpnesscurve.size() * std_dev_factor;

  // Add bounds checking for mean
  double minMean = 0.0;
  double maxMean = sharpnesscurve.size() - 1.0;

  for (int iter = 0; iter < maxIterations; iter++) {
    double h = 0.1;

    // Calculate gradient for mean only
    double gradientMean;
    if (currentMean < h) {
      double errorCurrent = calculateErrorWithAmplitudeAndOffset(
          sharpnesscurve, currentMean, amplitude, offset, sigma);
      double errorForward = calculateErrorWithAmplitudeAndOffset(
          sharpnesscurve, currentMean + h, amplitude, offset, sigma);
      gradientMean = (errorForward - errorCurrent) / h;
    } else if (currentMean > sharpnesscurve.size() - 1 - h) {
      double errorCurrent = calculateErrorWithAmplitudeAndOffset(
          sharpnesscurve, currentMean, amplitude, offset, sigma);
      double errorBackward = calculateErrorWithAmplitudeAndOffset(
          sharpnesscurve, currentMean - h, amplitude, offset, sigma);
      gradientMean = (errorCurrent - errorBackward) / h;
    } else {
      double errorPlus = calculateErrorWithAmplitudeAndOffset(
          sharpnesscurve, currentMean + h, amplitude, offset, sigma);
      double errorMinus = calculateErrorWithAmplitudeAndOffset(
          sharpnesscurve, currentMean - h, amplitude, offset, sigma);
      gradientMean = (errorPlus - errorMinus) / (2.0 * h);
    }

    // Update mean with bounds checking
    double newMean = currentMean - learningRate * gradientMean;
    newMean = std::max(minMean, std::min(maxMean, newMean)); // Clamp to bounds

    // Check for convergence
    if (std::abs(newMean - currentMean) < tolerance) {
      break;
    }

    currentMean = newMean;
  }

  // Generate the fitted curve exactly like the old version
  std::vector<double> norm_curve;
  for (int i = 0; i < sharpnesscurve.size(); i++) {
    double normResult = normpdf(i, currentMean, sigma);
    norm_curve.push_back(normResult);
  }

  // Scale exactly like the old version: find max, then scale so max equals
  // amplitude
  double maxNormValue = *std::max_element(norm_curve.begin(), norm_curve.end());
  double factor = 1.0 / maxNormValue;

  for (int i = 0; i < sharpnesscurve.size(); i++) {
    norm_curve[i] = norm_curve[i] * factor * amplitude + offset;
  }

  return norm_curve;
}

double autofocus::calculateErrorWithAmplitudeAndOffset(
    const std::vector<double> &sharpnesscurve, double mean, double amplitude,
    double offset, double sigma) {
  double totalError = 0.0;

  // Generate normal curve for this mean
  std::vector<double> norm_curve_temp;
  for (int i = 0; i < sharpnesscurve.size(); i++) {
    double normResult = normpdf(i, mean, sigma);
    norm_curve_temp.push_back(normResult);
  }

  // Scale exactly like the old version
  double maxNormValue =
      *std::max_element(norm_curve_temp.begin(), norm_curve_temp.end());
  double factor = 1.0 / maxNormValue;

  // Calculate error using absolute difference (like old version)
  for (int i = 0; i < sharpnesscurve.size(); i++) {
    double scaledNormValue = norm_curve_temp[i] * factor * amplitude + offset;
    double error = std::abs(sharpnesscurve[i] - scaledNormValue);
    totalError += error;
  }

  return totalError;
}

double autofocus::normpdf(double x, double u, double s) {
  const double ONE_OVER_SQRT_2PI = 0.39894228040143267793994605993438;
  return (ONE_OVER_SQRT_2PI / s) * exp(-0.5 * std::pow(((x - u) / s), 2));
}

void autofocus::adjust_bestFocus(int val) {
  // TODO: should be in mutex
  desiredLocBestFocus = val;
}

void autofocus::setPGain(double gain) {
  Kp = gain;
  std::cout << "P gain set to: " << Kp << std::endl;
  if (bAutofocusLogFlag) {
    logger->info("[autofocus::setPGain] P gain set to: {}", gain);
  }
}

double autofocus::getPGain() const { return Kp; }


struct data {
  double *t;
  double *y;
  size_t n;
};

/* model function: a * exp( -1/2 * [ (t - b) / c ]^2 ) */
double
gaussian(const double a, const double b, const double c, const double t)
{
  const double z = (t - b) / c;
  return (a * exp(-0.5 * z * z));
}

int func_f (const gsl_vector * x, void *params, gsl_vector * f) {
  // GSL least squares fitting function
  struct data *d = (struct data *) params;
  double a = gsl_vector_get (x, 0);
  double b = gsl_vector_get (x, 1);
  double c = gsl_vector_get (x, 2);
  // double d_offset = gsl_vector_get (x, 3);
  size_t i;

  for (i = 0; i < d->n; i++) {
    double ti = d->t[i];
    double yi = d->y[i];
    double y = gaussian(a,b,c,ti);
    gsl_vector_set (f, i, yi-y);
  }
  return GSL_SUCCESS;
}

int
func_df (const gsl_vector * x, void *params, gsl_matrix * J)
{
  struct data *d = (struct data *) params;
  double a = gsl_vector_get(x, 0);
  double b = gsl_vector_get(x, 1);
  double c = gsl_vector_get(x, 2);
  // double d_offset = gsl_vector_get(x, 3);
  size_t i;

  for (i = 0; i < d->n; ++i)
    {
      double ti = d->t[i];
      double zi = (ti - b) / c;
      double ei = exp(-0.5 * zi * zi);

      gsl_matrix_set(J, i, 0, -ei);
      gsl_matrix_set(J, i, 1, -(a / c) * ei * zi);
      gsl_matrix_set(J, i, 2, -(a / c) * ei * zi * zi);
      // gsl_matrix_set(J, i, 3, 1.0);
    }

  return GSL_SUCCESS;
}

int
func_fvv (const gsl_vector * x, const gsl_vector * v,
          void *params, gsl_vector * fvv)
{
  struct data *d = (struct data *) params;
  double a = gsl_vector_get(x, 0);
  double b = gsl_vector_get(x, 1);
  double c = gsl_vector_get(x, 2);
  // double d_offset = gsl_vector_get(x, 3);
  double va = gsl_vector_get(v, 0);
  double vb = gsl_vector_get(v, 1);
  double vc = gsl_vector_get(v, 2);
  // double vd = gsl_vector_get(v, 3);
  size_t i;

  for (i = 0; i < d->n; ++i)
    {
      double ti = d->t[i];
      double zi = (ti - b) / c;
      double ei = exp(-0.5 * zi * zi);
      double Dab = -zi * ei / c;
      double Dac = -zi * zi * ei / c;
      double Dbb = a * ei / (c * c) * (1.0 - zi*zi);
      double Dbc = a * zi * ei / (c * c) * (2.0 - zi*zi);
      double Dcc = a * zi * zi * ei / (c * c) * (3.0 - zi*zi);
      double sum;

      sum = 2.0 * va * vb * Dab +
            2.0 * va * vc * Dac +
                  vb * vb * Dbb +
            2.0 * vb * vc * Dbc +
                  vc * vc * Dcc;

      gsl_vector_set(fvv, i, sum);
    }

  return GSL_SUCCESS;
}

void
callback(const size_t iter, void *params,
         const gsl_multifit_nlinear_workspace *w)
{
  gsl_vector *f = gsl_multifit_nlinear_residual(w);
  gsl_vector *x = gsl_multifit_nlinear_position(w);
  double avratio = gsl_multifit_nlinear_avratio(w);
  double rcond;

  (void) params; /* not used */

  /* compute reciprocal condition number of J(x) */
  gsl_multifit_nlinear_rcond(&rcond, w);

  // fprintf(stderr, "iter %2zu: a = %.4f, b = %.4f, c = %.4f, d_offset = %.4f, |a|/|v| = %.4f cond(J) = %8.4f, |f(x)| = %.4f\n",
  //         iter,
  //         gsl_vector_get(x, 0),
  //         gsl_vector_get(x, 1),
  //         gsl_vector_get(x, 2),
  //         gsl_vector_get(x, 3),
  //         avratio,
  //         1.0 / rcond,
  //         gsl_blas_dnrm2(f));
}

int
solve_system(gsl_vector *x, gsl_multifit_nlinear_fdf *fdf,
             gsl_multifit_nlinear_parameters *params)
{
  const gsl_multifit_nlinear_type *T = gsl_multifit_nlinear_trust;
  const size_t max_iter = 200;
  const double xtol = 1.0e-3;
  const double gtol = 1.0e-3;
  const double ftol = 1.0e-3;
  const size_t n = fdf->n;
  const size_t p = fdf->p;
  gsl_multifit_nlinear_workspace *work =
    gsl_multifit_nlinear_alloc(T, params, n, p);
  gsl_vector * f = gsl_multifit_nlinear_residual(work);
  gsl_vector * y = gsl_multifit_nlinear_position(work);
  int info;
  double chisq0, chisq, rcond;

  /* initialize solver */
  gsl_multifit_nlinear_init(x, fdf, work);

  /* store initial cost */
  gsl_blas_ddot(f, f, &chisq0);

  /* iterate until convergence */
  gsl_multifit_nlinear_driver(max_iter, xtol, gtol, ftol,
                              callback, NULL, &info, work);

  /* store final cost */
  gsl_blas_ddot(f, f, &chisq);

  /* store cond(J(x)) */
  gsl_multifit_nlinear_rcond(&rcond, work);

  gsl_vector_memcpy(x, y);

  /* print summary */

  // fprintf(stderr, "NITER         = %zu\n", gsl_multifit_nlinear_niter(work));
  // fprintf(stderr, "NFEV          = %zu\n", fdf->nevalf);
  // fprintf(stderr, "NJEV          = %zu\n", fdf->nevaldf);
  // fprintf(stderr, "NAEV          = %zu\n", fdf->nevalfvv);
  // fprintf(stderr, "initial cost  = %.12e\n", chisq0);
  // fprintf(stderr, "final cost    = %.12e\n", chisq);
  // fprintf(stderr, "final x       = (%.12e, %.12e, %12e, %12e)\n",
  //         gsl_vector_get(x, 0), gsl_vector_get(x, 1), gsl_vector_get(x, 2), gsl_vector_get(x, 3));
  // fprintf(stderr, "final cond(J) = %.12e\n", 1.0 / rcond);

  gsl_multifit_nlinear_free(work);

  int n_iter = gsl_multifit_nlinear_niter(work);
  return n_iter;
}

double
autofocus::computeBestFocusGSL(cv::Mat image, int imgHeight,
                                          int imgWidth) {
  auto startTime = std::chrono::high_resolution_clock::now();

  // Resize to 1/2 x 1/2
  // auto resizeStart = std::chrono::high_resolution_clock::now();
  // cv::Mat resized;
  // cv::resize(image, resized, cv::Size(), 0.5, 0.5);
  // auto resizeEnd = std::chrono::high_resolution_clock::now();
  // auto resizeStart = std::chrono::high_resolution_clock::now();
  // cv::Mat resized;
  // cv::resize(image, resized, cv::Size(), 0.5, 0.5);
  // auto resizeEnd = std::chrono::high_resolution_clock::now();

  // Gaussian blur
  auto gauussianStart = std::chrono::high_resolution_clock::now();
  cv::Mat blurred;
  cv::GaussianBlur(image, blurred, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
  auto gaussianEnd = std::chrono::high_resolution_clock::now();

  // Roberts Cross gradients -> sharpness image (float)
  auto robertsStart = std::chrono::high_resolution_clock::now();
  // cv::filter2D(image, img_x_preallocated, CV_16S, roberts_kernelx);
  // cv::filter2D(image, img_y_preallocated, CV_16S, roberts_kernely);
  cv::filter2D(blurred, img_x_preallocated, CV_16S, roberts_kernelx);
  cv::filter2D(blurred, img_y_preallocated, CV_16S, roberts_kernely);
  cv::multiply(img_x_preallocated, img_x_preallocated, img_x_squared_preallocated);
  cv::multiply(img_y_preallocated, img_y_preallocated, img_y_squared_preallocated);
  cv::add(img_x_squared_preallocated, img_y_squared_preallocated, sum_xy_preallocated);
  auto robertsEnd = std::chrono::high_resolution_clock::now();

  // Column means
  auto columnStart = std::chrono::high_resolution_clock::now();
  cv::Mat columnMeansMatrix, columnMeansBlurred;
  cv::reduce(sum_xy_preallocated, columnMeansMatrix, 0, cv::REDUCE_AVG, CV_32F);
  // Use the Mat data directly with pointer(no copy)
  int cols = columnMeansMatrix.cols;
  const float* colPtr = cols > 0 ? columnMeansMatrix.ptr<float>(0) : nullptr;
  auto columnEnd = std::chrono::high_resolution_clock::now();

  // Compute offset
  auto offsetStart = std::chrono::high_resolution_clock::now();
  const int offsetWindow = 50;
  double offset_left = 0.0, offset_right = 0.0;
  if (cols > 0 && colPtr) {
      int w = std::min(offsetWindow, cols);
      // Use OpenCV's optimized sum on column subranges (columnMeansMatrix is CV_32F)
      cv::Mat left = columnMeansMatrix.colRange(0, w);
      cv::Mat right = columnMeansMatrix.colRange(cols - w, cols);
      double leftSum = cv::sum(left)[0];
      double rightSum = cv::sum(right)[0];
      offset_left = leftSum / static_cast<double>(w);
      offset_right = rightSum / static_cast<double>(w);
  } else {
      offset_left = 0.0;
      offset_right = 0.0;
  }
  double offset = std::min(offset_left, offset_right);

  // Build x_values / y_values converting floats to doubles as needed
  std::vector<double> x_values;
  std::vector<double> y_values;
  x_values.reserve(cols);
  y_values.reserve(cols);
  for (int i = 0; i < cols; ++i) {
      double adjusted_value = (colPtr ? static_cast<double>(colPtr[i]) : 0.0) - offset;
      y_values.push_back(adjusted_value);
      x_values.push_back(static_cast<double>(i));
  }
  // Move the x_values
  const size_t n = y_values.size();
  std::transform(x_values.begin(), x_values.end(), x_values.begin(),
               [n](double x) { return (x / n) - 0.5; });

  // Find the max value and its location as initial guess
  // double y_max = *std::max_element(y_values.begin(), y_values.end());
  auto maxIt = std::max_element(y_values.begin(), y_values.end());
  double y_max = *maxIt;
  size_t idx = std::distance(y_values.begin(), maxIt);
  double x_at_y_max = x_values[idx];

  auto offsetEnd = std::chrono::high_resolution_clock::now();

  // Gaussian Fitting using GSL
  auto fittingStart = std::chrono::high_resolution_clock::now();
  // Construct data for GSL fitting
  struct data fit_data = { x_values.data(), y_values.data(), y_values.size() };

  // const size_t n = y_values.size();
  const size_t p = 3; // number of parameters: a, b, c
  gsl_vector *f = gsl_vector_alloc(n);
  gsl_vector *x = gsl_vector_alloc(p);
  gsl_multifit_nlinear_fdf fdf;
  gsl_multifit_nlinear_parameters fdf_params =
    gsl_multifit_nlinear_default_parameters();
  
  /* define function to be minimized */
  fdf.f = func_f;
  fdf.df = func_df;
  fdf.fvv = func_fvv;
  fdf.n = n;
  fdf.p = p;
  fdf.params = &fit_data;

  /* initial guess for parameters */
  gsl_vector_set(x, 0, y_max);
  gsl_vector_set(x, 1, x_at_y_max); // Use x at max y as initial guess for mean
  gsl_vector_set(x, 2, 0.25);

  fdf_params.trs = gsl_multifit_nlinear_trs_lmaccel;
  int niter = solve_system(x, &fdf, &fdf_params);

  // std::cout << "GSL initial params: a=" << y_max << ", b=" << n / 2.0 << ", c=" << n / 4.0 << std::endl;

  // std::cout << "GSL size of x,y: " << x_values.size() << ", " << y_values.size() << std::endl;

  double A = gsl_vector_get(x, 0);
  double B = gsl_vector_get(x, 1);
  double C = gsl_vector_get(x, 2);
  // double D = gsl_vector_get(x, 3);
  // printf("Fitted parameters: A=%.4f, B=%.4f, C=%.4f\n", A, B, C);
  gsl_vector_free(f);
  gsl_vector_free(x);

  B += 0.5; // Rescale back
  B *= n;
  C *= n;

  B = std::clamp(B, -30.0, 700.0); // Clamp B to valid range

  auto fittingEnd = std::chrono::high_resolution_clock::now();

  // Visualise
  if (bSaveImages) {
    lastSharpnessCurve = y_values;
    double locBestFocusDouble = B; // Mean from GSL fit
    SaveImagesPnG(image, locBestFocusDouble, A, C, 0, increment2);
    increment2++;

  }

  auto endTime = std::chrono::high_resolution_clock::now();

   // --- Timing log unchanged footprint, writing "com_time_us" as estimator time
  // ---
  auto totalTime =
      std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
  // auto resizeTime = std::chrono::duration_cast<std::chrono::microseconds>(
  //     resizeEnd - resizeStart);
  auto gaussianTime = std::chrono::duration_cast<std::chrono::microseconds>(
      gaussianEnd - gauussianStart);
  auto robertsTime = std::chrono::duration_cast<std::chrono::microseconds>(
      robertsEnd - robertsStart);
  auto columnTime = std::chrono::duration_cast<std::chrono::microseconds>(
      columnEnd - columnStart);
  auto offsetTime = std::chrono::duration_cast<std::chrono::microseconds>(
      offsetEnd - offsetStart);
  auto fittingTime = std::chrono::duration_cast<std::chrono::microseconds>(
      fittingEnd - fittingStart);

  try {
    std::ofstream GSLBenchmarkFile("../output/focus_benchmark_GSL.csv",
                                       std::ios::app);
    if (GSLBenchmarkFile.is_open() && GSLBenchmarkFile.good()) {
      auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
      // keep CSV header compatibility: write estimator time in the
      // "com_time_us" column
      GSLBenchmarkFile << timestamp << "," << totalTime.count() << ","
                           << gaussianTime.count() << ","
                           << robertsTime.count() << "," 
                           << columnTime.count() << ","
                           << offsetTime.count() << "," 
                           << fittingTime.count() << ","
                           << std::endl;
      GSLBenchmarkFile.close();
    }
  } catch (...) { /* ignore */
  }


  return B; // Return the mean (best focus position)
}



// Functor used in Eigen LM (see below) 
struct LMFunctor : Eigen::DenseFunctor<double> {
  const std::vector<double>& x_values;
  const std::vector<double>& y_values;

  LMFunctor(const std::vector<double>& x_values_, const std::vector<double>& y_values_)
      : Eigen::DenseFunctor<double>(3, x_values_.size()), x_values(x_values_), y_values(y_values_) {}


  // Eigen::MatrixXd measuredValues; // 2-column matrix: col 0 = x data, col 1 = y data

  // LMFunctor(const std::vector<double>& x_values_, const std::vector<double>& y_values_)
  //     : x_values(x_values_), y_values(y_values_) {}



    // int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const {
    //   return -1; // Tells Eigen to use numerical differentiation
    // }

    // Compute the jacobian
    int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const
    {
        // 'x' has dimensions n x 1
        // It contains the current estimates for the parameters.

        // 'fjac' has dimensions m x n
        // It will contain the jacobian of the errors, calculated numerically in this case.

        // double epsilon;
        // epsilon = 1e-8;

        // Analytical jacobian
        int n = x_values.size();
        fjac.resize(n, 3); // 3 parameters: a, b, c
        double a = x[0], b = x[1], c = x[2];
        for (int i = 0; i < n; ++i) {
            double ti = x_values[i];
            double zi = (ti - b) / c;
            double ei = exp(-0.5 * zi * zi);
            // Derivatives
            fjac(i, 0) = -ei; // d/dA
            fjac(i, 1) = -(a / c) * ei * zi; // d/dB
            fjac(i, 2) = -(a / c) * ei * zi * zi; // d/dC
            // fjac(i, 3) = -1.0; // d/dOffset
        }


        // Compute the jacobian using finite differences
        // for (int i = 0; i < x.size(); i++) {
        //     Eigen::VectorXd xPlus(x);
        //     xPlus(i) += epsilon;
        //     Eigen::VectorXd xMinus(x);
        //     xMinus(i) -= epsilon;

        //     Eigen::VectorXd fvecPlus(values());
        //     operator()(xPlus, fvecPlus);

        //     Eigen::VectorXd fvecMinus(values());
        //     operator()(xMinus, fvecMinus);

        //     Eigen::VectorXd fvecDiff(values());
        //     fvecDiff = (fvecPlus - fvecMinus) / (2.0f * epsilon);

        //     fjac.block(0, i, values(), 1) = fvecDiff;
        // }

        return 0;
    }


    // Compute residuals
    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
      int n = x_values.size();
      fvec.resize(n);
      double a = x[0], b = x[1], c = x[2]; // , offset=x[3];
      for (int i = 0; i < n; ++i) {
          double ti = x_values[i];
          double yi = y_values[i];
            double yfit = gaussian(a, b, c, ti);
            
            fvec[i] = yi - yfit;
        }
        return 0;
    }
};














// Implement Eigen LM - supposedly faster than GSL
double
autofocus::computeBestFocusEigenLM(cv::Mat image, int imgHeight,
                                          int imgWidth) {
  auto startTime = std::chrono::high_resolution_clock::now();

  // Resize to 1/2 x 1/2
  // auto resizeStart = std::chrono::high_resolution_clock::now();
  // cv::Mat resized;
  // cv::resize(image, resized, cv::Size(), 0.5, 0.5);
  // auto resizeEnd = std::chrono::high_resolution_clock::now();

  // Gaussian blur
  auto gaussianStart = std::chrono::high_resolution_clock::now();
  cv::Mat blurred;
  cv::GaussianBlur(image, blurred, cv::Size(3, 3), 1, 1, cv::BORDER_DEFAULT);
  auto gaussianEnd = std::chrono::high_resolution_clock::now();

  // Roberts Cross gradients -> sharpness image (float)
  auto robertsStart = std::chrono::high_resolution_clock::now();
  // cv::Mat img_x, img_y;
  cv::filter2D(blurred, img_x_preallocated, CV_16S, roberts_kernelx);
  cv::filter2D(blurred, img_y_preallocated, CV_16S, roberts_kernely);
  cv::Mat img_x_squared, img_y_squared, sum_xy;
  cv::multiply(img_x_preallocated, img_x_preallocated, img_x_squared_preallocated);
  cv::multiply(img_y_preallocated, img_y_preallocated, img_y_squared_preallocated);
  cv::add(img_x_squared_preallocated, img_y_squared_preallocated, sum_xy_preallocated);
  auto robertsEnd = std::chrono::high_resolution_clock::now();

  // Column means
  auto columnStart = std::chrono::high_resolution_clock::now();
  cv::Mat columnMeansMatrix;
  cv::reduce(sum_xy_preallocated, columnMeansMatrix, 0, cv::REDUCE_AVG, CV_32F);
  // Use the Mat data directly with pointer(no copy)
  int cols = columnMeansMatrix.cols;
  const float* colPtr = cols > 0 ? columnMeansMatrix.ptr<float>(0) : nullptr;
  auto columnEnd = std::chrono::high_resolution_clock::now();


  // Moving Average
  auto MovAvgStart = std::chrono::high_resolution_clock::now();
  // Convolution with 1/movAvgWindow kernel
  const int movAvgWindow = 20;
  cv::Mat kernel = cv::Mat::ones(1, movAvgWindow, CV_32F) / static_cast<float>(movAvgWindow);
  cv::Mat movAvgResult;
  cv::filter2D(columnMeansMatrix, movAvgResult, -1, kernel);

  double min_signal, max_signal;
  cv::minMaxLoc(columnMeansMatrix, &min_signal, &max_signal);

  // Calculate Signal to Noise Ratio (SNR)

  // signal = max - min
  double signal = max_signal - min_signal;

  // noise = std(raw signal - moving average)
  cv::Mat Diff;
  cv::subtract(columnMeansMatrix, movAvgResult, Diff);
  double meanNoise = cv::mean(Diff)[0];
  double sq_sum = cv::sum(Diff.mul(Diff))[0];
  double varNoise = (sq_sum / Diff.total()) - (meanNoise * meanNoise);
  double noise = std::sqrt(varNoise);

  // SNR
  double SNR = (noise > 0.0) ? (signal / noise) : 0.0;

  // std::cout << "Signal: " << signal << ", Noise: " << noise << ", SNR: " << SNR << std::endl;
  
  auto movAvgEnd = std::chrono::high_resolution_clock::now();



  // Compute offset
  auto offsetStart = std::chrono::high_resolution_clock::now();

  // Moving Average
  double offset = min_signal;
  // const int offsetWindow = 50;
  // double offset_left = 0.0, offset_right = 0.0;

  // if (cols > 0 && colPtr) {
  //     int w = std::min(offsetWindow, cols);
  //     // Use OpenCV's optimized sum on column subranges (columnMeansMatrix is CV_32F)
  //     cv::Mat left = columnMeansMatrix.colRange(0, w);
  //     cv::Mat right = columnMeansMatrix.colRange(cols - w, cols);
  //     double leftSum = cv::sum(left)[0];
  //     double rightSum = cv::sum(right)[0];
  //     offset_left = leftSum / static_cast<double>(w);
  //     offset_right = rightSum / static_cast<double>(w);


  //     // calculate noise as std of min(left, right)

  //     if (offset_left < offset_right) {
  //         // left is smaller
  //         cv::Mat left_sq;
  //         cv::multiply(left, left, left_sq);
  //         sq_sum = cv::sum(left_sq)[0];
  //         varNoise = (sq_sum / static_cast<double>(w)) - (offset_left * offset_left);
  //         noise = std::sqrt(varNoise);
  //         SNR = (noise > 0.0) ? (signal / noise) : 0.0;


        
  //     } else {
  //         // right is smaller
  //         cv::Mat right_sq;
  //         cv::multiply(right, right, right_sq);
  //         sq_sum = cv::sum(right_sq)[0];
  //         varNoise = (sq_sum / static_cast<double>(w)) - (offset_right * offset_right);
  //         noise = std::sqrt(varNoise);
  //         SNR = (noise > 0.0) ? (signal / noise) : 0.0;

  //     }
  // } else {
  //     offset_left = 0.0;
  //     offset_right = 0.0;
  // }
  // double offset = std::min(offset_left, offset_right);


  // Compute offset - after Moving Average
  // const int offsetWindow = 50;
  // double offset_left = 0.0, offset_right = 0.0;
  // int w = std::min(offsetWindow, static_cast<int>(sharpnesscurve.size()));
  // if (w > 0) {
  //   offset_left = std::accumulate(sharpnesscurve.begin(), sharpnesscurve.begin() + w, 0.0) / static_cast<double>(w);
  //   offset_right = std::accumulate(sharpnesscurve.end() - w, sharpnesscurve.end(), 0.0) / static_cast<double>(w);
  // } else {
  //   offset_left = 0.0;
  //   offset_right = 0.0;
  // }
  // double offset = std::min(offset_left, offset_right);

  // Build x_values / y_values converting floats to doubles as needed
  std::vector<double> x_values;
  std::vector<double> y_values;
  x_values.reserve(cols);
  y_values.reserve(cols);
  for (int i = 0; i < cols; ++i) {
      double adjusted_value = (colPtr ? static_cast<double>(colPtr[i]) : 0.0) - offset;
      y_values.push_back(adjusted_value);
      x_values.push_back(static_cast<double>(i));

      // after moving average
      // double adjusted_value = (sharpnesscurve[i] - offset);
      // y_values.push_back(adjusted_value);
      // x_values.push_back(static_cast<double>(i));
  }

  // // Normalise x to [-0.5, 0.5] range
  int n = y_values.size();
  std::transform(
      x_values.begin(), x_values.end(), x_values.begin(),
      [n](double x) { return x / static_cast<double>(n) - 0.5; }
  );

  double y_max = *std::max_element(y_values.begin(), y_values.end());
  // double y_min = *std::min_element(y_values.begin(), y_values.end());

  // Get x value at y max
  auto maxIt = std::max_element(y_values.begin(), y_values.end());
  size_t idx = std::distance(y_values.begin(), maxIt);
  double x_at_y_max = x_values[idx];

  auto offsetEnd = std::chrono::high_resolution_clock::now();



  // Gaussian Fitting using Eigen LM
  auto fittingStart = std::chrono::high_resolution_clock::now();

  int p = 3; // number of parameters: a, b, c
  
  Eigen::VectorXd params(p);
  params << y_max, x_at_y_max, 1.0 / 4.0; //, y_min; // Initial guess

  // std::cout << "Initial D: " << y_min << std::endl;

  int max_iterations = 500;
  double epsilon = 1e-5;
  double epsilon_g = 1e-8;



  LMFunctor functor(x_values, y_values);
  Eigen::LevenbergMarquardt<LMFunctor> lm(functor);
  // Eigen::NumericalDiff<LMFunctor> numDiff(functor);

  // Eigen::LevenbergMarquardt<Eigen::NumericalDiff<LMFunctor>> lm(numDiff);

  lm.setXtol(epsilon);
  lm.setMaxfev(max_iterations);
  lm.setGtol(epsilon_g);
  lm.setFtol(epsilon);




  int status = lm.minimize(params);


  double A, B, C;
  A = params(0);
  B = params(1);
  C = params(2);


  // std::cout << "Status: " << status << ", iterations: " << lm.nfev() << std::endl;
  // std::cout << "Eigen LM fitted parameters: A=" << A << ", B=" << B << ", C=" << C << std::endl;

  // return B and C to original range
  B = (B + 0.5) * n; // Convert back to original range
  C = C * n; // Scale C back to original range
    
  // If B less than -30 or greater than 700, use previous value and log warning
  if (B < -30.0 || B > 700.0) {
    if (bAutofocusLogFlag) {
      logger->warn("[autofocus::computeBestFocusEigenLM] Fitted B out of range: {}. Using previous value: {}",
                    B, lastValidB);
    }

    B = lastValidB; // Use previous valid value


    // Debugging
    // std::cout << "Warning: B original: " << B << std::endl;
    // std::cout << "Status: " << status << ", iterations: " << lm.nfev() << std::endl;
  
  }
  else {
    lastValidB = B; // Update last valid B
  }


  // B = std::clamp(B, -320.0, 960.0); // Clamp B to valid range

  auto fittingEnd = std::chrono::high_resolution_clock::now();

  // Visualise
  if (bSaveImages) {
    lastSharpnessCurve = y_values;
    std::vector<double> movAvgCurve;
    movAvgResult.reshape(1,1).copyTo(movAvgCurve);
    lastMovAvgCurve = movAvgCurve;
    double locBestFocusDouble = B; // Mean from fit
    SaveImagesPnG(image, locBestFocusDouble, A, C, SNR, increment2, status);
  }

  if (bSaveSharpnessCurves) {
    // Implement this
    lastSharpnessCurve = y_values;
  }

  auto endTime = std::chrono::high_resolution_clock::now();

   // --- Timing log // ---
  auto totalTime =
      std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
  // auto resizeTime = std::chrono::duration_cast<std::chrono::microseconds>(
      // resizeEnd - resizeStart);
  auto gaussianTime = std::chrono::duration_cast<std::chrono::microseconds>(
      gaussianEnd - gaussianStart);
  auto robertsTime = std::chrono::duration_cast<std::chrono::microseconds>(
      robertsEnd - robertsStart);
  auto columnTime = std::chrono::duration_cast<std::chrono::microseconds>(
      columnEnd - columnStart);
  auto movAvgTime = std::chrono::duration_cast<std::chrono::microseconds>(
      movAvgEnd - MovAvgStart);
  auto offsetTime = std::chrono::duration_cast<std::chrono::microseconds>(
      offsetEnd - offsetStart);
  auto fittingTime = std::chrono::duration_cast<std::chrono::microseconds>(
      fittingEnd - fittingStart);

  try {
    std::ofstream EigenLMBenchmarkFile("../output/focus_benchmark_EigenLM.csv",
                                       std::ios::app);
    if (EigenLMBenchmarkFile.is_open() && EigenLMBenchmarkFile.good()) {
      auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
      // keep CSV header compatibility: write estimator time in the
      // "com_time_us" column
      EigenLMBenchmarkFile << timestamp << "," << totalTime.count() << ","
                          //  << resizeTime.count() << ","
                            << gaussianTime.count() << ","
                           << robertsTime.count() << "," 
                           << columnTime.count() << ","
                           << movAvgTime.count() << ","
                           << offsetTime.count() << "," 
                           << fittingTime.count() << ","
                           << lm.nfev() << ","
                           << std::endl;
      EigenLMBenchmarkFile.close();
    }
  } catch (...) { /* ignore */
  }


  return B; // Return the mean (best focus position)
}





// Fast integer power helper function
static inline double powi_pos(double x, int p) {
  // fast integer power, p>=1
  double r = 1.0;
  while (p > 0) {
    if (p & 1)
      r *= x;
    x *= x;
    p >>= 1;
  }
  return r;
}



void autofocus::reloadSettings() {
  try {
    // Reload settings from file
    settings.load();

    // Update internal member variables with new values
    // MIN_POSITION = settings.getMinPosition();
    // MAX_POSITION = settings.getMaxPosition();
    Kd = settings.getKd();
    Kp = settings.getKp();

    if (bAutofocusLogFlag) {
      logger->info("[autofocus::reloadSettings] Settings reloaded - Kd: {}, Kp: {}",
                   Kd, Kp);
    }
  } catch (const std::exception &e) {
    if (bAutofocusLogFlag) {
      logger->error("[autofocus::reloadSettings] Error reloading settings: {}",
                    e.what());
    }
  }
}

// Helper function to save images (png files)
// Input: resized image, locBestFocusDouble, 
//        Gaussian fit parameters: A, C, (B is equal to locBestFocusDouble)
//        increment counter

void autofocus::SaveImagesPnG(cv::Mat &image, double locBestFocusDouble, double A, double C, double signal, int& increment2, int status) {


  cv::Mat colorResized, combined;
  if (!lastSharpnessCurve.empty()) {
    try {
      // Convert resized to color if it's grayscale to match the graph image
      // type
      if (image.channels() == 1) {
        cv::cvtColor(image, colorResized, cv::COLOR_GRAY2BGR);
      } else {
        colorResized = image.clone();
    }

    // Create graph image with same width as resized image
    int graphHeight = 200;
    int graphWidth = colorResized.cols;
    cv::Mat graphImage =
        cv::Mat::zeros(graphHeight, graphWidth, CV_8UC3);

    int curveWidth = lastSharpnessCurve.size();

    // Find max value in sharpness curve for scaling
    double maxSharpness =
        lastSharpnessCurve.empty()
            ? 0.0
            : *std::max_element(lastSharpnessCurve.begin(),
                                lastSharpnessCurve.end());
    double minSharpness =
        lastSharpnessCurve.empty()
            ? 0.0
            : *std::min_element(lastSharpnessCurve.begin(),
                                lastSharpnessCurve.end());
    double range = maxSharpness - minSharpness;

    // Set y-axis range
    double yAxisMin = 0.0;
    double yAxisMax = 255.0;

    // Scale based on sharpness image
    // double maxVal = yAxisMax;
    // if (maxVal <= 0) {
    //   maxVal = 1.0;
    // }

    // Draw sharpness curve (blue)
    for (size_t i = 1; i < lastSharpnessCurve.size(); i++) {
      int x1, x2;

      // If we have specific x indices, use them
      if (!lastXIndices.empty()) {
        x1 = lastXIndices[i - 1];
        x2 = lastXIndices[i];
      } else {
        x1 = (i - 1);
        x2 = i;
      }

      // Only draw if within graph bounds
      if (x1 >= 0 && x2 < graphWidth) {
        int y1 =
            graphHeight -
            static_cast<int>((lastSharpnessCurve[i - 1] / yAxisMax) *
                              (graphHeight - 50));
        int y2 = graphHeight -
                  static_cast<int>((lastSharpnessCurve[i] / yAxisMax) *
                                  (graphHeight - 50));
        cv::line(graphImage, cv::Point(x1, y1), cv::Point(x2, y2),
                  cv::Scalar(255, 0, 0), 1); // Blue
      }
    }


    // Draw Gaussian fit (red)
    double B = locBestFocusDouble; // Use locBestFocusDouble from EigenLM fit
    std::vector<double> fittedCurve(lastSharpnessCurve.size());
    for (size_t i = 0; i < lastSharpnessCurve.size(); i++) {
      fittedCurve[i] = gaussian(A, B, C, static_cast<double>(i));
    }

    // Scale Gaussian to match sharpness curve range
    for (size_t i = 1; i < fittedCurve.size(); ++i) {
        int x1 = i - 1;
        int x2 = i;
        int y1 = graphHeight - static_cast<int>((fittedCurve[i - 1] / yAxisMax) * (graphHeight - 50));
        int y2 = graphHeight - static_cast<int>((fittedCurve[i] / yAxisMax) * (graphHeight - 50));
        // Draw only if within bounds
        if (x1 >= 0 && x2 < graphWidth) {
            cv::line(graphImage, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 1); // Red
        }
    }

    // Draw moving average curve
    if (!lastMovAvgCurve.empty()) {
      for (size_t i = 1; i < lastMovAvgCurve.size(); i++) {
        int x1 = i - 1;
        int x2 = i;
        int y1 = graphHeight - static_cast<int>((lastMovAvgCurve[i - 1] / yAxisMax) * (graphHeight - 50));
        int y2 = graphHeight - static_cast<int>((lastMovAvgCurve[i] / yAxisMax) * (graphHeight - 50));
        // Draw only if within bounds
        if (x1 >= 0 && x2 < graphWidth) {
            cv::line(graphImage, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 1); // Green
        }
        }
    }

    // Draw vertical line for locBestFocusDouble
    cv::line(graphImage, cv::Point(locBestFocusDouble, 0),
              cv::Point(locBestFocusDouble, graphHeight),
              cv::Scalar(255, 255, 0), 2);


    // Find amplitude of locBestFocus
    int sharpnessIdx = static_cast<int>(std::round(locBestFocusDouble));
    double sharpnessAtBest = (sharpnessIdx >= 0 && sharpnessIdx < lastSharpnessCurve.size())
          ? lastSharpnessCurve[sharpnessIdx]
          : 0.0;

    // Display range, COM, amplitude of BestFocus position with double precision

        std::string rangeText =
            "Range: " + std::to_string(range).substr(0, 6);
      std::string comText =
          "LoBF index: " + std::to_string(locBestFocusDouble).substr(0, 8);
      std::string sharpnessText = "LoBF sharpness: " + std::to_string(sharpnessAtBest).substr(0, 8);

      std::string SignalText = "SNR: " + std::to_string(signal).substr(0, 6);

      // Display status text on graph
      std::string statusText = "Status: " + std::to_string(status);

    // Add legend text and amplitude
    cv::putText(graphImage, "Sharpness Curve", cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0),
                1);
    cv::putText(graphImage, "Gaussian Fit", cv::Point(10, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    cv::putText(graphImage, "Location of Best Focus", cv::Point(175, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255, 255, 0), 1);

    // Place the values just below the legend
    cv::putText(graphImage, sharpnessText, cv::Point(400, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
    cv::putText(graphImage, rangeText, cv::Point(175, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
    cv::putText(graphImage, comText, cv::Point(400, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);

    cv::putText(graphImage, SignalText, cv::Point(175, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);

    cv::putText(graphImage, statusText, cv::Point(400, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
    
    // Draw axes on the graphImage
    int tickLength = 6;
    int numXTicks = 6;
    int numYTicks = 4;

    // Draw x-axis (horizontal line at the bottom)
    cv::line(graphImage, cv::Point(0, graphHeight - 1), 
      cv::Point(graphWidth - 1, graphHeight - 1), cv::Scalar(200, 200, 200), 1);
    
    // Draw y-axis (vertical line at the left)
    cv::line(graphImage, cv::Point(0, 0), cv::Point(0, graphHeight - 1), cv::Scalar(200, 200, 200), 1);

    double x_min, x_max;
    if (!lastXIndices.empty()) {
      x_min = *std::min_element(lastXIndices.begin(), lastXIndices.end());
      x_max = *std::max_element(lastXIndices.begin(), lastXIndices.end());
    } else {
      x_min = 0;
      x_max = lastSharpnessCurve.size() - 1;
    }

    // Draw x-axis ticks and optional labels
    for (int i = 0; i <= numXTicks; ++i) {
      double xValue = x_min + (x_max - x_min) * i / numXTicks;
      int x = static_cast<int>(xValue);
      cv::line(graphImage, cv::Point(x, graphHeight - 1), 
        cv::Point(x, graphHeight - 1 - tickLength), 
        cv::Scalar(200, 200, 200), 1);
      cv::putText(graphImage, std::to_string(static_cast<int>(xValue)), 
                  cv::Point(x + 2, graphHeight - 1 - 5), 
                  cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(180,180,180), 1);
    }

    for (int i = 0; i <= numYTicks; ++i) {
        int y = graphHeight - static_cast<int>(i * (graphHeight - 50) / numYTicks);
        double labelValue = yAxisMin + (yAxisMax - yAxisMin) * i / numYTicks;
        cv::line(graphImage, cv::Point(0, y), cv::Point(tickLength, y), cv::Scalar(200, 200, 200), 1);
        cv::putText(graphImage, cv::format("%.1f", labelValue), cv::Point(2, y - 2),
                    cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(180,180,180), 1);
    }

    // Combine image and graph
    cv::vconcat(colorResized, graphImage, combined);

    // Save with integer filename
    std::string FilePath = "../output/TiltedCam_Images_png/" +
                            std::to_string(increment2) + "_" +
                            std::to_string(static_cast<int>(
                                std::round(locBestFocusDouble))) +
                            ".png";
    cv::imwrite(FilePath, combined);
    } catch (const std::exception &e) {
      // Just save the original image if we can't combine
      std::string FilePath = "../output/TiltedCam_Images_png/" +
                              std::to_string(increment2) + "_" +
                              std::to_string(static_cast<int>(
                                  std::round(locBestFocusDouble))) +
                              ".png";
      cv::imwrite(FilePath, colorResized);
      std::cout << "Error combining images: " << e.what() << std::endl;
    }
    
  } else {
    // Just save the original image if no curve data
    std::string FilePath = "../output/TiltedCam_Images_png/" +
                            std::to_string(increment2) + "_" +
                            std::to_string(static_cast<int>(
                                std::round(locBestFocusDouble))) +
                            ".png";
    cv::imwrite(FilePath, colorResized);
  }  
  increment2++;
}

// Helper function to save sharpness curves as text files
// input: resized image, image after roberts cross (sharpness_float),
//        column means vector, sharpness curve vector, increment counter
void autofocus::SaveSharpnessTxt(const cv::Mat &resized,  const std::vector<double>& x_values, const std::vector<double>& y_values, double A, double B, double C, int& niter, int& increment) {
    std::string fileName = "SHARPNESS_CURVE_" + std::to_string(increment);
    std::string TextFile1 = "../output/SharpnessCurves/" + fileName + "_x_values.txt";
    std::string TextFile2 = "../output/SharpnessCurves/" + fileName + "_y_values.txt";
    std::string TextFile3 = "../output/SharpnessCurves/" + fileName + "_fit_params.txt";

    std::ofstream outputFile1(TextFile1);
    std::ofstream outputFile2(TextFile2);
    std::ofstream outputFile3(TextFile3);

    for (const auto& val : x_values) {
        outputFile1 << val << "\n";
    }
    for (const auto& val : y_values) {
        outputFile2 << val << "\n";
    }
    outputFile3 << "A: " << A << "\n";
    outputFile3 << "B: " << B << "\n";
    outputFile3 << "C: " << C << "\n";
    outputFile3 << "Niter: " << niter << "\n";

  


    outputFile1.close();
    outputFile2.close();
    outputFile3.close();

    increment++;

  }


  void autofocus::logEvent(const std::string& event, const std::string& details) {
    if (debugLogFile.is_open()) {
      auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
      debugLogFile << std::put_time(std::localtime(&now), "%F %T") << "," << event << "," << details << "\n";
      debugLogFile.flush();
  }
}