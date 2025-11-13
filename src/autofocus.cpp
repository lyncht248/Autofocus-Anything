#include "autofocus.hpp"
// #include "tiltedcam.hpp"
// #include "lens.hpp"
#include "ASICamera2.h" //TODO: Remove this when you move capturevideo() to tiltedcam.cc
#include "logfile.hpp"
#include "main.hpp"
#include "mainwindow.hpp"

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
#include <gsl/gsl_multifit_nlin.h>


#include "opencv2/highgui/highgui.hpp"

#include "settings.hpp"

// Global variables

// 0.0057 mm per pixel is the average!!!!

bool bAutofocusLogFlag = 0; // Flag that is 1 for when the autofocus log is
                            // being written to, 0 when it is not

std::atomic<bool> bNewImage = 0; // Flag that is 1 for when the buffer image is
                                 // new, 0 when buffer image is old

const long img_size = 1280 * 960; // Replace with actual image size
bool bSaveImages =
    0; // Saves images from the tilted camera to output folder. WARNING: will
       // produce enourmous number of images and slow down the system!
bool bSaveSharpnessCurves =
    1; // Saves text files with the sharpness curve data, similar to above

bool bSaveAllImages = 1; // Saves all images, not just during autofocus-ing
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
int increment = 0;

std::vector<double> lastSharpnessCurve;
std::vector<double> lastFittedCurve;

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

  // Initialize reduced resolution benchmark CSV file with headers
  std::ofstream reducedBenchmarkFile("../output/focus_benchmark_reduced.csv");
  if (reducedBenchmarkFile.is_open()) {
    reducedBenchmarkFile
        << "timestamp,total_time_us,resize_time_us,clahe_time_us,blur_time_us,"
           "roberts_time_us,column_time_us,sliding_time_us,com_time_us"
        << std::endl;
    reducedBenchmarkFile.close();
  }

  // Initialize very reduced resolution benchmark CSV file with headers
  std::ofstream veryReducedBenchmarkFile(
      "../output/focus_benchmark_very_reduced.csv");
  if (veryReducedBenchmarkFile.is_open()) {
    veryReducedBenchmarkFile
        << "timestamp,total_time_us,resize_time_us,clahe_time_us,blur_time_us,"
           "roberts_time_us,column_time_us,sliding_time_us,fitting_time_us"
        << std::endl;
    veryReducedBenchmarkFile.close();
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
  sharpness_float_preallocated = cv::Mat::zeros(imHeight, imWidth, CV_32F);
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

  if (bAutofocusLogFlag) {
    logger->info("[autofocus::run] while thread loop about to start");
  }

  while (!stop_thread.load()) {

    // Sophia - debug
    if (bSaveAllImages) {
      // Check if we have a new frame
      if (tiltedcam1.getLatestFrame(img_calc_buf, img_size)) {
        framesProcessed++;
        bNewImage = true;
        imgcount++;
        imgcountfile++;
        // Convert to OpenCV Mat - use reduced resolution for all processing
        cv::Mat image(imHeight, imWidth, CV_8UC1, img_calc_buf);

        double locBestFocusDouble = computeBestFocusReduced(
            image, imHeight, imWidth); //  returns double

        // Store the current measured focus position globally
        currentMeasuredFocus.store(locBestFocusDouble);

        // resize image to 1/2 resolution
        cv::Mat image_resized_for_save;
        cv::resize(image, image_resized_for_save, cv::Size(), 0.5, 0.5);

        // Apply CLAHE to the image for saving (same as used in
        // computeBestFocus)
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(2.0);
        clahe->setTilesGridSize(cv::Size(4, 4));
        cv::Mat clahe_enhanced_for_save;
        clahe->apply(image_resized_for_save, clahe_enhanced_for_save);

        // Apply Gaussian blur to the image for saving
        cv::Mat processed_img;
        cv::GaussianBlur(clahe_enhanced_for_save,
                          processed_img, cv::Size(3, 3), 1, 1,
                          cv::BORDER_DEFAULT);

        // try without CLAHE - bigger blur
        // cv::GaussianBlur(image_resized_for_save,
        //                   processed_img, cv::Size(7, 7), 1, 1,
        //                   cv::BORDER_DEFAULT);


        // locBestFocus is already in reduced resolution coordinates, so no
        // additional scaling needed
        int scaledLocBestFocus =
            static_cast<int>(std::round(locBestFocusDouble));

        // Draw vertical line at best focus position
        cv::Point p1(scaledLocBestFocus, 0),
            p2(scaledLocBestFocus, processed_img.rows);

          // Save raw image
        std::string FilePath = "../output/TiltedCam_Images_raw/" +
                                std::to_string(imgcountfile - 1) + ".png";
        cv::imwrite(FilePath, image);
        
          // Save reduced, post-CLAHE image
        FilePath = "../output/TiltedCam_Images_CLAHE/" +
                    std::to_string(imgcountfile - 1) + ".png";
        cv::imwrite(FilePath, clahe_enhanced_for_save);

// Save processed image with sharpness and LoBF overlay
        // Convert processed_img to color if it's grayscale to match the graph image
        // type
        cv::Mat colorResized;
        if (processed_img.channels() == 1) {
          cv::cvtColor(processed_img, colorResized, cv::COLOR_GRAY2BGR);
        } else {
          colorResized = processed_img.clone();
        }

        // Draw vertical line for center of mass if available - adjust for
        // curve start position
        // if (lastCenterOfMass >= 0) {
        //   int comX = 10 + static_cast<int>(
        //                       lastCenterOfMass); // Add kernel/2 offset (10
        //                                           // for reduced resolution)
        //   if (comX >= 0 && comX < colorResized.cols) {
        //     cv::Point comP1(comX, 0);
        //     cv::Point comP2(comX, colorResized.rows);
        //     cv::line(colorResized, comP1, comP2, cv::Scalar(255, 255, 0),
        //               2); // Cyan line for COM
        //   }
        // }


        // draw vertical line for locBestFocusDouble
            cv::line(colorResized, cv::Point(locBestFocusDouble, 0),
                      cv::Point(locBestFocusDouble, colorResized.rows),
                      cv::Scalar(255, 255, 0), 2);


        cv::Mat combined;

        // Draw graph if we have curve data
        if (!lastSharpnessCurve.empty()) {
          try {
            // Create graph image with same width as resized image
            int graphHeight = 200;
            int graphWidth = colorResized.cols;
            cv::Mat graphImage =
                cv::Mat::zeros(graphHeight, graphWidth, CV_8UC3);

            
              // The sharpness curve starts at kernel/2 and has (imageWidth -
              // kernel) points For reduced resolution: starts at 10, has (640/2
              // - 20) = 300 points
              int curveStartX =
                  10; // kernel/2 for reduced resolution (20/2 = 10)
              int curveWidth = lastSharpnessCurve.size();



              // // Find max value in sharpness curve for scaling
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

              // Scale based on sharpness curve range
              // double maxVal = maxSharpness;


              // Set fixed y-axis range
              double yAxisMin = 0.0;
              double yAxisMax = 1300.0;
              double maxVal = yAxisMax;


              if (maxVal <= 0)
                maxVal = 1.0;

              // Draw sharpness curve (blue) - map curve indices to correct x
              // positions
              for (size_t i = 1; i < lastSharpnessCurve.size(); i++) {
                int x1 = curveStartX + (i - 1);
                int x2 = curveStartX + i;

                // Only draw if within graph bounds
                if (x1 >= 0 && x2 < graphWidth) {
                  int y1 =
                      graphHeight -
                      static_cast<int>((lastSharpnessCurve[i - 1] / maxVal) *
                                       (graphHeight - 50));
                  int y2 = graphHeight -
                           static_cast<int>((lastSharpnessCurve[i] / maxVal) *
                                            (graphHeight - 50));
                  cv::line(graphImage, cv::Point(x1, y1), cv::Point(x2, y2),
                           cv::Scalar(255, 0, 0), 1); // Blue
                }
              }


            // Draw vertical line for center of mass on graph - adjust for
            // curve start position
            // if (lastCenterOfMass >= 0) {
            //   int comX = curveStartX + static_cast<int>(lastCenterOfMass);
            //   if (comX >= 0 && comX < graphWidth) {
            //     cv::line(graphImage, cv::Point(comX, 0),
            //               cv::Point(comX, graphHeight),
            //               cv::Scalar(255, 255, 0), 2);
            //   }
          //  }

// draw vertical line for locBestFocusDouble
            cv::line(graphImage, cv::Point(locBestFocusDouble, 0),
                      cv::Point(locBestFocusDouble, graphHeight),
                      cv::Scalar(255, 255, 0), 2); 

      // Find amplitude of locBestFocus
            int sharpnessIdx = static_cast<int>(std::round(locBestFocusDouble)) - curveStartX;
            double sharpnessAtBest = (sharpnessIdx >= 0 && sharpnessIdx < lastSharpnessCurve.size())
                  ? lastSharpnessCurve[sharpnessIdx]
                  : 0.0;

            // Display range, COM, amplitude of BestFocus position with double precision
             
            std::string rangeText =
                "Range: " + std::to_string(range).substr(0, 6);
            std::string comText =
                "COM: " + std::to_string(locBestFocusDouble).substr(0, 8);
            std::string sharpnessText = "LoBF sharpness: " + std::to_string(sharpnessAtBest).substr(0, 8);
            

            // Add legend text and amplitude
            cv::putText(graphImage, "Sharpness Curve", cv::Point(10, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0),
                        1);
            cv::putText(graphImage, "Center of Mass", cv::Point(10, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(255, 255, 0), 1);

            // Place the values just below the legend
            cv::putText(graphImage, sharpnessText, cv::Point(200, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            cv::putText(graphImage, rangeText, cv::Point(450, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            cv::putText(graphImage, comText, cv::Point(200, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);


//             cv::putText(graphImage, sharpnessText, cv::Point(10, graphHeight - 60),
//                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
//             cv::putText(
//                 graphImage, rangeText, cv::Point(10, graphHeight - 40),
//                 cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 0.5);
//             cv::putText(graphImage, comText, cv::Point(10, graphHeight - 20),
//                         cv::FONT_HERSHEY_SIMPLEX, 0.5,
//                         cv::Scalar(0, 255, 255), 0.5);

// // //axes
            
//             // Draw axes on the graphImage
            int tickLength = 6;
            int numXTicks = 6;
            int numYTicks = 4;
//             double plotHeight = graphHeight - 50; // Leave space for x-axis labels
//             double yOffset = 50;

            // Draw x-axis (horizontal line at the bottom)
            cv::line(graphImage, cv::Point(0, graphHeight - 1), cv::Point(graphWidth - 1, graphHeight - 1), cv::Scalar(200, 200, 200), 1);
            // cv::line(graphImage, cv::Point(0, graphHeight),
            //           cv::Point(graphWidth - 1, graphHeight),
            //           cv::Scalar(200, 200, 200), 1);

            // Draw y-axis (vertical line at the left)
            cv::line(graphImage, cv::Point(0, 0), cv::Point(0, graphHeight - 1), cv::Scalar(200, 200, 200), 1);
            // cv::line(graphImage, cv::Point(0, 0),
            //         cv::Point(0, graphHeight - 50),
            //         cv::Scalar(200, 200, 200), 1);


            // Draw x-axis ticks and optional labels
            for (int i = 0; i <= numXTicks; ++i) {
                int x = i * (graphWidth - 1) / numXTicks;
                cv::line(graphImage, cv::Point(x, graphHeight - 1), 
                        cv::Point(x, graphHeight - 1 - tickLength), 
                        cv::Scalar(200, 200, 200), 1);
                int labelValue = i * (curveWidth - 1) / numXTicks;
                cv::putText(graphImage, std::to_string(labelValue), 
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

//axes



            // Combine image and graph
            cv::vconcat(colorResized, graphImage, combined);

            // Save with integer filename
            std::string FilePath = "../output/TiltedCam_Images/" +
                                    std::to_string(imgcountfile - 1) + "_" +
                                    std::to_string(static_cast<int>(
                                        std::round(locBestFocusDouble))) +
                                    ".png";
            cv::imwrite(FilePath, combined);
          } catch (const std::exception &e) {
            // Just save the original image if we can't combine
            std::string FilePath = "../output/TiltedCam_Images/" +
                                    std::to_string(imgcountfile - 1) + "_" +
                                    std::to_string(static_cast<int>(
                                        std::round(locBestFocusDouble))) +
                                    ".png";
            cv::imwrite(FilePath, colorResized);
          }
        } else {
          // Just save the original image if no curve data
          std::string FilePath = "../output/TiltedCam_Images/" +
                                  std::to_string(imgcountfile - 1) + "_" +
                                  std::to_string(static_cast<int>(
                                      std::round(locBestFocusDouble))) +
                                  ".png";
          cv::imwrite(FilePath, colorResized);
        }
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

        double locBestFocusDouble = computeBestFocusReduced(
            image, imHeight, imWidth); //  returns double

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
        std::cout << "locBestFocus: " << locBestFocusDouble
                  << ", desiredLocBestFocus: " << desiredLocBestFocus
                  << ", FPS: " << std::fixed << std::setprecision(1) << fps
                  << std::endl;

        lastTime = currentTime;

        // Use the reduced resolution result for all autofocus logic
        if (bSaveImages) {
          // Apply CLAHE to the image for saving (same as used in
          // computeBestFocus)
          cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
          clahe->setClipLimit(2.0);
          clahe->setTilesGridSize(cv::Size(4, 4));
          cv::Mat clahe_enhanced_for_save;
          clahe->apply(image, clahe_enhanced_for_save);

          // Apply Gaussian blur to the image for saving
          cv::Mat blurred_preallocated_for_save;
          cv::GaussianBlur(clahe_enhanced_for_save,
                           blurred_preallocated_for_save, cv::Size(3, 3), 1, 1,
                           cv::BORDER_DEFAULT);

          // resize image to 1/2 resolution
          cv::Mat resized;
          cv::resize(blurred_preallocated_for_save, resized, cv::Size(), 0.5,
                     0.5);

          // locBestFocus is already in reduced resolution coordinates, so no
          // additional scaling needed
          int scaledLocBestFocus =
              static_cast<int>(std::round(locBestFocusDouble));

          // Draw vertical line at best focus position
          cv::Point p1(scaledLocBestFocus, 0),
              p2(scaledLocBestFocus, resized.rows);

          // Convert resized to color if it's grayscale to match the graph image
          // type
          cv::Mat colorResized;
          if (resized.channels() == 1) {
            cv::cvtColor(resized, colorResized, cv::COLOR_GRAY2BGR);
          } else {
            colorResized = resized.clone();
          }

          // Draw vertical line for center of mass if available - adjust for
          // curve start position
          if (lastCenterOfMass >= 0) {
            int comX = 10 + static_cast<int>(
                                lastCenterOfMass); // Add kernel/2 offset (10
                                                   // for reduced resolution)
            if (comX >= 0 && comX < colorResized.cols) {
              cv::Point comP1(comX, 0);
              cv::Point comP2(comX, colorResized.rows);
              cv::line(colorResized, comP1, comP2, cv::Scalar(255, 255, 0),
                       2); // Cyan line for COM
            }
          }

          cv::Mat combined;

          // Draw graph if we have curve data
          if (!lastSharpnessCurve.empty()) {
            try {
              // Create graph image with same width as resized image
              int graphHeight = 200;
              int graphWidth = colorResized.cols;
              cv::Mat graphImage =
                  cv::Mat::zeros(graphHeight, graphWidth, CV_8UC3);

              // The sharpness curve starts at kernel/2 and has (imageWidth -
              // kernel) points For reduced resolution: starts at 10, has (640/2
              // - 20) = 300 points
              int curveStartX =
                  10; // kernel/2 for reduced resolution (20/2 = 10)
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
              double amplitude = maxSharpness - minSharpness;

              // Scale based on sharpness curve range
              double maxVal = maxSharpness;
              if (maxVal <= 0)
                maxVal = 1.0;

              // Draw sharpness curve (blue) - map curve indices to correct x
              // positions
              for (size_t i = 1; i < lastSharpnessCurve.size(); i++) {
                int x1 = curveStartX + (i - 1);
                int x2 = curveStartX + i;

                // Only draw if within graph bounds
                if (x1 >= 0 && x2 < graphWidth) {
                  int y1 =
                      graphHeight -
                      static_cast<int>((lastSharpnessCurve[i - 1] / maxVal) *
                                       (graphHeight - 50));
                  int y2 = graphHeight -
                           static_cast<int>((lastSharpnessCurve[i] / maxVal) *
                                            (graphHeight - 50));
                  cv::line(graphImage, cv::Point(x1, y1), cv::Point(x2, y2),
                           cv::Scalar(255, 0, 0), 1); // Blue
                }
              }

              // Draw vertical line for center of mass on graph - adjust for
              // curve start position
              if (lastCenterOfMass >= 0) {
                int comX = curveStartX + static_cast<int>(lastCenterOfMass);
                if (comX >= 0 && comX < graphWidth) {
                  cv::line(graphImage, cv::Point(comX, 0),
                           cv::Point(comX, graphHeight),
                           cv::Scalar(255, 255, 0), 2);
                }
              }

              // Add legend text and amplitude
              cv::putText(graphImage, "Sharpness Curve", cv::Point(10, 20),
                          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0),
                          1);
              cv::putText(graphImage, "Center of Mass", cv::Point(10, 40),
                          cv::FONT_HERSHEY_SIMPLEX, 0.5,
                          cv::Scalar(255, 255, 0), 1);

              // Display amplitude and COM with double precision
              std::string amplitudeText =
                  "Amplitude: " + std::to_string(amplitude).substr(0, 6);
              std::string comText =
                  "COM: " + std::to_string(locBestFocusDouble).substr(0, 8);
              cv::putText(
                  graphImage, amplitudeText, cv::Point(10, graphHeight - 40),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
              cv::putText(graphImage, comText, cv::Point(10, graphHeight - 20),
                          cv::FONT_HERSHEY_SIMPLEX, 0.5,
                          cv::Scalar(0, 255, 255), 2);

              // Combine image and graph
              cv::vconcat(colorResized, graphImage, combined);

              // Save with integer filename
              std::string FilePath = "../output/TiltedCam_Images/" +
                                     std::to_string(imgcountfile - 1) + "_" +
                                     std::to_string(static_cast<int>(
                                         std::round(locBestFocusDouble))) +
                                     ".png";
              cv::imwrite(FilePath, combined);
            } catch (const std::exception &e) {
              // Just save the original image if we can't combine
              std::string FilePath = "../output/TiltedCam_Images/" +
                                     std::to_string(imgcountfile - 1) + "_" +
                                     std::to_string(static_cast<int>(
                                         std::round(locBestFocusDouble))) +
                                     ".png";
              cv::imwrite(FilePath, colorResized);
            }
          } else {
            // Just save the original image if no curve data
            std::string FilePath = "../output/TiltedCam_Images/" +
                                   std::to_string(imgcountfile - 1) + "_" +
                                   std::to_string(static_cast<int>(
                                       std::round(locBestFocusDouble))) +
                                   ".png";
            cv::imwrite(FilePath, colorResized);
          }
        };

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
        double pScaleFactor;
        if (errorMagnitude <= 3.0) {
          pScaleFactor = 0.13;
        } else if (errorMagnitude >= 130.0) {
          pScaleFactor = 1.0;
        } else {
          // Linear interpolation between 3 pixels (0.1x) and 100 pixels (1.0x)
          pScaleFactor =
              0.13 + (errorMagnitude - 3.0) * (1.0 - 0.13) / (100.0 - 3.0);
        }

        // Apply directional multiplier for downward moves (positive errors)
        double effectiveKp = Kp;
        if (currentError > 0) {
          // Moving toward more positive values (340→320, 320→300) - these were
          // slower
          effectiveKp *= 1.0; // 20% more aggressive for downward moves
        }

        double pSignal = effectiveKp * currentError * pScaleFactor;

        // Calculate derivative using filtered measurement
        double filteredCurrentError =
            desiredLocBestFocus - filteredLocBestFocus;

        // Calculate derivative using filtered error
        double rawDerivative =
            (filteredCurrentError - filteredPreviousError) / dt;
        double dSignal = Kd * rawDerivative;

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

int autofocus::computeBestFocus(cv::Mat image, int imgHeight, int imgWidth) {
  auto startTime = std::chrono::high_resolution_clock::now();

  // CLAHE preprocessing with reduced noise amplification
  auto claheStart = std::chrono::high_resolution_clock::now();
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(2.0); // Reduced from 2.0 to limit noise amplification
  clahe->setTilesGridSize(cv::Size(
      8, 8)); // Larger tiles (16x16) for less aggressive local adaptation
  cv::Mat clahe_enhanced;
  clahe->apply(image, clahe_enhanced);
  auto claheEnd = std::chrono::high_resolution_clock::now();

  // Gaussian Blur: ~300μs (5.5%)
  auto blurStart = std::chrono::high_resolution_clock::now();
  cv::GaussianBlur(clahe_enhanced, blurred_preallocated, cv::Size(3, 3), 1, 1,
                   cv::BORDER_DEFAULT);
  auto blurEnd = std::chrono::high_resolution_clock::now();

  // Tenengrad method: ~1,400μs (25.5%), occasionally spikes to ~4,000μs
  auto tenengradStart = std::chrono::high_resolution_clock::now();

  // Compute gradients using Sobel operators
  cv::Sobel(blurred_preallocated, img_x_preallocated, CV_16S, 1, 0,
            3); // 3x3 Sobel for X direction
  cv::Sobel(blurred_preallocated, img_y_preallocated, CV_16S, 0, 1,
            3); // 3x3 Sobel for Y direction

  // Square the gradients and sum them (Tenengrad formula: Gx² + Gy²)
  cv::multiply(img_x_preallocated, img_x_preallocated,
               img_x_squared_preallocated);
  cv::multiply(img_y_preallocated, img_y_preallocated,
               img_y_squared_preallocated);
  cv::add(img_x_squared_preallocated, img_y_squared_preallocated,
          sum_xy_preallocated);

  // Convert to float for better precision in subsequent calculations
  sum_xy_preallocated.convertTo(sharpness_float_preallocated, CV_32F);
  auto tenengradEnd = std::chrono::high_resolution_clock::now();

  // Column Means: ~2,400μs (43.6%), occasionally spikes to ~6,000μs
  auto columnStart = std::chrono::high_resolution_clock::now();
  cv::Mat columnMeansMatrix;
  cv::reduce(sharpness_float_preallocated, columnMeansMatrix, 0, cv::REDUCE_AVG,
             CV_64F);

  // Convert to vector for compatibility with existing code
  std::vector<double> columnMeans;
  columnMeansMatrix.copyTo(columnMeans);

  auto columnEnd = std::chrono::high_resolution_clock::now();

  // Sliding Window with Hamming: ~8μs (0.1%)
  auto slidingStart = std::chrono::high_resolution_clock::now();
  std::vector<double> sharpnesscurve;
  int kernel = 40; // must be an even number
  // 32*2*2*2*2 = 512 pixels
  // Pre-compute Hamming window coefficients
  std::vector<double> hammingWindow(kernel);
  for (int i = 0; i < kernel; i++) {
    hammingWindow[i] = 0.54 - 0.46 * std::cos(2 * M_PI * i / (kernel - 1.0));
  }

  for (int i = 0; i < imgWidth - kernel; i++) {
    double regionSharpnessScore = 0.0;
    double windowSum = 0.0;
    for (int k = 0; k < kernel; k++) {
      regionSharpnessScore += columnMeans[i + k] * hammingWindow[k];
      windowSum += hammingWindow[k];
    }
    regionSharpnessScore /=
        windowSum; // Normalize by sum of window coefficients
    sharpnesscurve.push_back(regionSharpnessScore);
  }
  auto slidingEnd = std::chrono::high_resolution_clock::now();

  // Calculate and save the max amplitude of the sharpness curve
  double maxVal =
      *std::max_element(sharpnesscurve.begin(), sharpnesscurve.end());
  double minVal =
      *std::min_element(sharpnesscurve.begin(), sharpnesscurve.end());
  double amplitude = maxVal - minVal;
  double offset = minVal;
  // If the amplitude is too low, return the desiredLocBestFocus so no lens
  // movement occurs. TODO: add error message
  if (amplitude < 0.3) {
    std::cout << "Amplitude is too low, returning desiredLocBestFocus\n";
    return desiredLocBestFocus;
  }

  // Fitting a normal curve to the sharpness curve, to avoid local peaks in the
  // data around vessel edges (and deal with multiple real peaks) Curve Fitting:
  // ~1,400μs (25.5%), occasionally spikes to ~4,000μs
  auto fittingStart = std::chrono::high_resolution_clock::now();
  //  std::vector<double> sharpnesscurvenormalized =
  //  fitnormalcurve(sharpnesscurve, amplitude, offset, 0.2);
  std::vector<double> sharpnesscurvenormalized =
      fitnormalcurveBruteForce(sharpnesscurve, amplitude, offset, 0.2);
  auto fittingEnd = std::chrono::high_resolution_clock::now();

  // printing to text files for testing
  if (bSaveSharpnessCurves) {
    std::string FileName = "TESTING" + std::to_string(increment);
    std::string TextFile =
        "/home/hvi/Desktop/HVI-data/Blendi_SharpnessCurves/" + FileName +
        "_SharpnessCurve.txt";
    std::string TextFile2 =
        "/home/hvi/Desktop/HVI-data/Blendi_SharpnessCurves/" + FileName +
        "_FittedNorm.txt";
    std::ofstream outputFile(TextFile);
    std::ofstream outputFile2(TextFile2);
    std::ostream_iterator<double> output_iterator(outputFile, ", ");
    std::copy(sharpnesscurve.begin(), sharpnesscurve.end(), output_iterator);
    outputFile << "\n";
    std::ostream_iterator<double> output_iterator2(outputFile2, ", ");
    std::copy(sharpnesscurvenormalized.begin(), sharpnesscurvenormalized.end(),
              output_iterator2);
    outputFile2 << "\n";
    outputFile.close();
    outputFile2.close();
    increment++;
  }

  int locBestFocus = distance(begin(sharpnesscurvenormalized),
                              max_element(begin(sharpnesscurvenormalized),
                                          end(sharpnesscurvenormalized)));

  if (bSaveImages) {
    // Store the curves for visualization
    lastSharpnessCurve = sharpnesscurve;
    lastFittedCurve = sharpnesscurvenormalized;
  }

  auto endTime = std::chrono::high_resolution_clock::now();

  // Calculate timing benchmarks
  auto claheTime = std::chrono::duration_cast<std::chrono::microseconds>(
      claheEnd - claheStart);
  auto blurTime = std::chrono::duration_cast<std::chrono::microseconds>(
      blurEnd - blurStart);
  auto tenengradTime = std::chrono::duration_cast<std::chrono::microseconds>(
      tenengradEnd - tenengradStart);
  auto columnTime = std::chrono::duration_cast<std::chrono::microseconds>(
      columnEnd - columnStart);
  auto slidingTime = std::chrono::duration_cast<std::chrono::microseconds>(
      slidingEnd - slidingStart);
  auto fittingTime = std::chrono::duration_cast<std::chrono::microseconds>(
      fittingEnd - fittingStart);
  auto totalTime = std::chrono::duration_cast<std::chrono::microseconds>(
      endTime - startTime);

  // Save timing information to CSV file with better error handling
  try {
    std::ofstream benchmarkFile("../output/focus_benchmark.csv", std::ios::app);
    if (benchmarkFile.is_open() && benchmarkFile.good()) {
      time_t now = time(0);
      benchmarkFile << now << "," << totalTime.count() << ","
                    << claheTime.count() << "," << blurTime.count() << ","
                    << tenengradTime.count() << "," << columnTime.count() << ","
                    << slidingTime.count() << "," << fittingTime.count()
                    << std::endl;

      if (benchmarkFile.fail()) {
        if (bAutofocusLogFlag) {
          logger->warn(
              "[autofocus::computeBestFocus] Failed to write benchmark data");
        }
      }
      benchmarkFile.close();
    }
  } catch (const std::exception &ex) {
    if (bAutofocusLogFlag) {
      logger->error("[autofocus::computeBestFocus] Benchmark file error: " +
                    std::string(ex.what()));
    }
  }

  return locBestFocus +
         kernel / 2; // Note: kernel/2 offset needed for sliding window
}

double autofocus::computeBestFocusReduced(cv::Mat image, int imgHeight,
                                          int imgWidth) {
  auto startTime = std::chrono::high_resolution_clock::now();

  // Resize to 1/2 x 1/2
  auto resizeStart = std::chrono::high_resolution_clock::now();
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), 0.5, 0.5);
  auto resizeEnd = std::chrono::high_resolution_clock::now();

  // CLAHE
  auto claheStart = std::chrono::high_resolution_clock::now();
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(2.0);
  clahe->setTilesGridSize(cv::Size(4, 4));
  cv::Mat clahe_enhanced;
  clahe->apply(resized, clahe_enhanced);
  auto claheEnd = std::chrono::high_resolution_clock::now();

  // Gaussian Blur
  auto blurStart = std::chrono::high_resolution_clock::now();
  cv::Mat blurred;
  cv::GaussianBlur(clahe_enhanced, blurred, cv::Size(3, 3), 1, 1,
                   cv::BORDER_DEFAULT);

  // try without CLAHE
  // cv::GaussianBlur(resized, blurred, cv::Size(7, 7), 1, 1,
  //                 cv::BORDER_DEFAULT);
  auto blurEnd = std::chrono::high_resolution_clock::now();

  // Roberts Cross gradients -> sharpness image (float)
  auto robertsStart = std::chrono::high_resolution_clock::now();
  cv::Mat img_x, img_y;
  cv::filter2D(blurred, img_x, CV_16S, roberts_kernelx);
  cv::filter2D(blurred, img_y, CV_16S, roberts_kernely);
  cv::Mat img_x_squared, img_y_squared, sum_xy;
  cv::multiply(img_x, img_x, img_x_squared);
  cv::multiply(img_y, img_y, img_y_squared);
  cv::add(img_x_squared, img_y_squared, sum_xy);
  cv::Mat sharpness_float;
  sum_xy.convertTo(sharpness_float, CV_32F);
  auto robertsEnd = std::chrono::high_resolution_clock::now();

  // Column means
  auto columnStart = std::chrono::high_resolution_clock::now();
  cv::Mat columnMeansMatrix;
  cv::reduce(sharpness_float, columnMeansMatrix, 0, cv::REDUCE_AVG, CV_64F);
  std::vector<double> columnMeans;
  columnMeansMatrix.copyTo(columnMeans);
  auto columnEnd = std::chrono::high_resolution_clock::now();

  // Sliding Hamming
  auto slidingStart = std::chrono::high_resolution_clock::now();
  std::vector<double> sharpnesscurve;
  const int kernel = 20; // for 1/2 resolution
  std::vector<double> hammingWindow(kernel);
  for (int i = 0; i < kernel; ++i)
    hammingWindow[i] = 0.54 - 0.46 * std::cos(2 * M_PI * i / (kernel - 1.0));

  sharpnesscurve.reserve(std::max(0, blurred.cols - kernel));
  for (int i = 0; i < blurred.cols - kernel; ++i) {
    double acc = 0.0, wsum = 0.0;
    for (int k = 0; k < kernel; ++k) {
      const double w = hammingWindow[k];
      acc += columnMeans[i + k] * w;
      wsum += w;
    }
    sharpnesscurve.push_back(acc / wsum);
  }

  // Use moving average instead of Hamming window
  // for (int i = 0; i < blurred.cols - kernel; i++) {
  //   double regionSharpnessScore = 0.0;
  //   for (int k = 0; k < kernel; k++) {
  //     regionSharpnessScore += columnMeans[i + k];
  //   }
  //   regionSharpnessScore /= kernel; // Normalize by kernel size
  //   sharpnesscurve.push_back(regionSharpnessScore);
  // }

  auto slidingEnd = std::chrono::high_resolution_clock::now();


  // Save range of values after each stage for debugging
  if (bSaveSharpnessCurves) {
    std::string FileName = "TESTING_" + std::to_string(increment);
    std::string TextFile1 = "../output/SharpnessCurves_steps/" + FileName + "_CLAHE_1.txt";
    std::string TextFile2 = "../output/SharpnessCurves_steps/" + FileName + "_blur_2.txt";
    std::string TextFile3 = "../output/SharpnessCurves_steps/" + FileName + "_robertscross_3.txt";
    std::string TextFile4 = "../output/SharpnessCurves_steps/" + FileName + "_column_means_4.txt";
    std::string TextFile5 = "../output/SharpnessCurves_steps/" + FileName + "_hamming_5.txt";
    std::ofstream outputFile1(TextFile1);
    std::ofstream outputFile2(TextFile2);
    std::ofstream outputFile3(TextFile3);
    std::ofstream outputFile4(TextFile4);
    std::ofstream outputFile5(TextFile5);

    std::ostream_iterator<double> output_iterator(outputFile1, ", ");
    for (int i = 0; i < clahe_enhanced.rows; ++i) {
      const uchar* row_ptr = clahe_enhanced.ptr<uchar>(i);
      for (int j = 0; j < clahe_enhanced.cols; ++j) {
        outputFile1 << static_cast<double>(row_ptr[j]) << ", ";
      }
    }
    outputFile1 << "\n";
    std::ostream_iterator<double> output_iterator2(outputFile2, ", ");
    for (int i = 0; i < blurred.rows; ++i) {
      const uchar* row_ptr = blurred.ptr<uchar>(i);
      for (int j = 0; j < blurred.cols; ++j) {
        outputFile2 << static_cast<double>(row_ptr[j]) << ", ";
      }
    }
    outputFile2 << "\n";
    std::ostream_iterator<double> output_iterator3(outputFile3, ", ");
    for (int i = 0; i < sharpness_float.rows; ++i) {
        const float* row_ptr = sharpness_float.ptr<float>(i);
        for (int j = 0; j < sharpness_float.cols; ++j) {
            outputFile3 << static_cast<double>(row_ptr[j]) << ", ";
        }
    }
    outputFile3 << "\n";
    std::ostream_iterator<double> output_iterator4(outputFile4, ", ");
    std::copy(columnMeans.begin(), columnMeans.end(), output_iterator4);
    outputFile4 << "\n";
    std::ostream_iterator<double> output_iterator5(outputFile5, ", ");
    std::copy(sharpnesscurve.begin(), sharpnesscurve.end(), output_iterator5);
    outputFile5 << "\n";

    outputFile1.close();
    outputFile2.close();
    outputFile3.close();
    outputFile4.close();
    outputFile5.close();
    increment++;
  }


  // Normalize baseline to zero for robust COM / TG moments
  if (!sharpnesscurve.empty()) {
    const double minVal =
        *std::min_element(sharpnesscurve.begin(), sharpnesscurve.end());
    for (double &v : sharpnesscurve)
      v = std::max(0.0, v - minVal);
  }

  // Amplitude check (edge case / low SNR): behave like your current shortcut.
  const double maxVal =
      sharpnesscurve.empty()
          ? 0.0
          : *std::max_element(sharpnesscurve.begin(), sharpnesscurve.end());
  if (maxVal < 3.0) {
    // keep behavior: do nothing when almost no structure is visible
    return static_cast<double>(desiredLocBestFocus);
  }

  // --- Choose estimator on the curve domain [0..N-1] ---
  auto estStart = std::chrono::high_resolution_clock::now();

  double mu_curve = 0.0; // result in curve index units
  lastFittedCurve.clear();
  lastSharpnessCurve = sharpnesscurve;
  lastCenterOfMass = -1.0; // we'll fill it if available

  switch (m_peakLocator) {
  case PeakLocator::CenterOfMass: {
    // your plain COM (already baseline-subtracted)
    mu_curve = findCenterOfMass(sharpnesscurve);
    lastCenterOfMass = mu_curve;
  } break;

  case PeakLocator::PowerCOM: {
    double com_plain = -1.0;
    mu_curve = estimatePeakPowerCOM(sharpnesscurve, m_powerExponent,
                                    m_powerCOMQuadRefine, &com_plain);
    // Keep overlay meaning: cyan line = plain COM
    if (com_plain >= 0.0)
      lastCenterOfMass = com_plain;
  } break;

  case PeakLocator::TruncatedGaussian: {
    // Work in reduced coordinates. If user set σ at full-res, scale it by 0.5
    // If not set, use the same estimation as the brute force method
    double sigmaReduced;
    if (m_sigmaPxFullRes > 0.0) {
      sigmaReduced = 0.5 * m_sigmaPxFullRes;
    } else {
      // Use the same σ estimation as brute force Gaussian: 0.2 * curve_length
      sigmaReduced = 0.2 * sharpnesscurve.size();
    }
    double mu_a_dbg = -1.0;
    mu_curve = estimatePeakTruncatedGaussian(sharpnesscurve, sigmaReduced,
                                             m_edgeAvgCount, 1e-2, &mu_a_dbg,
                                             nullptr, nullptr);
    if (mu_a_dbg >= 0.0)
      lastCenterOfMass = mu_a_dbg; // overlay: COM
  } break;

  case PeakLocator::GaussianFitBrute: {
    const double amplitude = maxVal;
    const double offset = 0.0;
    std::vector<double> fitted =
        fitnormalcurveBruteForce(sharpnesscurve, amplitude, offset, 0.2);
    auto it = std::max_element(fitted.begin(), fitted.end());
    mu_curve = static_cast<double>(std::distance(fitted.begin(), it));
   // (optional) lastCenterOfMass = findCenterOfMass(sharpnesscurve);
  } break;
  }

  // Optional exponential smoothing of the peak (keeps continuity but still
  // fast)
  if (m_peakEmaBeta < 1.0) {
    if (std::isnan(m_prevMuReduced))
      m_prevMuReduced = mu_curve;
    mu_curve =
        (1.0 - m_peakEmaBeta) * m_prevMuReduced + m_peakEmaBeta * mu_curve;
    m_prevMuReduced = mu_curve;
  }

  // For your saved debug overlay: show COM (μ_a) if we have it
  // (mu_a_dbg is only available in TruncatedGaussian case, handled in switch
  // above)

  auto estEnd = std::chrono::high_resolution_clock::now();

  // --- Timing log unchanged footprint, writing "com_time_us" as estimator time
  // ---
  auto totalTime =
      std::chrono::duration_cast<std::chrono::microseconds>(estEnd - startTime);
  auto resizeTime = std::chrono::duration_cast<std::chrono::microseconds>(
      resizeEnd - resizeStart);
  auto claheTime = std::chrono::duration_cast<std::chrono::microseconds>(
      claheEnd - claheStart);
  auto blurTime = std::chrono::duration_cast<std::chrono::microseconds>(
      blurEnd - blurStart);
  auto robertsTime = std::chrono::duration_cast<std::chrono::microseconds>(
      robertsEnd - robertsStart);
  auto columnTime = std::chrono::duration_cast<std::chrono::microseconds>(
      columnEnd - columnStart);
  auto slidingTime = std::chrono::duration_cast<std::chrono::microseconds>(
      slidingEnd - slidingStart);
  auto estTime =
      std::chrono::duration_cast<std::chrono::microseconds>(estEnd - estStart);

  try {
    std::ofstream reducedBenchmarkFile("../output/focus_benchmark_reduced.csv",
                                       std::ios::app);
    if (reducedBenchmarkFile.is_open() && reducedBenchmarkFile.good()) {
      auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
      // keep CSV header compatibility: write estimator time in the
      // "com_time_us" column
      reducedBenchmarkFile << timestamp << "," << totalTime.count() << ","
                           << resizeTime.count() << "," << claheTime.count()
                           << "," << blurTime.count() << ","
                           << robertsTime.count() << "," << columnTime.count()
                           << "," << slidingTime.count() << ","
                           << estTime.count() << std::endl;
      reducedBenchmarkFile.close();
    }
  } catch (...) { /* ignore */
  }

  // Map curve index back to reduced-image x by adding the kernel/2 offset,
  // exactly like your original COM method (no scaling to full-res here).
  return mu_curve + kernel / 2.0;
}

int autofocus::computeBestFocusVeryReduced(cv::Mat image, int imgHeight,
                                           int imgWidth) {
  auto startTime = std::chrono::high_resolution_clock::now();

  // Resize to 1/16 size (1/4 in each dimension)
  auto resizeStart = std::chrono::high_resolution_clock::now();
  cv::Mat resized_image;
  cv::resize(image, resized_image, cv::Size(imgWidth / 4, imgHeight / 4), 0, 0,
             cv::INTER_LINEAR);
  int reducedWidth = resized_image.cols;
  int reducedHeight = resized_image.rows;
  auto resizeEnd = std::chrono::high_resolution_clock::now();

  // CLAHE preprocessing with reduced noise amplification
  auto claheStart = std::chrono::high_resolution_clock::now();
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(2.0);
  clahe->setTilesGridSize(
      cv::Size(4, 4)); // Increased from 2x2 to 4x4 to avoid artifacts
  cv::Mat clahe_enhanced;
  clahe->apply(resized_image, clahe_enhanced);
  auto claheEnd = std::chrono::high_resolution_clock::now();

  // Gaussian Blur
  auto blurStart = std::chrono::high_resolution_clock::now();
  cv::Mat blurred_very_reduced;
  cv::GaussianBlur(clahe_enhanced, blurred_very_reduced, cv::Size(3, 3), 1, 1,
                   cv::BORDER_DEFAULT);
  auto blurEnd = std::chrono::high_resolution_clock::now();

  // Roberts Cross method instead of Tenengrad
  auto robertsStart = std::chrono::high_resolution_clock::now();

  // Apply Roberts Cross kernels
  cv::Mat img_x_roberts, img_y_roberts;
  cv::filter2D(blurred_very_reduced, img_x_roberts, CV_16S, roberts_kernelx);
  cv::filter2D(blurred_very_reduced, img_y_roberts, CV_16S, roberts_kernely);

  // Square the gradients and sum them (Roberts Cross formula: Gx² + Gy²)
  cv::Mat img_x_squared_roberts, img_y_squared_roberts, sum_xy_roberts;
  cv::multiply(img_x_roberts, img_x_roberts, img_x_squared_roberts);
  cv::multiply(img_y_roberts, img_y_roberts, img_y_squared_roberts);
  cv::add(img_x_squared_roberts, img_y_squared_roberts, sum_xy_roberts);

  // Convert to float for better precision in subsequent calculations
  cv::Mat sharpness_float_roberts;
  sum_xy_roberts.convertTo(sharpness_float_roberts, CV_32F);
  auto robertsEnd = std::chrono::high_resolution_clock::now();

  // Column Means
  auto columnStart = std::chrono::high_resolution_clock::now();
  cv::Mat columnMeansMatrix;
  cv::reduce(sharpness_float_roberts, columnMeansMatrix, 0, cv::REDUCE_AVG,
             CV_64F);

  // Convert to vector for compatibility with existing code
  std::vector<double> columnMeans;
  columnMeansMatrix.copyTo(columnMeans);
  auto columnEnd = std::chrono::high_resolution_clock::now();

  // Sliding Window with Hamming - scale kernel size for very reduced resolution
  auto slidingStart = std::chrono::high_resolution_clock::now();
  std::vector<double> sharpnesscurve;
  int kernel = 16; // Reduced from 40 to 10 for 1/4 resolution (40/4 = 10)

  // Pre-compute Hamming window coefficients
  std::vector<double> hammingWindow(kernel);
  for (int i = 0; i < kernel; i++) {
    hammingWindow[i] = 0.54 - 0.46 * std::cos(2 * M_PI * i / (kernel - 1.0));
  }

  for (int i = 0; i < reducedWidth - kernel; i++) {
    double regionSharpnessScore = 0.0;
    double windowSum = 0.0;
    for (int k = 0; k < kernel; k++) {
      regionSharpnessScore += columnMeans[i + k] * hammingWindow[k];
      windowSum += hammingWindow[k];
    }
    regionSharpnessScore /=
        windowSum; // Normalize by sum of window coefficients
    sharpnesscurve.push_back(regionSharpnessScore);
  }
  auto slidingEnd = std::chrono::high_resolution_clock::now();

  // Calculate and save the max amplitude of the sharpness curve
  double maxVal =
      *std::max_element(sharpnesscurve.begin(), sharpnesscurve.end());
  double minVal =
      *std::min_element(sharpnesscurve.begin(), sharpnesscurve.end());
  double amplitude = maxVal - minVal;
  double offset = minVal;

  // If the amplitude is too low, return the scaled desiredLocBestFocus
  if (amplitude < 0.3) {
    std::cout << "Amplitude is too low, returning scaled desiredLocBestFocus\n";
    return desiredLocBestFocus / 4; // Scale down for very reduced resolution
  }

  // Fitting a normal curve - scale std_dev_factor for very reduced resolution
  auto fittingStart = std::chrono::high_resolution_clock::now();
  // std::vector<double> sharpnesscurvenormalized =
  // fitnormalcurveBruteForce(sharpnesscurve, amplitude, offset, 0.2); // 0.2 *
  // 4 = 0.8
  std::vector<double> sharpnesscurvenormalized = fitnormalcurveBruteForce(
      sharpnesscurve, amplitude, offset, 0.2); // 0.2 * 4 = 0.8

  auto fittingEnd = std::chrono::high_resolution_clock::now();

  // Save sharpness curves if enabled
  if (bSaveSharpnessCurves) {
    std::string FileName = "TESTING_VERY_REDUCED" + std::to_string(increment);
    std::string TextFile =
        "/home/hvi/Desktop/HVI-data/Blendi_SharpnessCurves/" + FileName +
        "_SharpnessCurve.txt";
    std::string TextFile2 =
        "/home/hvi/Desktop/HVI-data/Blendi_SharpnessCurves/" + FileName +
        "_FittedNorm.txt";
    std::ofstream outputFile(TextFile);
    std::ofstream outputFile2(TextFile2);
    std::ostream_iterator<double> output_iterator(outputFile, ", ");
    std::copy(sharpnesscurve.begin(), sharpnesscurve.end(), output_iterator);
    outputFile << "\n";
    std::ostream_iterator<double> output_iterator2(outputFile2, ", ");
    std::copy(sharpnesscurvenormalized.begin(), sharpnesscurvenormalized.end(),
              output_iterator2);
    outputFile2 << "\n";
    outputFile.close();
    outputFile2.close();
  }

  int locBestFocus = distance(begin(sharpnesscurvenormalized),
                              max_element(begin(sharpnesscurvenormalized),
                                          end(sharpnesscurvenormalized)));

  if (bSaveImages) {
    // Scale the curves for visualization to match full resolution length
    // Original full resolution curve would be approximately (imgWidth - 40)
    // points Very reduced curve is approximately (imgWidth/4 - 10) points Scale
    // factor is roughly 4x

    int targetLength = imgWidth - 40; // Target length to match full resolution
    std::vector<double> scaledSharpnessCurve;
    std::vector<double> scaledFittedCurve;

    // Simple linear interpolation to scale up the curves
    for (int i = 0; i < targetLength; i++) {
      double sourceIndex =
          (double)i * (sharpnesscurve.size() - 1) / (targetLength - 1);
      int lowerIndex = (int)sourceIndex;
      int upperIndex = std::min(lowerIndex + 1, (int)sharpnesscurve.size() - 1);
      double fraction = sourceIndex - lowerIndex;

      // Interpolate sharpness curve
      double interpolatedSharpness =
          sharpnesscurve[lowerIndex] * (1.0 - fraction) +
          sharpnesscurve[upperIndex] * fraction;
      scaledSharpnessCurve.push_back(interpolatedSharpness);

      // Interpolate fitted curve
      double interpolatedFitted =
          sharpnesscurvenormalized[lowerIndex] * (1.0 - fraction) +
          sharpnesscurvenormalized[upperIndex] * fraction;
      scaledFittedCurve.push_back(interpolatedFitted);
    }

    // Store the scaled curves for visualization
    lastSharpnessCurve = scaledSharpnessCurve;
    lastFittedCurve = scaledFittedCurve;
  }

  auto endTime = std::chrono::high_resolution_clock::now();

  // Calculate timing benchmarks
  auto resizeTime = std::chrono::duration_cast<std::chrono::microseconds>(
      resizeEnd - resizeStart);
  auto claheTime = std::chrono::duration_cast<std::chrono::microseconds>(
      claheEnd - claheStart);
  auto blurTime = std::chrono::duration_cast<std::chrono::microseconds>(
      blurEnd - blurStart);
  auto robertsTime = std::chrono::duration_cast<std::chrono::microseconds>(
      robertsEnd - robertsStart);
  auto columnTime = std::chrono::duration_cast<std::chrono::microseconds>(
      columnEnd - columnStart);
  auto slidingTime = std::chrono::duration_cast<std::chrono::microseconds>(
      slidingEnd - slidingStart);
  auto fittingTime = std::chrono::duration_cast<std::chrono::microseconds>(
      fittingEnd - fittingStart);
  auto totalTime = std::chrono::duration_cast<std::chrono::microseconds>(
      endTime - startTime);

  // Save timing information to CSV file with different filename
  std::ofstream benchmarkFile("../output/focus_benchmark_very_reduced.csv",
                              std::ios::app);
  if (benchmarkFile.is_open()) {
    time_t now = time(0);
    benchmarkFile << now << "," << totalTime.count() << ","
                  << resizeTime.count() << "," << claheTime.count() << ","
                  << blurTime.count() << "," << robertsTime.count() << ","
                  << columnTime.count() << "," << slidingTime.count() << ","
                  << fittingTime.count() << std::endl;
    benchmarkFile.close();
  }

  // Scale the result back to full resolution and add kernel offset
  return (locBestFocus + kernel / 2) * 4; // Scale up by 4x for full resolution
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
}

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
  size_t i;

  for (i = 0; i < d->n; i++) {
    double t = d->t[i];
    double yi = d->y[i];
    double y = gaussian(a,b,c,ti)
    gsl_vector_set (f, i, yi-y);
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
  double va = gsl_vector_get(v, 0);
  double vb = gsl_vector_get(v, 1);
  double vc = gsl_vector_get(v, 2);
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

  fprintf(stderr, "iter %2zu: a = %.4f, b = %.4f, c = %.4f, |a|/|v| = %.4f cond(J) = %8.4f, |f(x)| = %.4f\n",
          iter,
          gsl_vector_get(x, 0),
          gsl_vector_get(x, 1),
          gsl_vector_get(x, 2),
          avratio,
          1.0 / rcond,
          gsl_blas_dnrm2(f));
}

void
solve_system(gsl_vector *x, gsl_multifit_nlinear_fdf *fdf,
             gsl_multifit_nlinear_parameters *params)
{
  const gsl_multifit_nlinear_type *T = gsl_multifit_nlinear_trust;
  const size_t max_iter = 200;
  const double xtol = 1.0e-8;
  const double gtol = 1.0e-8;
  const double ftol = 1.0e-8;
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

  fprintf(stderr, "NITER         = %zu\n", gsl_multifit_nlinear_niter(work));
  fprintf(stderr, "NFEV          = %zu\n", fdf->nevalf);
  fprintf(stderr, "NJEV          = %zu\n", fdf->nevaldf);
  fprintf(stderr, "NAEV          = %zu\n", fdf->nevalfvv);
  fprintf(stderr, "initial cost  = %.12e\n", chisq0);
  fprintf(stderr, "final cost    = %.12e\n", chisq);
  fprintf(stderr, "final x       = (%.12e, %.12e, %12e)\n",
          gsl_vector_get(x, 0), gsl_vector_get(x, 1), gsl_vector_get(x, 2));
  fprintf(stderr, "final cond(J) = %.12e\n", 1.0 / rcond);

  gsl_multifit_nlinear_free(work);
}

double
autofocus::computeBestFocusGSL(v::Mat image, int imgHeight,
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
  cv::filter2D(resized, img_x, CV_16S, roberts_kernelx);
  cv::filter2D(resized, img_y, CV_16S, roberts_kernely);
  cv::Mat img_x_squared, img_y_squared, sum_xy;
  cv::multiply(img_x, img_x, img_x_squared);
  cv::multiply(img_y, img_y, img_y_squared);
  cv::add(img_x_squared, img_y_squared, sum_xy);
  cv::Mat sharpness_float;
  sum_xy.convertTo(sharpness_float, CV_32F);
  auto robertsEnd = std::chrono::high_resolution_clock::now();

  // Column means
  auto columnStart = std::chrono::high_resolution_clock::now();
  cv::Mat columnMeansMatrix;
  cv::reduce(sharpness_float, columnMeansMatrix, 0, cv::REDUCE_AVG, CV_64F);
  std::vector<double> columnMeans;
  columnMeansMatrix.copyTo(columnMeans);
  auto columnEnd = std::chrono::high_resolution_clock::now();
}

std::vector<double>
autofocus::fitnormalcurveBruteForce(std::vector<double> sharpnesscurve,
                                    double amplitude, double offset,
                                    double std_dev_factor) {
  // Fits a normal curve to the data. Should try to find a proper curve-fitting
  // library...
  std::vector<double> norm_curve;
  std::vector<double> sum_of_diffs;
  double std_dev =
      std_dev_factor * (sharpnesscurve.size()); // now uses the parameter

  // Calculates sum_of_elems, a vector showing how well each normal curve with
  // mean j fits grad
  for (int j = 0; j < sharpnesscurve.size(); j++) // Start from 0, not 1
  {
    // Generates normal curve for given mean j
    std::vector<double> norm_curve_temp;
    for (int i = 0; i < sharpnesscurve.size(); i++) {
      double normResult =
          normpdf(i, j, std_dev); // j is now the actual mean position
      norm_curve_temp.push_back(normResult);
    }
    // scales so the max value (when i=j) is equal to the desired amplitude
    double factor =
        1.0 / *max_element(begin(norm_curve_temp), end(norm_curve_temp));
    for (int i = 0; i < sharpnesscurve.size(); i++) {
      norm_curve_temp[i] = norm_curve_temp[i] * factor * amplitude + offset;
    }

    // Calculates difference between norm curve and sharpness curve
    std::vector<double> differences;
    for (int z = 0; z < sharpnesscurve.size(); z++) {
      double diff = abs(sharpnesscurve[z] - norm_curve_temp[z]);
      differences.push_back(diff);
    }
    double sum_of_diff = std::accumulate(differences.begin(), differences.end(),
                                         decltype(differences)::value_type(0));
    sum_of_diffs.push_back(sum_of_diff);
  }

  // Picks the mean that has the lowest overall difference
  int bestMeanIndex = distance(
      begin(sum_of_diffs), min_element(begin(sum_of_diffs), end(sum_of_diffs)));
  double mean = bestMeanIndex; // No +1 needed since j starts from 0

  // Generates final norm_curve using the correct mean
  for (int i = 0; i < sharpnesscurve.size(); i++) {
    double normResult = normpdf(i, mean, std_dev);
    norm_curve.push_back(normResult);
  }
  double factor = 1.0 / *max_element(begin(norm_curve), end(norm_curve));
  for (int i = 0; i < (sharpnesscurve.size()); i++) {
    norm_curve[i] = norm_curve[i] * factor * amplitude + offset;
  }

  std::vector<double> fittednormalcurve(norm_curve.begin(), norm_curve.end());
  return fittednormalcurve;
}

double autofocus::findCenterOfMass(const std::vector<double> &curve) {
  double weightedSum = 0.0;
  double totalWeight = 0.0;

  for (int i = 0; i < curve.size(); i++) {
    weightedSum += i * curve[i];
    totalWeight += curve[i];
  }

  return totalWeight > 0 ? (weightedSum / totalWeight) : curve.size() / 2.0;
}

void autofocus::setPeakLocator(PeakLocator m) { m_peakLocator = m; }

void autofocus::setTruncGaussSigmaFullRes(double sigmaPx) {
  m_sigmaPxFullRes = sigmaPx;
}

void autofocus::setPowerCOMExponent(int p) { m_powerExponent = std::max(1, p); }

void autofocus::setPowerCOMQuadraticRefine(bool on) {
  m_powerCOMQuadRefine = on;
}

void autofocus::setTruncGaussEdgeAveraging(int count) {
  m_edgeAvgCount = std::max(1, count);
}

void autofocus::setPeakSmoothing(double beta) {
  // clamp to (0,1]; beta=1 means "no smoothing"
  m_peakEmaBeta = std::min(std::max(beta, 0.0), 1.0);
  if (m_peakEmaBeta >= 1.0) {
    m_prevMuReduced = std::numeric_limits<double>::quiet_NaN();
  }
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

double autofocus::estimatePeakPowerCOM(const std::vector<double> &v, int power,
                                       bool quadraticRefine,
                                       double *out_plainCOM) {
  const int N = static_cast<int>(v.size());
  if (N <= 0) {
    if (out_plainCOM)
      *out_plainCOM = 0.0;
    return 0.0;
  }

  // Baseline subtract to suppress center pull from background
  const double minv = *std::min_element(v.begin(), v.end());

  // Optional: also compute plain COM for overlay/debug
  if (out_plainCOM) {
    double S0 = 0.0, S1 = 0.0;
    for (int i = 0; i < N; ++i) {
      const double w = std::max(0.0, v[i] - minv);
      S0 += w;
      S1 += w * i;
    }
    *out_plainCOM = (S0 > 1e-12) ? (S1 / S0) : 0.5 * (N - 1);
  }

  double sumw = 0.0, sumiw = 0.0;
  for (int i = 0; i < N; ++i) {
    double w = v[i] - minv;
    if (w <= 0.0)
      continue;
    double wp = powi_pos(w, std::max(1, power)); // winner-take-most
    sumw += wp;
    sumiw += wp * i;
  }

  if (sumw <= 1e-12)
    return 0.5 * (N - 1); // degenerate fallback

  double idx = sumiw / sumw;

  // Optional sub-pixel parabolic refine around nearest integer bin
  if (quadraticRefine) {
    int k = std::clamp((int)std::round(idx), 0, N - 1);
    if (k > 0 && k < N - 1) {
      double y1 = v[k - 1], y2 = v[k], y3 = v[k + 1];
      double denom = 2.0 * y2 - y1 - y3;
      if (std::fabs(denom) > 1e-12) {
        double delta = 0.5 * (y1 - y3) / denom; // ~[-0.5, +0.5]
        idx = std::clamp(k + delta, 0.0, (double)(N - 1));
      }
    }
  }
  return idx;
}

// Discrete truncated-Gaussian correction on f[0..N-1] (non-negative).
// sigmaPxReduced: Gaussian sigma in the same pixel units as f's x-axis (reduced
// resolution). If sigmaPxReduced <= 0, we auto-estimate σ from apparent
// variance σ_a^2. eps: threshold for the |t| test (per note in the memo);
// default ~1e-2.
double autofocus::estimatePeakTruncatedGaussian(
    const std::vector<double> &f, double sigmaPxReduced, int edgeAvgCount,
    double eps, double *out_mu_a, double *out_sigma_a_sq, double *out_mu_ab) {
  const int N = static_cast<int>(f.size());
  if (N <= 1) {
    if (out_mu_a)
      *out_mu_a = 0.0;
    if (out_sigma_a_sq)
      *out_sigma_a_sq = 0.0;
    if (out_mu_ab)
      *out_mu_ab = 0.0;
    return 0.0;
  }

  // --- sums for μ_a and σ_a^2 (apparent, discrete) ---
  double S0 = 0.0, S1 = 0.0, S2 = 0.0;
  for (int i = 0; i < N; ++i) {
    const double w = std::max(0.0, f[i]); // ensure non-negative
    S0 += w;
    S1 += w * i;
    S2 += w * i * i;
  }

  if (S0 <= 1e-12) {
    // degenerate: return middle as a safe value
    const double mid = 0.5 * (N - 1);
    if (out_mu_a)
      *out_mu_a = mid;
    if (out_sigma_a_sq)
      *out_sigma_a_sq = 0.0;
    if (out_mu_ab)
      *out_mu_ab = mid;
    return mid;
  }

  const double mu_a = S1 / S0;
  const double sigma_a_sq = std::max(0.0, S2 / S0 - mu_a * mu_a);

  // --- edge averages for f(a), f(b) (robust to noise) ---
  const int k = std::min(
      edgeAvgCount, std::max(1, N / 10)); // don't overreach on short signals
  double fa = 0.0, fb = 0.0;
  for (int i = 0; i < k; ++i)
    fa += f[i];
  for (int i = 0; i < k; ++i)
    fb += f[N - 1 - i];
  fa /= k;
  fb /= k;

  // --- μ_[a,b] with a=0, b=N-1 (discrete indices) ---
  // μ_[a,b] = (b f(b) - a f(a)) / (f(b) - f(a))  with a=0 => μ_[a,b] = b f(b) /
  // (f(b) - f(a))
  double mu_ab;
  const double denom_fb_fa = (fb - fa);
  if (std::abs(denom_fb_fa) <= 1e-12) {
    // symmetric edges -> μ_ab → ∞ ; in that case μ = μ_a (per note)
    mu_ab = std::numeric_limits<double>::infinity();
  } else {
    mu_ab = (N - 1) * fb / denom_fb_fa;
  }

  // --- choose σ: user-provided (full-res) scaled to reduced, or auto from σ_a
  // ---
  double sigma_used = sigmaPxReduced;
  if (sigma_used <= 0.0) {
    // fallback: use sqrt(σ_a^2) but guard with a minimum to avoid near-zero
    sigma_used = std::sqrt(std::max(sigma_a_sq, 1.0));
  }

  // --- t test: t = 2*(f(b)-f(a))/(f(b)+f(a)) ; if |t| < eps => use μ_a ---
  const double denom_sum = fb + fa;
  const double t =
      (std::abs(denom_sum) <= 1e-12) ? 0.0 : (2.0 * (fb - fa) / denom_sum);

  double mu_hat;
  if (!std::isfinite(mu_ab) || std::abs(t) < eps) {
    mu_hat = mu_a; // safe case near symmetric edges
  } else {
    const double denom = (mu_a - mu_ab);
    if (std::abs(denom) <= 1e-9) {
      mu_hat = mu_a; // avoid blow-ups
    } else {
      // Core formula: μ = μ_a + (σ_a^2 - σ^2) / (μ_a - μ_[a,b])
      mu_hat = mu_a + (sigma_a_sq - sigma_used * sigma_used) / denom;
    }
  }

  // Bound to observed domain to keep downstream code happy (drawing, indexing).
  // This *doesn't* introduce discrete jumps; μ̂ approaches the bound smoothly.
  mu_hat = std::clamp(mu_hat, 0.0, static_cast<double>(N - 1));

  if (out_mu_a)
    *out_mu_a = mu_a;
  if (out_sigma_a_sq)
    *out_sigma_a_sq = sigma_a_sq;
  if (out_mu_ab)
    *out_mu_ab = mu_ab;

  return mu_hat;
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
