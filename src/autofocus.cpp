#include "autofocus.hpp"
// #include "tiltedcam.hpp"
// #include "lens.hpp"
#include "main.hpp"
#include "logfile.hpp"
#include "ASICamera2.h" //TODO: Remove this when you move capturevideo() to tiltedcam.cc
#include "mainwindow.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <iterator>
#include <cmath>
#include <numeric>
#include <chrono>
#include <atomic>
#include <unistd.h>
#include <thread>
#include <sched.h>
#include <filesystem>
#include <sstream>
#include <iomanip>
// #include <gtk/gtk.h>

#include "opencv2/highgui/highgui.hpp"

// Global variables

// 0.0057 mm per pixel is the average!!!!

bool bAutofocusLogFlag = 0; // Flag that is 1 for when the autofocus log is being written to, 0 when it is not

std::atomic<bool> bNewImage = 0; // Flag that is 1 for when the buffer image is new, 0 when buffer image is old

const long img_size = 1280 * 960; // Replace with actual image size
bool bSaveImages = 0;             // Saves images from the tilted camera to output folder. WARNING: will produce enourmous number of images and slow down the system!
bool bSaveSharpnessCurves = 0;    // Saves text files with the sharpness curve data, similar to above

bool bBlinking = 0;

unsigned char *img_buf = (unsigned char *)malloc(img_size);      // Accessed by thread1 and thread2
unsigned char *img_get_buf = (unsigned char *)malloc(img_size);  // Used by by image pulling thread
unsigned char *img_calc_buf = (unsigned char *)malloc(img_size); // Used by image analysis thread
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
double Kp = 0.005; // Changed from 0.0012 to 0.0014

autofocus::autofocus() : lens1(),
                         tiltedcam1(),
                         stop_thread(false)
{
}

bool autofocus::initialize()
{
  bool bLensInit = lens1.initialize();
  bool bTiltedCamInit = tiltedcam1.initialize();
  std::cout << "bLensInit: " << bLensInit << " bTiltedCamInit: " << bTiltedCamInit << "\n";

  // Initialize benchmark CSV files with headers
  std::ofstream cpuBenchmarkFile("../output/focus_benchmark.csv");
  if (cpuBenchmarkFile.is_open())
  {
    cpuBenchmarkFile << "timestamp,total_time_us,clahe_time_us,blur_time_us,tenengrad_time_us,column_time_us,sliding_time_us,fitting_time_us" << std::endl;
    cpuBenchmarkFile.close();
  }

  // Initialize reduced resolution benchmark CSV file with headers
  std::ofstream reducedBenchmarkFile("../output/focus_benchmark_reduced.csv");
  if (reducedBenchmarkFile.is_open())
  {
    reducedBenchmarkFile << "timestamp,total_time_us,resize_time_us,clahe_time_us,blur_time_us,roberts_time_us,column_time_us,sliding_time_us,com_time_us" << std::endl;
    reducedBenchmarkFile.close();
  }

  // Initialize very reduced resolution benchmark CSV file with headers
  std::ofstream veryReducedBenchmarkFile("../output/focus_benchmark_very_reduced.csv");
  if (veryReducedBenchmarkFile.is_open())
  {
    veryReducedBenchmarkFile << "timestamp,total_time_us,resize_time_us,clahe_time_us,blur_time_us,roberts_time_us,column_time_us,sliding_time_us,fitting_time_us" << std::endl;
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

  if (bLensInit && bTiltedCamInit)
  {
    tAutofocus = std::thread(&autofocus::run, this);

    // Set high priority for autofocus thread
    pthread_t handle = tAutofocus.native_handle();
    struct sched_param params;
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (pthread_setschedparam(handle, SCHED_FIFO, &params) != 0)
    {
      // Fallback to nice priority if RT scheduling fails
      pthread_setschedprio(handle, -10); // Higher priority
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset); // Pin to CPU core 2 (adjust based on your system)
    CPU_SET(3, &cpuset); // And core 3
    pthread_setaffinity_np(tAutofocus.native_handle(), sizeof(cpu_set_t), &cpuset);

    if (bAutofocusLogFlag)
    {
      logger->info("[autofocus::autofocus] autofocus::run thread started");
    }

    // Initialize CSV logging
    std::string outputDir = "../output";
    std::filesystem::create_directories(outputDir);

    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    csvFilename = outputDir + "/autofocus_data.csv";
    csvFile.open(csvFilename);

    if (csvFile.is_open())
    {
      // Write CSV header using actual variable names
      csvFile << "timestamp_ms,imgcountfile,desiredLocBestFocus,locBestFocus,pSignal,filteredLocBestFocus,dSignal,totalPdSignal,Kp\n";
      csvFile.flush();
      logger->info("[autofocus::initialize] CSV logging started: " + csvFilename);
    }
    else
    {
      logger->error("[autofocus::initialize] Failed to open CSV file: " + csvFilename);
    }

    return true;
  }
  else
  {
    return false;
  }
}

autofocus::~autofocus()
{
  // Stops the autofocus thread
  stop_thread.store(true);

  // Make sure the tiltedcam capture thread is stopped
  tiltedcam1.stopCaptureThread();

  if (tAutofocus.joinable())
  {
    tAutofocus.join();
  }

  // Free the globally allocated buffers
  if (img_buf != nullptr)
  {
    free(img_buf);
    img_buf = nullptr;
  }

  if (img_get_buf != nullptr)
  {
    free(img_get_buf);
    img_get_buf = nullptr;
  }

  if (img_calc_buf != nullptr)
  {
    free(img_calc_buf);
    img_calc_buf = nullptr;
  }

  if (bAutofocusLogFlag)
  {
    logger->info("[autofocus::~autofocus] destructor completed");
  }

  // Close CSV file
  if (csvFile.is_open())
  {
    csvFile.close();
    logger->info("[autofocus::~autofocus] CSV file closed: " + csvFilename);
  }
}

void autofocus::run()
{
  auto t1 = std::chrono::steady_clock::now();
  usleep(10000); // 10ms
  auto t2 = std::chrono::steady_clock::now();
  auto s_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

  // For detecting oscillations
  std::deque<int> locBestFocusHistory; // use deque for easy push/pop at front and back

  // Start the camera capture thread
  logger->info("[autofocus::run] Starting camera capture thread");
  tiltedcam1.startCaptureThread();

  int moved = 1;
  int previous = desiredLocBestFocus; // TODO: should be in a mutex
  int tol = 6;                        // Tolerance zone where no movement is made
  int blink = 0;                      // Becomes 1 when a blink is detected
  int blinkframes = 15;               // number of frames to ignore when blink is detected
  imgcount = 0;                       // Keeps track of the number of images recieved and analyzed from the camera, but resets when switching between FindFocus and HoldFocus
  int imgcountfile = 0;               // Keeps track of TOTAL number of images, for filenaming and debugging

  bHoldFocus = 0;

  int imWidth = tiltedcam1.getImageWidth();
  int imHeight = tiltedcam1.getImageHeight();

  //// PID CONTROLLER
  double dt = 1.0 / 60.0; // time per frame on the TILTED CAMERA! Assumes 60Hz.
  double max = 3;         // maximum relative move the lens can be ordered to make. Set to +-3mm
  double min = -3;
  // double Kp = 0.0012;   this has to be set as a member variable now
  double Ki = 0.0;
  // double Kd = 0.00008;
  double Kd = 0.00005; // Changed from 0.0 to add a small derivative term

  // PD variables for manual calculation and logging
  double filteredPreviousError = 0.0;
  double filteredLocBestFocus = 0.0; // Will be initialized on first frame
  bool isFirstFrame = true;
  const double derivativeFilterAlpha = 0.1; // Low-pass filter for measurement

  if (bAutofocusLogFlag)
  {
    logger->info("[autofocus::run] while thread loop about to start");
  }

  while (!stop_thread.load())
  {
    // Only perform autofocus when either HoldFocus or FindFocus is active
    if (bHoldFocus || bFindFocus)
    {
      // Check if we have a new frame
      if (tiltedcam1.getLatestFrame(img_calc_buf, img_size))
      {
        framesProcessed++;
        bNewImage = true;
        imgcount++;
        imgcountfile++;
        // Convert to OpenCV Mat - use reduced resolution for all processing
        cv::Mat image(imHeight, imWidth, CV_8UC1, img_calc_buf);

        double locBestFocusDouble = computeBestFocusReduced(image, imHeight, imWidth); //  returns double

        static auto lastTime = std::chrono::high_resolution_clock::now();
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime).count();

        // Calculate FPS
        double fps = timeDiff > 0 ? 1000.0 / timeDiff : 0.0;

        // // Output to console for every frame
        // std::cout << "locBestFocus: " << locBestFocusDouble
        //           << ", desiredLocBestFocus: " << desiredLocBestFocus
        //           << ", FPS: " << std::fixed << std::setprecision(1) << fps << std::endl;

        lastTime = currentTime;

        // Use the reduced resolution result for all autofocus logic
        if (bSaveImages)
        {
          // Apply CLAHE to the image for saving (same as used in computeBestFocus)
          cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
          clahe->setClipLimit(2.0);
          clahe->setTilesGridSize(cv::Size(4, 4));
          cv::Mat clahe_enhanced_for_save;
          clahe->apply(image, clahe_enhanced_for_save);

          // Apply Gaussian blur to the image for saving
          cv::Mat blurred_preallocated_for_save;
          cv::GaussianBlur(clahe_enhanced_for_save, blurred_preallocated_for_save, cv::Size(3, 3), 1, 1, cv::BORDER_DEFAULT);

          // resize image to 1/2 resolution
          cv::Mat resized;
          cv::resize(blurred_preallocated_for_save, resized, cv::Size(), 0.5, 0.5);

          // locBestFocus is already in reduced resolution coordinates, so no additional scaling needed
          int scaledLocBestFocus = static_cast<int>(std::round(locBestFocusDouble));

          // Draw vertical line at best focus position
          cv::Point p1(scaledLocBestFocus, 0), p2(scaledLocBestFocus, resized.rows);

          // Convert resized to color if it's grayscale to match the graph image type
          cv::Mat colorResized;
          if (resized.channels() == 1)
          {
            cv::cvtColor(resized, colorResized, cv::COLOR_GRAY2BGR);
          }
          else
          {
            colorResized = resized.clone();
          }

          // Draw vertical line for center of mass if available - adjust for curve start position
          if (lastCenterOfMass >= 0)
          {
            int comX = 10 + static_cast<int>(lastCenterOfMass); // Add kernel/2 offset (10 for reduced resolution)
            if (comX >= 0 && comX < colorResized.cols)
            {
              cv::Point comP1(comX, 0);
              cv::Point comP2(comX, colorResized.rows);
              cv::line(colorResized, comP1, comP2, cv::Scalar(255, 255, 0), 2); // Cyan line for COM
            }
          }

          cv::Mat combined;

          // Draw graph if we have curve data
          if (!lastSharpnessCurve.empty())
          {
            try
            {
              // Create graph image with same width as resized image
              int graphHeight = 200;
              int graphWidth = colorResized.cols;
              cv::Mat graphImage = cv::Mat::zeros(graphHeight, graphWidth, CV_8UC3);

              // The sharpness curve starts at kernel/2 and has (imageWidth - kernel) points
              // For reduced resolution: starts at 10, has (640/2 - 20) = 300 points
              int curveStartX = 10; // kernel/2 for reduced resolution (20/2 = 10)
              int curveWidth = lastSharpnessCurve.size();

              // Find max value in sharpness curve for scaling
              double maxSharpness = lastSharpnessCurve.empty() ? 0.0 : *std::max_element(lastSharpnessCurve.begin(), lastSharpnessCurve.end());
              double minSharpness = lastSharpnessCurve.empty() ? 0.0 : *std::min_element(lastSharpnessCurve.begin(), lastSharpnessCurve.end());
              double amplitude = maxSharpness - minSharpness;

              // Scale based on sharpness curve range
              double maxVal = maxSharpness;
              if (maxVal <= 0)
                maxVal = 1.0;

              // Draw sharpness curve (blue) - map curve indices to correct x positions
              for (size_t i = 1; i < lastSharpnessCurve.size(); i++)
              {
                int x1 = curveStartX + (i - 1);
                int x2 = curveStartX + i;

                // Only draw if within graph bounds
                if (x1 >= 0 && x2 < graphWidth)
                {
                  int y1 = graphHeight - static_cast<int>((lastSharpnessCurve[i - 1] / maxVal) * (graphHeight - 50));
                  int y2 = graphHeight - static_cast<int>((lastSharpnessCurve[i] / maxVal) * (graphHeight - 50));
                  cv::line(graphImage, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0), 1); // Blue
                }
              }

              // Draw vertical line for center of mass on graph - adjust for curve start position
              if (lastCenterOfMass >= 0)
              {
                int comX = curveStartX + static_cast<int>(lastCenterOfMass);
                if (comX >= 0 && comX < graphWidth)
                {
                  cv::line(graphImage, cv::Point(comX, 0), cv::Point(comX, graphHeight), cv::Scalar(255, 255, 0), 2);
                }
              }

              // Add legend text and amplitude
              cv::putText(graphImage, "Sharpness Curve", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
              cv::putText(graphImage, "Center of Mass", cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

              // Display amplitude and COM with double precision
              std::string amplitudeText = "Amplitude: " + std::to_string(amplitude).substr(0, 6);
              std::string comText = "COM: " + std::to_string(locBestFocusDouble).substr(0, 8);
              cv::putText(graphImage, amplitudeText, cv::Point(10, graphHeight - 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
              cv::putText(graphImage, comText, cv::Point(10, graphHeight - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);

              // Combine image and graph
              cv::vconcat(colorResized, graphImage, combined);

              // Save with integer filename
              std::string FilePath = "../output/TiltedCam_Images/" + std::to_string(imgcountfile - 1) + "_" + std::to_string(static_cast<int>(std::round(locBestFocusDouble))) + ".png";
              cv::imwrite(FilePath, combined);
            }
            catch (const std::exception &e)
            {
              // Just save the original image if we can't combine
              std::string FilePath = "../output/TiltedCam_Images/" + std::to_string(imgcountfile - 1) + "_" + std::to_string(static_cast<int>(std::round(locBestFocusDouble))) + ".png";
              cv::imwrite(FilePath, colorResized);
            }
          }
          else
          {
            // Just save the original image if no curve data
            std::string FilePath = "../output/TiltedCam_Images/" + std::to_string(imgcountfile - 1) + "_" + std::to_string(static_cast<int>(std::round(locBestFocusDouble))) + ".png";
            cv::imwrite(FilePath, colorResized);
          }
        };

        // If imgcount==1, then the user has just turned on FindFocus or HoldFocus
        if (imgcount == 1)
        {
          if (bHoldFocus)
          {
            desiredLocBestFocus = static_cast<int>(std::round(locBestFocusDouble));
            previous = desiredLocBestFocus;
          }
          else if (bFindFocus)
          {
            desiredLocBestFocus = 400;
            previous = static_cast<int>(std::round(locBestFocusDouble));
            if (bAutofocusLogFlag)
            {
              logger->info("[autofocus::run] Set desiredLocBestFocus back to 320 in autofocus.cc");
            }
          }
        }

        // Filter the measurement for derivative term only
        if (isFirstFrame)
        {
          filteredLocBestFocus = static_cast<int>(std::round(locBestFocusDouble)); // Initialize on first frame
          isFirstFrame = false;
        }
        else
        {
          filteredLocBestFocus = static_cast<int>(std::round((1.0 - derivativeFilterAlpha) * filteredLocBestFocus + derivativeFilterAlpha * locBestFocusDouble));
        }

        // Calculate PD components - P uses raw measurement with double precision
        double currentError = desiredLocBestFocus - locBestFocusDouble; // Use double for higher precision

        // Scale P gain based on error magnitude
        double errorMagnitude = abs(currentError);
        double pScaleFactor;
        if (errorMagnitude <= 3.0)
        {
          pScaleFactor = 0.1;
        }
        else if (errorMagnitude >= 100.0)
        {
          pScaleFactor = 1.0;
        }
        else
        {
          // Linear interpolation between 3 pixels (0.1x) and 100 pixels (1.0x)
          pScaleFactor = 0.1 + (errorMagnitude - 3.0) * (1.0 - 0.1) / (100.0 - 3.0);
        }

        // Apply directional multiplier for negative errors (locBestFocus < desired)
        double effectiveKp = Kp;
        if (currentError > 0)
        {
          effectiveKp *= 1.3; // 30% more aggressive when moving in positive direction
        }

        double pSignal = effectiveKp * currentError * pScaleFactor;

        // Calculate derivative using filtered measurement
        double filteredCurrentError = desiredLocBestFocus - filteredLocBestFocus;

        // Calculate derivative using filtered error
        double rawDerivative = (filteredCurrentError - filteredPreviousError) / dt;
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

        // Log data to CSV
        if (csvFile.is_open())
        {
          auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count();

          csvFile << currentTime << ","
                  << imgcountfile << ","
                  << desiredLocBestFocus << ","
                  << locBestFocusDouble << ","
                  << pSignal << ","
                  << filteredLocBestFocus << ","
                  << dSignal << ","
                  << totalPdSignal << ","
                  << Kp << "\n";

          // Flush every 10 frames to ensure data is written
          if (imgcountfile % 10 == 0)
          {
            csvFile.flush();
          }
        }

        //// BLINK DETECTION. TODO: try removing 'moved' variable
        if (moved == 0 && blink == 0 && abs(static_cast<int>(std::round(locBestFocusDouble)) - previous) > (50) && bBlinking)
        { // if the location of best focus changes by more than 50 pixels with no move, it is a blink
          // Blink starts
          if (bAutofocusLogFlag)
            logger->info("[autofocus::run] Frame ignored; blink detected");
          blink = 1;
        }
        else if (blink == 1)
        {
          if (bAutofocusLogFlag)
            logger->info("[autofocus::run] Frame ignored; blink detected");
          blinkframes--;
          if (blinkframes == 0)
          {
            blinkframes = 15; // resets blinkframes
            blink = 0;
          }
        }
        else
        {
          // Use double precision measurement for tolerance checking
          if (abs(locBestFocusDouble - desiredLocBestFocus) <= tol)
          {
            // std::cout << ", in TOL band\n";
            moved = 0;
          }
          else
          {
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
            // // Check if history contains values both above and below desiredLocBestFocus
            // if(locBestFocusHistory.size() == 20) {
            //   bool hasAbove = std::any_of(locBestFocusHistory.begin(), locBestFocusHistory.end(), [](int x) { return x > desiredLocBestFocus; });
            //   bool hasBelow = std::any_of(locBestFocusHistory.begin(), locBestFocusHistory.end(), [](int x) { return x < desiredLocBestFocus; });
            //   if (hasAbove && hasBelow) {
            //     // Find the value closest to desiredLocBestFocus
            //     auto closestIt = std::min_element(locBestFocusHistory.begin(), locBestFocusHistory.end(), [](int a, int b) {
            //         return std::abs(a - desiredLocBestFocus) < std::abs(b - desiredLocBestFocus);
            //     });
            //     desiredLocBestFocus = *closestIt;
            //     std::cout << "DETECTED OSCILLATION!! Adjusting desiredLocBestFocus to " << desiredLocBestFocus << "\n";
            //     locBestFocusHistory.clear(); // Clear history after adjusting desiredLocBestFocus
            //   }
            // }
          }
        }
        previous = static_cast<int>(std::round(locBestFocusDouble));

        // Periodically report frame drop rate
        if (framesProcessed % 300 == 0)
        { // Every 5 seconds at 60fps
          // std::cout << "Frames processed: " << framesProcessed
          //<< ", Estimated drops: " << (framesProcessed * 17/16.67 - framesProcessed) << std::endl;
        }
      }
    }
    else
    {
      // Reset counter when autofocus is not active
      imgcount = 0;

      // Sleep a bit to reduce CPU usage when not actively focusing
      usleep(50000); // 50ms
    }
  }
  tiltedcam1.stopCaptureThread();
}

int autofocus::computeBestFocus(cv::Mat image, int imgHeight, int imgWidth)
{
  auto startTime = std::chrono::high_resolution_clock::now();

  // CLAHE preprocessing with reduced noise amplification
  auto claheStart = std::chrono::high_resolution_clock::now();
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(2.0);                // Reduced from 2.0 to limit noise amplification
  clahe->setTilesGridSize(cv::Size(8, 8)); // Larger tiles (16x16) for less aggressive local adaptation
  cv::Mat clahe_enhanced;
  clahe->apply(image, clahe_enhanced);
  auto claheEnd = std::chrono::high_resolution_clock::now();

  // Gaussian Blur: ~300μs (5.5%)
  auto blurStart = std::chrono::high_resolution_clock::now();
  cv::GaussianBlur(clahe_enhanced, blurred_preallocated, cv::Size(3, 3), 1, 1, cv::BORDER_DEFAULT);
  auto blurEnd = std::chrono::high_resolution_clock::now();

  // Tenengrad method: ~1,400μs (25.5%), occasionally spikes to ~4,000μs
  auto tenengradStart = std::chrono::high_resolution_clock::now();

  // Compute gradients using Sobel operators
  cv::Sobel(blurred_preallocated, img_x_preallocated, CV_16S, 1, 0, 3); // 3x3 Sobel for X direction
  cv::Sobel(blurred_preallocated, img_y_preallocated, CV_16S, 0, 1, 3); // 3x3 Sobel for Y direction

  // Square the gradients and sum them (Tenengrad formula: Gx² + Gy²)
  cv::multiply(img_x_preallocated, img_x_preallocated, img_x_squared_preallocated);
  cv::multiply(img_y_preallocated, img_y_preallocated, img_y_squared_preallocated);
  cv::add(img_x_squared_preallocated, img_y_squared_preallocated, sum_xy_preallocated);

  // Convert to float for better precision in subsequent calculations
  sum_xy_preallocated.convertTo(sharpness_float_preallocated, CV_32F);
  auto tenengradEnd = std::chrono::high_resolution_clock::now();

  // Column Means: ~2,400μs (43.6%), occasionally spikes to ~6,000μs
  auto columnStart = std::chrono::high_resolution_clock::now();
  cv::Mat columnMeansMatrix;
  cv::reduce(sharpness_float_preallocated, columnMeansMatrix, 0, cv::REDUCE_AVG, CV_64F);

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
  for (int i = 0; i < kernel; i++)
  {
    hammingWindow[i] = 0.54 - 0.46 * std::cos(2 * M_PI * i / (kernel - 1.0));
  }

  for (int i = 0; i < imgWidth - kernel; i++)
  {
    double regionSharpnessScore = 0.0;
    double windowSum = 0.0;
    for (int k = 0; k < kernel; k++)
    {
      regionSharpnessScore += columnMeans[i + k] * hammingWindow[k];
      windowSum += hammingWindow[k];
    }
    regionSharpnessScore /= windowSum; // Normalize by sum of window coefficients
    sharpnesscurve.push_back(regionSharpnessScore);
  }
  auto slidingEnd = std::chrono::high_resolution_clock::now();

  // Calculate and save the max amplitude of the sharpness curve
  double maxVal = *std::max_element(sharpnesscurve.begin(), sharpnesscurve.end());
  double minVal = *std::min_element(sharpnesscurve.begin(), sharpnesscurve.end());
  double amplitude = maxVal - minVal;
  double offset = minVal;
  // If the amplitude is too low, return the desiredLocBestFocus so no lens movement occurs. TODO: add error message
  if (amplitude < 0.3)
  {
    std::cout << "Amplitude is too low, returning desiredLocBestFocus\n";
    return desiredLocBestFocus;
  }

  // Fitting a normal curve to the sharpness curve, to avoid local peaks in the data around vessel edges (and deal with multiple real peaks)
  // Curve Fitting: ~1,400μs (25.5%), occasionally spikes to ~4,000μs
  auto fittingStart = std::chrono::high_resolution_clock::now();
  //  std::vector<double> sharpnesscurvenormalized = fitnormalcurve(sharpnesscurve, amplitude, offset, 0.2);
  std::vector<double> sharpnesscurvenormalized = fitnormalcurveBruteForce(sharpnesscurve, amplitude, offset, 0.2);
  auto fittingEnd = std::chrono::high_resolution_clock::now();

  // printing to text files for testing
  if (bSaveSharpnessCurves)
  {
    std::string FileName = "TESTING" + std::to_string(increment);
    std::string TextFile = "/home/hvi/Desktop/HVI-data/Blendi_SharpnessCurves/" + FileName + "_SharpnessCurve.txt";
    std::string TextFile2 = "/home/hvi/Desktop/HVI-data/Blendi_SharpnessCurves/" + FileName + "_FittedNorm.txt";
    std::ofstream outputFile(TextFile);
    std::ofstream outputFile2(TextFile2);
    std::ostream_iterator<double> output_iterator(outputFile, ", ");
    std::copy(sharpnesscurve.begin(), sharpnesscurve.end(), output_iterator);
    outputFile << "\n";
    std::ostream_iterator<double> output_iterator2(outputFile2, ", ");
    std::copy(sharpnesscurvenormalized.begin(), sharpnesscurvenormalized.end(), output_iterator2);
    outputFile2 << "\n";
    outputFile.close();
    outputFile2.close();
    increment++;
  }

  int locBestFocus = distance(begin(sharpnesscurvenormalized), max_element(begin(sharpnesscurvenormalized), end(sharpnesscurvenormalized)));

  if (bSaveImages)
  {
    // Store the curves for visualization
    lastSharpnessCurve = sharpnesscurve;
    lastFittedCurve = sharpnesscurvenormalized;
  }

  auto endTime = std::chrono::high_resolution_clock::now();

  // Calculate timing benchmarks
  auto claheTime = std::chrono::duration_cast<std::chrono::microseconds>(claheEnd - claheStart);
  auto blurTime = std::chrono::duration_cast<std::chrono::microseconds>(blurEnd - blurStart);
  auto tenengradTime = std::chrono::duration_cast<std::chrono::microseconds>(tenengradEnd - tenengradStart);
  auto columnTime = std::chrono::duration_cast<std::chrono::microseconds>(columnEnd - columnStart);
  auto slidingTime = std::chrono::duration_cast<std::chrono::microseconds>(slidingEnd - slidingStart);
  auto fittingTime = std::chrono::duration_cast<std::chrono::microseconds>(fittingEnd - fittingStart);
  auto totalTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);

  // Save timing information to CSV file
  std::ofstream benchmarkFile("../output/focus_benchmark.csv", std::ios::app);
  if (benchmarkFile.is_open())
  {
    time_t now = time(0);
    benchmarkFile << now << ","
                  << totalTime.count() << ","
                  << claheTime.count() << ","
                  << blurTime.count() << ","
                  << tenengradTime.count() << ","
                  << columnTime.count() << ","
                  << slidingTime.count() << ","
                  << fittingTime.count() << std::endl;
    benchmarkFile.close();
  }

  return locBestFocus + kernel / 2; // Note: kernel/2 offset needed for sliding window
}

double autofocus::computeBestFocusReduced(cv::Mat image, int imgHeight, int imgWidth)
{
  auto startTime = std::chrono::high_resolution_clock::now();

  // Resize to 1/4 size (1/2 in each dimension)
  auto resizeStart = std::chrono::high_resolution_clock::now();
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), 0.5, 0.5);
  auto resizeEnd = std::chrono::high_resolution_clock::now();

  // Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
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
  cv::GaussianBlur(clahe_enhanced, blurred, cv::Size(3, 3), 1, 1, cv::BORDER_DEFAULT);
  auto blurEnd = std::chrono::high_resolution_clock::now();

  // Compute sharpness using Roberts Cross operator
  auto robertsStart = std::chrono::high_resolution_clock::now();
  cv::Mat img_x, img_y;
  cv::filter2D(blurred, img_x, CV_16S, roberts_kernelx);
  cv::filter2D(blurred, img_y, CV_16S, roberts_kernely);

  // Square the gradients and sum them
  cv::Mat img_x_squared, img_y_squared, sum_xy;
  cv::multiply(img_x, img_x, img_x_squared);
  cv::multiply(img_y, img_y, img_y_squared);
  cv::add(img_x_squared, img_y_squared, sum_xy);

  // Convert to float for better precision
  cv::Mat sharpness_float;
  sum_xy.convertTo(sharpness_float, CV_32F);
  auto robertsEnd = std::chrono::high_resolution_clock::now();

  // Column Means
  auto columnStart = std::chrono::high_resolution_clock::now();
  cv::Mat columnMeansMatrix;
  cv::reduce(sharpness_float, columnMeansMatrix, 0, cv::REDUCE_AVG, CV_64F);

  // Convert to vector for compatibility with existing code
  std::vector<double> columnMeans;
  columnMeansMatrix.copyTo(columnMeans);
  auto columnEnd = std::chrono::high_resolution_clock::now();

  // Sliding Window with Hamming
  auto slidingStart = std::chrono::high_resolution_clock::now();
  std::vector<double> sharpnesscurve;
  int kernel = 20; // Reduced from 40 for 1/2 resolution (40/2 = 20)

  // Pre-compute Hamming window coefficients
  std::vector<double> hammingWindow(kernel);
  for (int i = 0; i < kernel; i++)
  {
    hammingWindow[i] = 0.54 - 0.46 * std::cos(2 * M_PI * i / (kernel - 1.0));
  }

  for (int i = 0; i < blurred.cols - kernel; i++)
  {
    double regionSharpnessScore = 0.0;
    double windowSum = 0.0;
    for (int k = 0; k < kernel; k++)
    {
      regionSharpnessScore += columnMeans[i + k] * hammingWindow[k];
      windowSum += hammingWindow[k];
    }
    regionSharpnessScore /= windowSum; // Normalize by sum of window coefficients
    sharpnesscurve.push_back(regionSharpnessScore);
  }
  auto slidingEnd = std::chrono::high_resolution_clock::now();

  // Calculate and save the max amplitude of the sharpness curve
  double minVal = *std::min_element(sharpnesscurve.begin(), sharpnesscurve.end());
  for (double &val : sharpnesscurve)
  {
    val -= minVal;
  }
  double maxVal = *std::max_element(sharpnesscurve.begin(), sharpnesscurve.end());
  double amplitude = maxVal;
  // Print center of mass to console
  // std::cout << "COM: " << centerOfMass << ", Amplitude: " << amplitude << std::endl;

  // If the amplitude is too low, return the scaled desiredLocBestFocus
  if (amplitude < 4.0)
  {
    std::cout << "Amplitude is too low, returning scaled desiredLocBestFocus\n";
    return static_cast<double>(desiredLocBestFocus);
  }
  // Calculate center of mass
  auto comStart = std::chrono::high_resolution_clock::now();
  double centerOfMass = findCenterOfMass(sharpnesscurve);
  auto comEnd = std::chrono::high_resolution_clock::now();

  // Save sharpness curves if enabled
  if (bSaveSharpnessCurves)
  {
    std::string FileName = "TESTING_REDUCED" + std::to_string(increment);
    std::string TextFile = "/home/hvi/Desktop/HVI-data/Blendi_SharpnessCurves/" + FileName + "_SharpnessCurve.txt";
    std::ofstream outputFile(TextFile);
    std::ostream_iterator<double> output_iterator(outputFile, ", ");
    std::copy(sharpnesscurve.begin(), sharpnesscurve.end(), output_iterator);
    outputFile << "\n";
    outputFile.close();
  }

  if (bSaveImages)
  {
    // Store the curves for visualization and center of mass
    lastSharpnessCurve = sharpnesscurve;
    lastFittedCurve.clear(); // No fitted curve when using COM
    lastCenterOfMass = centerOfMass;
  }

  auto endTime = std::chrono::high_resolution_clock::now();

  // Benchmark timing
  auto totalTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
  auto resizeTime = std::chrono::duration_cast<std::chrono::microseconds>(resizeEnd - resizeStart);
  auto claheTime = std::chrono::duration_cast<std::chrono::microseconds>(claheEnd - claheStart);
  auto blurTime = std::chrono::duration_cast<std::chrono::microseconds>(blurEnd - blurStart);
  auto robertsTime = std::chrono::duration_cast<std::chrono::microseconds>(robertsEnd - robertsStart);
  auto columnTime = std::chrono::duration_cast<std::chrono::microseconds>(columnEnd - columnStart);
  auto slidingTime = std::chrono::duration_cast<std::chrono::microseconds>(slidingEnd - slidingStart);
  auto comTime = std::chrono::duration_cast<std::chrono::microseconds>(comEnd - comStart);

  std::ofstream reducedBenchmarkFile("../output/focus_benchmark_reduced.csv", std::ios::app);
  if (reducedBenchmarkFile.is_open())
  {
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
    reducedBenchmarkFile << timestamp << ","
                         << totalTime.count() << ","
                         << resizeTime.count() << ","
                         << claheTime.count() << ","
                         << blurTime.count() << ","
                         << robertsTime.count() << ","
                         << columnTime.count() << ","
                         << slidingTime.count() << ","
                         << comTime.count() << std::endl;
    reducedBenchmarkFile.close();
  }

  return (centerOfMass + kernel / 2);
}

int autofocus::computeBestFocusVeryReduced(cv::Mat image, int imgHeight, int imgWidth)
{
  auto startTime = std::chrono::high_resolution_clock::now();

  // Resize to 1/16 size (1/4 in each dimension)
  auto resizeStart = std::chrono::high_resolution_clock::now();
  cv::Mat resized_image;
  cv::resize(image, resized_image, cv::Size(imgWidth / 4, imgHeight / 4), 0, 0, cv::INTER_LINEAR);
  int reducedWidth = resized_image.cols;
  int reducedHeight = resized_image.rows;
  auto resizeEnd = std::chrono::high_resolution_clock::now();

  // CLAHE preprocessing with reduced noise amplification
  auto claheStart = std::chrono::high_resolution_clock::now();
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(2.0);
  clahe->setTilesGridSize(cv::Size(4, 4)); // Increased from 2x2 to 4x4 to avoid artifacts
  cv::Mat clahe_enhanced;
  clahe->apply(resized_image, clahe_enhanced);
  auto claheEnd = std::chrono::high_resolution_clock::now();

  // Gaussian Blur
  auto blurStart = std::chrono::high_resolution_clock::now();
  cv::Mat blurred_very_reduced;
  cv::GaussianBlur(clahe_enhanced, blurred_very_reduced, cv::Size(3, 3), 1, 1, cv::BORDER_DEFAULT);
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
  cv::reduce(sharpness_float_roberts, columnMeansMatrix, 0, cv::REDUCE_AVG, CV_64F);

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
  for (int i = 0; i < kernel; i++)
  {
    hammingWindow[i] = 0.54 - 0.46 * std::cos(2 * M_PI * i / (kernel - 1.0));
  }

  for (int i = 0; i < reducedWidth - kernel; i++)
  {
    double regionSharpnessScore = 0.0;
    double windowSum = 0.0;
    for (int k = 0; k < kernel; k++)
    {
      regionSharpnessScore += columnMeans[i + k] * hammingWindow[k];
      windowSum += hammingWindow[k];
    }
    regionSharpnessScore /= windowSum; // Normalize by sum of window coefficients
    sharpnesscurve.push_back(regionSharpnessScore);
  }
  auto slidingEnd = std::chrono::high_resolution_clock::now();

  // Calculate and save the max amplitude of the sharpness curve
  double maxVal = *std::max_element(sharpnesscurve.begin(), sharpnesscurve.end());
  double minVal = *std::min_element(sharpnesscurve.begin(), sharpnesscurve.end());
  double amplitude = maxVal - minVal;
  double offset = minVal;

  // If the amplitude is too low, return the scaled desiredLocBestFocus
  if (amplitude < 0.3)
  {
    std::cout << "Amplitude is too low, returning scaled desiredLocBestFocus\n";
    return desiredLocBestFocus / 4; // Scale down for very reduced resolution
  }

  // Fitting a normal curve - scale std_dev_factor for very reduced resolution
  auto fittingStart = std::chrono::high_resolution_clock::now();
  // std::vector<double> sharpnesscurvenormalized = fitnormalcurveBruteForce(sharpnesscurve, amplitude, offset, 0.2); // 0.2 * 4 = 0.8
  std::vector<double> sharpnesscurvenormalized = fitnormalcurveBruteForce(sharpnesscurve, amplitude, offset, 0.2); // 0.2 * 4 = 0.8

  auto fittingEnd = std::chrono::high_resolution_clock::now();

  // Save sharpness curves if enabled
  if (bSaveSharpnessCurves)
  {
    std::string FileName = "TESTING_VERY_REDUCED" + std::to_string(increment);
    std::string TextFile = "/home/hvi/Desktop/HVI-data/Blendi_SharpnessCurves/" + FileName + "_SharpnessCurve.txt";
    std::string TextFile2 = "/home/hvi/Desktop/HVI-data/Blendi_SharpnessCurves/" + FileName + "_FittedNorm.txt";
    std::ofstream outputFile(TextFile);
    std::ofstream outputFile2(TextFile2);
    std::ostream_iterator<double> output_iterator(outputFile, ", ");
    std::copy(sharpnesscurve.begin(), sharpnesscurve.end(), output_iterator);
    outputFile << "\n";
    std::ostream_iterator<double> output_iterator2(outputFile2, ", ");
    std::copy(sharpnesscurvenormalized.begin(), sharpnesscurvenormalized.end(), output_iterator2);
    outputFile2 << "\n";
    outputFile.close();
    outputFile2.close();
  }

  int locBestFocus = distance(begin(sharpnesscurvenormalized), max_element(begin(sharpnesscurvenormalized), end(sharpnesscurvenormalized)));

  if (bSaveImages)
  {
    // Scale the curves for visualization to match full resolution length
    // Original full resolution curve would be approximately (imgWidth - 40) points
    // Very reduced curve is approximately (imgWidth/4 - 10) points
    // Scale factor is roughly 4x

    int targetLength = imgWidth - 40; // Target length to match full resolution
    std::vector<double> scaledSharpnessCurve;
    std::vector<double> scaledFittedCurve;

    // Simple linear interpolation to scale up the curves
    for (int i = 0; i < targetLength; i++)
    {
      double sourceIndex = (double)i * (sharpnesscurve.size() - 1) / (targetLength - 1);
      int lowerIndex = (int)sourceIndex;
      int upperIndex = std::min(lowerIndex + 1, (int)sharpnesscurve.size() - 1);
      double fraction = sourceIndex - lowerIndex;

      // Interpolate sharpness curve
      double interpolatedSharpness = sharpnesscurve[lowerIndex] * (1.0 - fraction) +
                                     sharpnesscurve[upperIndex] * fraction;
      scaledSharpnessCurve.push_back(interpolatedSharpness);

      // Interpolate fitted curve
      double interpolatedFitted = sharpnesscurvenormalized[lowerIndex] * (1.0 - fraction) +
                                  sharpnesscurvenormalized[upperIndex] * fraction;
      scaledFittedCurve.push_back(interpolatedFitted);
    }

    // Store the scaled curves for visualization
    lastSharpnessCurve = scaledSharpnessCurve;
    lastFittedCurve = scaledFittedCurve;
  }

  auto endTime = std::chrono::high_resolution_clock::now();

  // Calculate timing benchmarks
  auto resizeTime = std::chrono::duration_cast<std::chrono::microseconds>(resizeEnd - resizeStart);
  auto claheTime = std::chrono::duration_cast<std::chrono::microseconds>(claheEnd - claheStart);
  auto blurTime = std::chrono::duration_cast<std::chrono::microseconds>(blurEnd - blurStart);
  auto robertsTime = std::chrono::duration_cast<std::chrono::microseconds>(robertsEnd - robertsStart);
  auto columnTime = std::chrono::duration_cast<std::chrono::microseconds>(columnEnd - columnStart);
  auto slidingTime = std::chrono::duration_cast<std::chrono::microseconds>(slidingEnd - slidingStart);
  auto fittingTime = std::chrono::duration_cast<std::chrono::microseconds>(fittingEnd - fittingStart);
  auto totalTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);

  // Save timing information to CSV file with different filename
  std::ofstream benchmarkFile("../output/focus_benchmark_very_reduced.csv", std::ios::app);
  if (benchmarkFile.is_open())
  {
    time_t now = time(0);
    benchmarkFile << now << ","
                  << totalTime.count() << ","
                  << resizeTime.count() << ","
                  << claheTime.count() << ","
                  << blurTime.count() << ","
                  << robertsTime.count() << ","
                  << columnTime.count() << ","
                  << slidingTime.count() << ","
                  << fittingTime.count() << std::endl;
    benchmarkFile.close();
  }

  // Scale the result back to full resolution and add kernel offset
  return (locBestFocus + kernel / 2) * 4; // Scale up by 4x for full resolution
}

cv::Scalar autofocus::robertscross(cv::Mat imagedata)
{
  // // Creating Roberts Cross matrices
  // cv::Mat matrixx = (cv::Mat_<double>(2, 2) << 1, 0, 0, -1);
  // cv::Mat matrixy = (cv::Mat_<double>(2, 2) << 0, 1, -1, 0);

  // //convolving imagedata with the matrices
  // cv::Mat img_x, img_y;
  // cv::filter2D(imagedata, img_x, -1, matrixx);
  // cv::filter2D(imagedata, img_y, -1, matrixy);

  // //squaring and summing the resultant matrices, which gives us the sharpness (or, gradient) for each pixel in imagedata, and taking the average score over the portion of image
  // cv::Scalar mean_sharpness = mean(img_x.mul(img_x) + img_y.mul(img_y));
  // return mean_sharpness;

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

  // squaring and summing the resultant matrices, which gives us the sharpness (or, gradient) for each pixel in imagedata, and taking the average score over the portion of image
  cv::Mat img_x_squared, img_y_squared;
  cv::multiply(img_x, img_x, img_x_squared);
  cv::multiply(img_y, img_y, img_y_squared);
  cv::Mat sum_xy;
  cv::add(img_x_squared, img_y_squared, sum_xy);
  cv::Scalar mean_sharpness = cv::mean(sum_xy);
  return mean_sharpness;
}

cv::Scalar autofocus::tenengrad(cv::Mat img)
{
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
cv::Scalar autofocus::vollath(cv::Mat img)
{
  // Vollath's F4
  int sum1 = 0;
  int sum2 = 0;
  int h = img.rows; // 480
  int w = img.cols; // 16
  for (int i = 0; i < h - 1; i++)
  {
    for (int j = 0; j < w; j++)
    {
      sum1 = sum1 + static_cast<int>(img.at<uchar>(i, j)) * static_cast<int>(img.at<uchar>(i + 1, j)); // might need to flip i and j
    }
  }
  for (int i = 0; i < h - 2; i++)
  {
    for (int j = 0; j < w; j++)
    {
      sum2 = sum2 + static_cast<int>(img.at<uchar>(i, j)) * static_cast<int>(img.at<uchar>(i + 2, j));
    }
  }
  cv::Scalar mean_grad_vol = sum1 - sum2;
  return mean_grad_vol;
}
cv::Scalar autofocus::canny(cv::Mat img)
{
  // Canny //
  cv::Mat edges;
  cv::Canny(img, edges, 25, 55, 3);
  cv::Scalar mean_grad_canny = mean(edges.mul(edges));
  return mean_grad_canny;
}

std::vector<double> autofocus::fitnormalcurve(std::vector<double> sharpnesscurve, double amplitude, double offset, double std_dev_factor)
{
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

  for (int iter = 0; iter < maxIterations; iter++)
  {
    double h = 0.1;

    // Calculate gradient for mean only
    double gradientMean;
    if (currentMean < h)
    {
      double errorCurrent = calculateErrorWithAmplitudeAndOffset(sharpnesscurve, currentMean, amplitude, offset, sigma);
      double errorForward = calculateErrorWithAmplitudeAndOffset(sharpnesscurve, currentMean + h, amplitude, offset, sigma);
      gradientMean = (errorForward - errorCurrent) / h;
    }
    else if (currentMean > sharpnesscurve.size() - 1 - h)
    {
      double errorCurrent = calculateErrorWithAmplitudeAndOffset(sharpnesscurve, currentMean, amplitude, offset, sigma);
      double errorBackward = calculateErrorWithAmplitudeAndOffset(sharpnesscurve, currentMean - h, amplitude, offset, sigma);
      gradientMean = (errorCurrent - errorBackward) / h;
    }
    else
    {
      double errorPlus = calculateErrorWithAmplitudeAndOffset(sharpnesscurve, currentMean + h, amplitude, offset, sigma);
      double errorMinus = calculateErrorWithAmplitudeAndOffset(sharpnesscurve, currentMean - h, amplitude, offset, sigma);
      gradientMean = (errorPlus - errorMinus) / (2.0 * h);
    }

    // Update mean with bounds checking
    double newMean = currentMean - learningRate * gradientMean;
    newMean = std::max(minMean, std::min(maxMean, newMean)); // Clamp to bounds

    // Check for convergence
    if (std::abs(newMean - currentMean) < tolerance)
    {
      break;
    }

    currentMean = newMean;
  }

  // Generate the fitted curve exactly like the old version
  std::vector<double> norm_curve;
  for (int i = 0; i < sharpnesscurve.size(); i++)
  {
    double normResult = normpdf(i, currentMean, sigma);
    norm_curve.push_back(normResult);
  }

  // Scale exactly like the old version: find max, then scale so max equals amplitude
  double maxNormValue = *std::max_element(norm_curve.begin(), norm_curve.end());
  double factor = 1.0 / maxNormValue;

  for (int i = 0; i < sharpnesscurve.size(); i++)
  {
    norm_curve[i] = norm_curve[i] * factor * amplitude + offset;
  }

  return norm_curve;
}

double autofocus::calculateErrorWithAmplitudeAndOffset(const std::vector<double> &sharpnesscurve, double mean, double amplitude, double offset, double sigma)
{
  double totalError = 0.0;

  // Generate normal curve for this mean
  std::vector<double> norm_curve_temp;
  for (int i = 0; i < sharpnesscurve.size(); i++)
  {
    double normResult = normpdf(i, mean, sigma);
    norm_curve_temp.push_back(normResult);
  }

  // Scale exactly like the old version
  double maxNormValue = *std::max_element(norm_curve_temp.begin(), norm_curve_temp.end());
  double factor = 1.0 / maxNormValue;

  // Calculate error using absolute difference (like old version)
  for (int i = 0; i < sharpnesscurve.size(); i++)
  {
    double scaledNormValue = norm_curve_temp[i] * factor * amplitude + offset;
    double error = std::abs(sharpnesscurve[i] - scaledNormValue);
    totalError += error;
  }

  return totalError;
}

double autofocus::normpdf(double x, double u, double s)
{
  const double ONE_OVER_SQRT_2PI = 0.39894228040143267793994605993438;
  return (ONE_OVER_SQRT_2PI / s) * exp(-0.5 * std::pow(((x - u) / s), 2));
}

void autofocus::adjust_bestFocus(int val)
{
  // TODO: should be in mutex
  desiredLocBestFocus = val;
}

void autofocus::setPGain(double gain)
{
  Kp = gain;
  std::cout << "P gain set to: " << Kp << std::endl;
  if (bAutofocusLogFlag)
  {
    logger->info("[autofocus::setPGain] P gain set to: {}", gain);
  }
}

double autofocus::getPGain() const
{
  return Kp;
}

std::vector<double> autofocus::fitnormalcurveBruteForce(std::vector<double> sharpnesscurve, double amplitude, double offset, double std_dev_factor)
{
  // Fits a normal curve to the data. Should try to find a proper curve-fitting library...
  std::vector<double> norm_curve;
  std::vector<double> sum_of_diffs;
  double std_dev = std_dev_factor * (sharpnesscurve.size()); // now uses the parameter

  // Calculates sum_of_elems, a vector showing how well each normal curve with mean j fits grad
  for (int j = 0; j < sharpnesscurve.size(); j++) // Start from 0, not 1
  {
    // Generates normal curve for given mean j
    std::vector<double> norm_curve_temp;
    for (int i = 0; i < sharpnesscurve.size(); i++)
    {
      double normResult = normpdf(i, j, std_dev); // j is now the actual mean position
      norm_curve_temp.push_back(normResult);
    }
    // scales so the max value (when i=j) is equal to the desired amplitude
    double factor = 1.0 / *max_element(begin(norm_curve_temp), end(norm_curve_temp));
    for (int i = 0; i < sharpnesscurve.size(); i++)
    {
      norm_curve_temp[i] = norm_curve_temp[i] * factor * amplitude + offset;
    }

    // Calculates difference between norm curve and sharpness curve
    std::vector<double> differences;
    for (int z = 0; z < sharpnesscurve.size(); z++)
    {
      double diff = abs(sharpnesscurve[z] - norm_curve_temp[z]);
      differences.push_back(diff);
    }
    double sum_of_diff = std::accumulate(differences.begin(), differences.end(), decltype(differences)::value_type(0));
    sum_of_diffs.push_back(sum_of_diff);
  }

  // Picks the mean that has the lowest overall difference
  int bestMeanIndex = distance(begin(sum_of_diffs), min_element(begin(sum_of_diffs), end(sum_of_diffs)));
  double mean = bestMeanIndex; // No +1 needed since j starts from 0

  // Generates final norm_curve using the correct mean
  for (int i = 0; i < sharpnesscurve.size(); i++)
  {
    double normResult = normpdf(i, mean, std_dev);
    norm_curve.push_back(normResult);
  }
  double factor = 1.0 / *max_element(begin(norm_curve), end(norm_curve));
  for (int i = 0; i < (sharpnesscurve.size()); i++)
  {
    norm_curve[i] = norm_curve[i] * factor * amplitude + offset;
  }

  std::vector<double> fittednormalcurve(norm_curve.begin(), norm_curve.end());
  return fittednormalcurve;
}

double autofocus::findCenterOfMass(const std::vector<double> &curve)
{
  double weightedSum = 0.0;
  double totalWeight = 0.0;

  for (int i = 0; i < curve.size(); i++)
  {
    weightedSum += i * curve[i];
    totalWeight += curve[i];
  }

  return totalWeight > 0 ? (weightedSum / totalWeight) : curve.size() / 2.0;
}
