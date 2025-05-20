#include "autofocus.hpp"
// #include "tiltedcam.hpp"
// #include "lens.hpp"
#include "pid.hpp"
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
// #include <gtk/gtk.h>

#include "opencv2/highgui/highgui.hpp"

// Global variables

// 0.0057 mm per pixel is the average!!!!

bool bAutofocusLogFlag = 0; // Flag that is 1 for when the autofocus log is being written to, 0 when it is not

std::atomic<bool> bNewImage = 0; // Flag that is 1 for when the buffer image is new, 0 when buffer image is old

const long img_size = 1280 * 960; // Replace with actual image size
bool bSaveImages = 0;             // Saves images from the tilted camera to output folder (check filepath is right). WARNING: will produce enourmous number of images!
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
  if (bLensInit && bTiltedCamInit)
  {
    tAutofocus = std::thread(&autofocus::run, this); // starts a thread that executes autofocus::run()
    if (bAutofocusLogFlag)
    {
      logger->info("[autofocus::autofocus] autofocus::run thread started");
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
  
  if (bAutofocusLogFlag)
  {
    logger->info("[autofocus::~autofocus] destructor completed");
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

  // Scale the 1280*960 image to 640*480, or 320*240, etc.
  double scale = 0.5;

  // Start the camera capture thread
  logger->info("[autofocus::run] Starting camera capture thread");
  tiltedcam1.startCaptureThread();

  int moved = 1;
  int previous = desiredLocBestFocus; // TODO: should be in a mutex
  int tol = 6;                        // Tolerance zone of pixels about the desiredLocBestFocus where no lense movement is triggered. Lower than 4 causes constant signals to lens
  // Should be 4 for scale=0.5
  int blink = 0;        // Becomes 1 when a blink is detected
  int blinkframes = 15; // number of frames to ignore when blink is detected
  imgcount = 0;         // Keeps track of the number of images recieved and analyzed from the camera, but resets when switching between FindFocus and HoldFocus
  int imgcountfile = 0; // Keeps track of TOTAL number of images, for filenaming and debugging

  bHoldFocus = 0;

  int imWidth = tiltedcam1.getImageWidth();
  int imHeight = tiltedcam1.getImageHeight();

  //// PID CONTROLLER
  double dt = 1.0 / 60.0; // time per frame on the TILTED CAMERA! Assumes 60Hz.
  double max = 3;         // maximum relative move the lens can be ordered to make. Set to +-3mm
  double min = -3;
  double Kp = 0.0012;
  //double Kp = 0.006;
  double Ki = 0.0;
  double Kd = 0.00008;
  //double Kd = 0.0;
  PID pid = PID(dt, max, min, Kp, Kd, Ki);

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
        bNewImage = true;
        imgcount++;
        imgcountfile++;

        cv::Mat image = cv::Mat(imHeight, imWidth, CV_8U, img_calc_buf);
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(), scale, scale); // function is fast; negligible speed difference if placed in while loop. TODO: Replace with crop

        int locBestFocus = computeBestFocus(resized, resized.rows, resized.cols);

        //  Print image with line at the location of best focus
        if (bSaveImages)
        {
          // Draw vertical line at best focus position
          cv::Point p1(locBestFocus, 0), p2(locBestFocus, resized.rows);
          
          // Convert resized to color if it's grayscale to match the graph image type
          cv::Mat colorResized;
          if (resized.channels() == 1) {
            cv::cvtColor(resized, colorResized, cv::COLOR_GRAY2BGR);
          } else {
            colorResized = resized.clone();
          }
          
          // Draw on the color version
          cv::line(colorResized, p1, p2, cv::Scalar(0, 0, 255), 2);
          
          // Create a separate visualization for the curves
          int graphHeight = 150; // Height of the graph area
          cv::Mat graphImage(graphHeight, colorResized.cols, CV_8UC3, cv::Scalar(255, 255, 255));
          
          // Only draw the curves if we have data
          if (!lastSharpnessCurve.empty() && !lastFittedCurve.empty() && 
              lastSharpnessCurve.size() == lastFittedCurve.size()) {
            
            // Normalize curves for visualization
            double maxSharpness = *std::max_element(lastSharpnessCurve.begin(), lastSharpnessCurve.end());
            double minSharpness = *std::min_element(lastSharpnessCurve.begin(), lastSharpnessCurve.end());
            double range = maxSharpness - minSharpness;
            
            if (range > 0) {  // Prevent division by zero
              // Draw sharpness curve in blue
              for (size_t i = 0; i < lastSharpnessCurve.size() - 1 && i + 1 < graphImage.cols; i++) {
                double normalized1 = (lastSharpnessCurve[i] - minSharpness) / range;
                double normalized2 = (lastSharpnessCurve[i+1] - minSharpness) / range;
                
                cv::Point p1(i, graphHeight - normalized1 * (graphHeight - 20) - 10);
                cv::Point p2(i+1, graphHeight - normalized2 * (graphHeight - 20) - 10);
                
                // Make sure points are within image bounds
                p1.y = std::max(0, std::min(p1.y, graphHeight - 1));
                p2.y = std::max(0, std::min(p2.y, graphHeight - 1));
                
                cv::line(graphImage, p1, p2, cv::Scalar(255, 0, 0), 1);
              }
              
              // Draw fitted normal curve in green
              for (size_t i = 0; i < lastFittedCurve.size() - 1 && i + 1 < graphImage.cols; i++) {
                double normalized1 = (lastFittedCurve[i] - minSharpness) / range;
                double normalized2 = (lastFittedCurve[i+1] - minSharpness) / range;
                
                cv::Point p1(i, graphHeight - normalized1 * (graphHeight - 20) - 10);
                cv::Point p2(i+1, graphHeight - normalized2 * (graphHeight - 20) - 10);
                
                // Make sure points are within image bounds
                p1.y = std::max(0, std::min(p1.y, graphHeight - 1));
                p2.y = std::max(0, std::min(p2.y, graphHeight - 1));
                
                cv::line(graphImage, p1, p2, cv::Scalar(0, 255, 0), 1);
              }
            }
            
            // Draw vertical line at best focus position on graph
            if (locBestFocus < graphImage.cols) {
              cv::line(graphImage, cv::Point(locBestFocus, 0), cv::Point(locBestFocus, graphHeight), cv::Scalar(0, 0, 255), 2);
            }
            
            // Add legend
            cv::putText(graphImage, "Sharpness Curve", cv::Point(10, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
            cv::putText(graphImage, "Fitted Normal", cv::Point(10, 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
          }
          
          // Verify both matrices have the same width and type before concatenating
          if (colorResized.cols == graphImage.cols && colorResized.type() == graphImage.type()) {
            // Combine original image with graph
            cv::Mat combined;
            cv::vconcat(colorResized, graphImage, combined);
            
            // Save the combined image
            std::string FilePath = "../output/TiltedCam_Images/" + std::to_string(imgcountfile - 1) + "_" + std::to_string(locBestFocus) + ".png";
            cv::imwrite(FilePath, combined);
          } else {
            // Just save the original image if we can't combine
            std::string FilePath = "../output/TiltedCam_Images/" + std::to_string(imgcountfile - 1) + "_" + std::to_string(locBestFocus) + ".png";
            cv::imwrite(FilePath, colorResized);
          }
        };

        // If imgcount==1, then the user has just turned on FindFocus or HoldFocus
        if (imgcount == 1)
        {
          if (bHoldFocus)
          {
            desiredLocBestFocus = locBestFocus;
            previous = desiredLocBestFocus;
          }
          else if (bFindFocus)
          {
            desiredLocBestFocus = 180 * (scale / 0.25); // This was set when scale=0.25, so adjusting
            previous = 180 * (scale / 0.25);
            if (bAutofocusLogFlag)
            {
              logger->info("[autofocus::run] Set desiredLocBestFocus back to 140 in autofocus.cc");
            }
          }
        }

        //// BLINK DETECTION. TODO: try removing 'moved' variable
        else if (moved == 0 && blink == 0 && abs(locBestFocus - previous) > (50) && bBlinking)
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
          // do nothing inside tol band
          if (abs(locBestFocus - desiredLocBestFocus) <= tol)
          { // tol
            std::cout << ", in TOL band\n";
            moved = 0;
          }
          else
          { // outside tol band
            // PID CONTROLLER
            mmToMove = pid.calculate(desiredLocBestFocus, locBestFocus) * -1.0;
            // reduce mmToMove if near desiredLocBestFocus to prevent overshooting
            if (abs(locBestFocus - desiredLocBestFocus) < 50) {
              mmToMove = mmToMove * 0.5;
            }
            std::cout << locBestFocus << ", " << desiredLocBestFocus << ", " << mmToMove << "\n";
            bNewMoveRel = 1; // MoveRel lets the lens object keep track of the actual lens position
            moved = 1;


            // // FEEDFORWARD
            // // get actual lens position
            // double EPOS = lens1.getLensPosition();
            // int error = desiredLocBestFocus - locBestFocus;
            // double desiredEPOS = EPOS - (error * 0.00057);
            // lens1.setDesiredLensPosition(desiredEPOS);
            // std::cout << s_int.count() << ", " << desiredLocBestFocus << ", " << locBestFocus << ", " << EPOS << ", " << error << ", " << desiredEPOS << "\n";

            // calculate desired lens position
            // double currentLensPosition = lens1.getPosition();
            // double desiredLensPosition = currentLensPosition + error;
            // lens1.move(desiredLensPosition);







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
        previous = locBestFocus;
        t2 = std::chrono::steady_clock::now();
        s_int = s_int + std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
      }
    }
    else {
      // Reset counter when autofocus is not active
      imgcount = 0;
      
      // Sleep a bit to reduce CPU usage when not actively focusing
      usleep(50000); // 50ms
    }
  }
  
  // t2 = std::chrono::steady_clock::now();
  // s_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  // std::cout << " Analyzed " << imgcountfile << " images in " << s_int.count() << " milliseconds" << "\n";
  tiltedcam1.stopCaptureThread();

  // std::cout.rdbuf(original_cout);  // Redirect std::cout back to the console
  // out_file.close();
}

int autofocus::computeBestFocus(cv::Mat image, int imgHeight, int imgWidth)
{
  cv::Mat blurred;
  cv::GaussianBlur(image, blurred, cv::Size(3, 3), 1, 1, cv::BORDER_DEFAULT);

  ///// ROBERTS CROSS OVER THE WHOLE IMAGE ////
  cv::Mat temp_matrixx = (cv::Mat_<double>(2, 2) << 1, 0, 0, -1);
  cv::Mat temp_matrixy = (cv::Mat_<double>(2, 2) << 0, 1, -1, 0);

  // cv::UMat matrixx = temp_matrixx.getUMat(cv::ACCESS_READ);
  // cv::UMat matrixy = temp_matrixy.getUMat(cv::ACCESS_READ);

  // convolving imagedata with the matrices
  cv::Mat img_x, img_y;
  cv::filter2D(blurred, img_x, -1, temp_matrixx);
  cv::filter2D(blurred, img_y, -1, temp_matrixy);

  // squaring and summing the resultant matrices, which gives us the sharpness (or, gradient) for each pixel in imagedata, and taking the average score over the portion of image
  cv::Mat img_x_squared, img_y_squared;
  cv::multiply(img_x, img_x, img_x_squared);
  cv::multiply(img_y, img_y, img_y_squared);
  cv::Mat sharpness_image;
  cv::add(img_x_squared, img_y_squared, sharpness_image);

  // //// TENEGRAD OVER WHOLE IMAGE ///
  // cv::UMat Gx, Gy;
  // cv::Sobel(image, Gx, CV_64F, 1, 0, 3);
  // cv::Sobel(image, Gy, CV_64F, 0, 1, 3);
  // cv::UMat Gx2, Gy2;
  // cv::multiply(Gx, Gx, Gx2);
  // cv::multiply(Gy, Gy, Gy2);
  // cv::UMat sharpness_image;
  // cv::add(Gx2, Gy2, sharpness_image);

  // ///// ROI'ING ////
  std::vector<double> sharpnesscurve;
  int kernel = 16; // must be an even number

  for (int i = 0; i < imgWidth - kernel; i++)
  {
    cv::Rect roi(i, 0, kernel, imgHeight);
    cv::Mat regionSharpnessImage = sharpness_image(roi);
    cv::Scalar regionSharpness = cv::mean(regionSharpnessImage);
    double regionSharpnessScore = regionSharpness[0];
    sharpnesscurve.push_back(regionSharpnessScore);
  }

  // Fitting a normal curve to the sharpness curve, to avoid local peaks in the data around vessel edges
  std::vector<double> sharpnesscurvenormalized = fitnormalcurve(sharpnesscurve, kernel);

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
  
  // Store the curves for visualization
  lastSharpnessCurve = sharpnesscurve;
  lastFittedCurve = sharpnesscurvenormalized;
  
  return locBestFocus + kernel / 2;
}

std::vector<double> autofocus::computesharpness(cv::Mat image, int imgHeight, int imgWidth, int kernel)
{
  // vector<int> grad;
  // vector<int> grad_canny;
  // vector<int> grad_vollath;
  std::vector<double> sharpnesscurve;

  for (int i = 0; i < imgWidth - kernel; i++)
  {
    // gets the region of interest, a rectangle of 'kernel' width for each pixel along the width of the image
    cv::Rect roi(i, 0, kernel, imgHeight);
    cv::Mat imageofinterest = image(roi);
    // Mat pixels = img(cv::Rect(0, 0, kernel_width, h)).clone(); (another method of finding submatrix)

    cv::Scalar sharpness = tenengrad(imageofinterest); // doesn't matter if tenegrad or robertscross!!!!
    double sharpnessscore = cv::sum(sharpness)[0];
    sharpnesscurve.push_back(sharpnessscore);
  }
  return sharpnesscurve;
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

std::vector<double> autofocus::fitnormalcurve(std::vector<double> sharpnesscurve, int kernel, double std_dev_factor)
{
  // Fits a normal curve to the data. Should try to find a proper curve-fitting library...
  std::vector<double> norm_curve;
  std::vector<double> sum_of_diffs;
  double std_dev = std_dev_factor * (sharpnesscurve.size()); // now uses the parameter

  double amplitude = (*max_element(begin(sharpnesscurve), end(sharpnesscurve)) - *min_element(begin(sharpnesscurve), end(sharpnesscurve)));
  double offset = *min_element(begin(sharpnesscurve), end(sharpnesscurve));

  // Calculates sum_of_elems, a vector showing how well each normal curve with mean j fits grad
  for (int j = 1; j <= sharpnesscurve.size(); j++)
  {

    // Generates normal curve for given mean j
    std::vector<double> norm_curve_temp;
    for (int i = 0; i < sharpnesscurve.size(); i++)
    {
      //// creates sin curve
      // double sinResult = amplitude * sin(2 * M_PI * (0.5/275) * (i - j + 152) )  + offset;
      // if (sinResult < offset) { sinResult = offset; }
      double normResult = normpdf(i, j, std_dev);
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
  double mean = distance(begin(sum_of_diffs), min_element(begin(sum_of_diffs), end(sum_of_diffs)));

  // Generates final norm_curve
  for (int i = 0; i < sharpnesscurve.size(); i++)
  {
    //// creates sin curve
    // double sinResult = amplitude * sin(2 * M_PI * (0.5/275) * (i - j + 152) )  + offset;
    // if (sinResult < offset) { sinResult = offset; }
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
