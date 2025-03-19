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
//#include <gtk/gtk.h>

#include "opencv2/highgui/highgui.hpp"

//Global variables 
bool bAutofocusLogFlag = 0; //Flag that is 1 for when the autofocus log is being written to, 0 when it is not

std::atomic<bool> bNewImage = 1; //Flag that is 1 for when the buffer image is new, 0 when buffer image is old

const long img_size = 1280 * 960; //Replace with actual image size
bool bSaveImages = 0; // Saves images from the tilted camera to output folder (check filepath is right). WARNING: will produce enourmous number of images!
bool bSaveSharpnessCurves = 0; // Saves text files with the sharpness curve data, similar to above

bool bBlinking = 0;

unsigned char* img_buf = (unsigned char*)malloc(img_size); // Accessed by thread1 and thread2
unsigned char* img_get_buf = (unsigned char*)malloc(img_size); // Used by by image pulling thread
unsigned char* img_calc_buf = (unsigned char*)malloc(img_size); // Used by image analysis thread
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

autofocus::autofocus() :
  lens1(), 
  tiltedcam1(),  
  stop_thread(false)
{}

bool autofocus::initialize() {
  bool bLensInit = lens1.initialize();
  bool bTiltedCamInit = tiltedcam1.initialize();
  std::cout << "bLensInit: " << bLensInit << " bTiltedCamInit: " << bTiltedCamInit << "\n";
  if(bLensInit && bTiltedCamInit) {
      tAutofocus = std::thread(&autofocus::run, this); // starts a thread that executes autofocus::run()
      if(bAutofocusLogFlag) {logger->info("[autofocus::autofocus] autofocus::run thread started");}
      return true;
  }
  else {
      return false;
  }
}

autofocus::~autofocus() { 
  // Stops the autofocus thread
  stop_thread.store(true);
  if(tAutofocus.joinable()) {
      tAutofocus.join();
  }
  if(bAutofocusLogFlag) {logger->info("[autofocus::~autofocus] destructor completed");}
}

void autofocus::run () {

  // COMMENT THIS OUT!
  std::ofstream out_file("/home/hvi/Desktop/HVI-data/output.txt");
  std::streambuf* original_cout = std::cout.rdbuf();  // Save the buffer of std::cout
  std::cout.rdbuf(out_file.rdbuf());                  // Redirect std::cout to the file

  auto t1 = std::chrono::steady_clock::now();
  usleep(10000);
  auto t2 = std::chrono::steady_clock::now();
  auto s_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  
  //For detecting oscillations
  std::deque<int> locBestFocusHistory; // use deque for easy push/pop at front and back

  //Scale the 1280*960 image to 640*480, or 320*240, etc.
  double scale = 0.5; 

  //Starting the capture video thread
  std::thread tCaptureVideo(&autofocus::capturevideo, this);

  //The following variables assume imgWidth = 320; must be changed if this isn't the case.
  int moved = 1;
  int previous = desiredLocBestFocus; //TODO: should be in a mutex
  int tol = 4 * scale; //Tolerance zone of pixels about the desiredLocBestFocus where no lense movement is triggered. Lower than 4 causes constant signals to lens
  //Should be 4 for scale=0.5
  int blink = 0; //Becomes 1 when a blink is detected
  int blinkframes = 15; //number of frames to ignore when blink is detected
  imgcount = 0; //Keeps track of the number of images recieved and analyzed from the camera, but resets when switching between FindFocus and HoldFocus
  int imgcountfile = 0; //Keeps track of TOTAL number of images, for filenaming and debugging

  bHoldFocus = 0;

  int imWidth = tiltedcam1.getImageWidth();
  int imHeight = tiltedcam1.getImageHeight();

  //// PID CONTROLLER
  // Tuning: found K_cr = 0.004, T_cr = 0.0524s, so using Kp = 0.0024, Ki = 0.0916, Kd = 0.00001572
  double dt = 1.0 / 60.0; //time per frame on the TILTED CAMERA! Assumes 60Hz. TODO
  double max = 3;  //maximum relative move the lens can be ordered to make. Set to +-3mm
  double min = -3; 
  //double Kp = 0.0018 * 3.0; //*4-5.0 is on the edge of instability; *3.0 seems stable
  //double Kp = 0.004;
  double Kp = 0.0015;
  double Ki = 0.0;
  double Kd = 0.0; //Should try to add a small Kd; further testing required
  PID pid = PID(dt, max, min, Kp, Kd, Ki);  

  if(bAutofocusLogFlag) {logger->info("[autofocus::run] while thread loop about to start");}
  while(!stop_thread.load()) {
    

    if (bNewImage && (bHoldFocus || bFindFocus)) {

      t1 = std::chrono::steady_clock::now();
      
      { 
      std::lock_guard<std::mutex> lck{ mtx };
      img_calc_buf = img_buf;
      }
      bNewImage = 0;
      imgcount++;
      imgcountfile++;

      // cv::Mat image = cv::Mat(imHeight, imWidth, CV_8U, img_calc_buf);
      // cv::Mat resized;
      // double scale = 1;
      // cv::resize(image, resized, cv::Size(), scale, scale); //function is fast; negligible speed difference if placed in while loop. TODO: Replace with crop

      // Same as above but OpenCL-friendly
      cv::Mat image = cv::Mat(imHeight, imWidth, CV_8U, img_calc_buf);
      //cv::UMat image = temp_image.getUMat(cv::ACCESS_READ);
      cv::Mat resized;
      cv::resize(image, resized, cv::Size(), scale, scale); //function is fast; negligible speed difference if placed in while loop. TODO: Replace with crop

      //int locBestFocus = computebestfocus(resized, resized.rows, resized.cols); //drops to 0.5 fps if placed in while loop
      int locBestFocus = computebestfocusReversed(resized, resized.rows, resized.cols);
      //std::cout << locBestFocus << ", ";

      //int locBestFocus = 220; //this is fast AF, like 960 fps
      // std::cout << locBestFocus << ", ";
      // std::cout << locBestFocusReversed << ", ";
      // std::cout << locBestFocus - locBestFocusReversed << ", ";
      // //Print image with line at the location of best focus
        cv::Point p1(locBestFocus, 0), p2(locBestFocus, resized.cols);
        cv::line(resized, p1, p2, cv::Scalar(0, 0, 255), 2);
        //cv::namedWindow("Image With Loc of Best-Focus", cv::WINDOW_AUTOSIZE);
        //cv::imshow("Image With Loc of Best-Focus", resized);
        std::string FilePath = "/home/hvi/Desktop/TiltedCam-Output/" + std::to_string(imgcountfile - 1) + "_" + std::to_string(locBestFocus) + ".png";
        if (bSaveImages) {cv::imwrite(FilePath, resized);};
        //cv::waitKey(0);

      std::cout << s_int.count() << ", " << desiredLocBestFocus << ", " << locBestFocus << ", ";


      //If imgcount==1, then the user has just turned on FindFocus or HoldFocus
      if (imgcount == 1) {
          if (bHoldFocus) {
            desiredLocBestFocus = locBestFocus;
            previous = desiredLocBestFocus;
            //window.setBestFocusScaleValue(desiredLocBestFocus); //set the slider to be equal to the current location of best-focus
          }
          else if (bFindFocus) {
            desiredLocBestFocus = 140 * (scale/0.25); //This was set when scale=0.25, so adjusting
            previous = 140 * (scale/0.25);
            if(bAutofocusLogFlag) {logger->info("[autofocus::run] Set desiredLocBestFocus back to 140 in autofocus.cc");}
            //window.setBestFocusScaleValue(desiredLocBestFocus); //set the slider to be equal to 160, the center-point
          }
      }

      //// BLINK DETECTION. TODO: try removing 'moved' variable
      else if (moved == 0 && blink == 0 && abs(locBestFocus - previous) > (50) && bBlinking) { // if the location of best focus changes by more than 50 pixels with no move, it is a blink
          //Blink starts
          if(bAutofocusLogFlag) logger->info("[autofocus::run] Frame ignored; blink detected");
          blink = 1;
      }
      else if (blink == 1) {
          if(bAutofocusLogFlag) logger->info("[autofocus::run] Frame ignored; blink detected");
          blinkframes--;
          if (blinkframes == 0) {
              blinkframes = 15; //resets blinkframes
              blink = 0;
          }
      }
      

      else {
          //do nothing inside tol band
          if (abs(locBestFocus - desiredLocBestFocus) <= tol) { // tol
            std::cout << ", in TOL band\n";
            moved = 0;

            locBestFocusHistory.clear(); //When a value is inside the tolerance band, focus has been found, so clear the history
          }
          else { //outside tol band
            // PID CONTROLLER with TOL band removed
            if(locBestFocus - desiredLocBestFocus > 0) {
              mmToMove = pid.calculate(desiredLocBestFocus + tol, locBestFocus) * -1.0;
              std::cout << mmToMove << "\n";
            }
            else if(locBestFocus - desiredLocBestFocus < 0) {
              mmToMove = pid.calculate(desiredLocBestFocus - tol, locBestFocus) * -1.0;
              std::cout << mmToMove << "\n";
            }              
            bNewMoveRel = 1;
            moved = 1;
            
            //// OSCILLATION DETECTION
            //Adding to locBestFocusHistory when outside TOL band
            locBestFocusHistory.push_back(locBestFocus);
            // If we have more than 20 values, remove the oldest
            if (locBestFocusHistory.size() > 20) {
              locBestFocusHistory.pop_front();
            }
            // Check if history contains values both above and below desiredLocBestFocus
            if(locBestFocusHistory.size() == 20) {
              bool hasAbove = std::any_of(locBestFocusHistory.begin(), locBestFocusHistory.end(), [](int x) { return x > desiredLocBestFocus; });
              bool hasBelow = std::any_of(locBestFocusHistory.begin(), locBestFocusHistory.end(), [](int x) { return x < desiredLocBestFocus; });              if (hasAbove && hasBelow) {
                // Find the value closest to desiredLocBestFocus
                auto closestIt = std::min_element(locBestFocusHistory.begin(), locBestFocusHistory.end(), [](int a, int b) {
                    return std::abs(a - desiredLocBestFocus) < std::abs(b - desiredLocBestFocus);
                });
                desiredLocBestFocus = *closestIt;
                std::cout << "DETECTED OSCILLATION!! Adjusting desiredLocBestFocus to " << desiredLocBestFocus << "\n";
                locBestFocusHistory.clear(); // Clear history after adjusting desiredLocBestFocus
              }
            }
          }
        } 
      previous = locBestFocus;
      t2 = std::chrono::steady_clock::now();
      s_int = s_int + std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    }
    // t2 = std::chrono::steady_clock::now();
    // s_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    // if (s_int.count() > (time_autofocusing_s*1000)) {
    // }
  
  }
  // t2 = std::chrono::steady_clock::now();
  // s_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  // std::cout << " Analyzed " << imgcountfile << " images in " << s_int.count() << " milliseconds" << "\n";
  tCaptureVideo.join(); // Stops the CaptureVideo thread too

  // std::cout.rdbuf(original_cout);  // Redirect std::cout back to the console
  // out_file.close();
} 

//TODO: This should be in tiltedcam.cc, but I need to use the global variables... Poor code!
void autofocus::capturevideo() {
  if(bAutofocusLogFlag) {logger->info("[autofocus::capturevideo] thread started");}
  while(!stop_thread.load()) {

    //for testing
    // cv::Mat image;
    // image = cv::imread("/home/tom/projects/autofocus_v9/test/NoLine/img1748_294.png", 1 );
    // if ( !image.data )
    // {
    //     std::cout << "No image data \n";
    // }

    //Actually pulling from camera. TODO: get img_size from camera, not from global variables... 
     if (ASIGetVideoData(0, img_get_buf, img_size, -1) != ASI_SUCCESS)
     {
         logger->error("[autofocus::capturevideo] Error getting image from tilted camera!");
     }

    
    //TODO: replace with a temp storage queue
    {
    std::lock_guard<std::mutex> lck{ mtx };
    //img_buf = image.data;
    img_buf = img_get_buf;
    }
    bNewImage = 1;
    //std::cout << "got an image" << "\n";
    //usleep(16670); //is in microseconds
  }
}

int autofocus::computebestfocusReversed (cv::Mat image, int imgHeight, int imgWidth) {
  cv::Mat blurred;
  cv::GaussianBlur(image, blurred, cv::Size(3,3),1,1,cv::BORDER_DEFAULT);

  ///// ROBERTS CROSS OVER THE WHOLE IMAGE ////
  cv::Mat temp_matrixx = (cv::Mat_<double>(2, 2) << 1, 0, 0, -1);
  cv::Mat temp_matrixy = (cv::Mat_<double>(2, 2) << 0, 1, -1, 0);
  
  // cv::UMat matrixx = temp_matrixx.getUMat(cv::ACCESS_READ);
  // cv::UMat matrixy = temp_matrixy.getUMat(cv::ACCESS_READ);

  //convolving imagedata with the matrices
  cv::Mat img_x, img_y;  
  cv::filter2D(blurred, img_x, -1, temp_matrixx);
  cv::filter2D(blurred, img_y, -1, temp_matrixy);

  //squaring and summing the resultant matrices, which gives us the sharpness (or, gradient) for each pixel in imagedata, and taking the average score over the portion of image
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
  int kernel = 16; //must be an even number

  for (int i = 0; i < imgWidth - kernel; i++) {
    cv::Rect roi(i, 0, kernel, imgHeight);
    cv::Mat regionSharpnessImage = sharpness_image(roi);
    cv::Scalar regionSharpness = cv::mean(regionSharpnessImage);
    double regionSharpnessScore = regionSharpness[0];
    sharpnesscurve.push_back(regionSharpnessScore);
  }
  
  //Fitting a normal curve to the sharpness curve, to avoid local peaks in the data around vessel edges
  std::vector<double> sharpnesscurvenormalized = fitnormalcurve(sharpnesscurve, kernel);

  //printing to text files for testing
  if(bSaveSharpnessCurves) {
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

  int locBestFocus = distance( begin(sharpnesscurvenormalized), max_element(begin(sharpnesscurvenormalized), end(sharpnesscurvenormalized)));
  return locBestFocus + kernel/2; 
}

int autofocus::computebestfocus (cv::Mat image, int imgHeight, int imgWidth) {
  // //Specular reflection rejection
  // cv::Mat thresh;
  // cv::threshold(image, thresh, 140, 255, cv::THRESH_TRUNC);

  //Gaussian blurring, very little differences
  cv::Mat blurred;
  cv::GaussianBlur(image, blurred, cv::Size(3,3),1,1,cv::BORDER_DEFAULT);

  //Computing the sharpness curve along the horizontal of the image 
  int kernel = 16; //must be an even number
  std::vector<double> sharpnesscurve = computesharpness(image, imgHeight, imgWidth, kernel);
  
  //Fitting a normal curve to the sharpness curve, to avoid local peaks in the data around vessel edges
  std::vector<double> sharpnesscurvenormalized = fitnormalcurve(sharpnesscurve, kernel);


  int locBestFocus = distance( begin(sharpnesscurvenormalized), max_element(begin(sharpnesscurvenormalized), end(sharpnesscurvenormalized)));
  return locBestFocus + kernel/2; 
}


std::vector<double> autofocus::computesharpness(cv::Mat image, int imgHeight, int imgWidth, int kernel) {
  //vector<int> grad;
  //vector<int> grad_canny;
  //vector<int> grad_vollath;
  std::vector<double> sharpnesscurve;

  for (int i = 0; i < imgWidth - kernel; i++) {
    //gets the region of interest, a rectangle of 'kernel' width for each pixel along the width of the image
    cv::Rect roi(i, 0, kernel, imgHeight); 
    cv::Mat imageofinterest = image(roi); 
    //Mat pixels = img(cv::Rect(0, 0, kernel_width, h)).clone(); (another method of finding submatrix)

    cv::Scalar sharpness = tenengrad(imageofinterest); //doesn't matter if tenegrad or robertscross!!!!
    double sharpnessscore = cv::sum(sharpness)[0];
    sharpnesscurve.push_back(sharpnessscore);
  }
return sharpnesscurve;
}



cv::Scalar autofocus::robertscross(cv::Mat imagedata) {
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

  //convolving imagedata with the matrices. WARNING, REPLACE TEMP_ WITH ACTUAL
  cv::Mat img_x, img_y;
  cv::filter2D(imagedata, img_x, -1, temp_matrixx);
  cv::filter2D(imagedata, img_y, -1, temp_matrixy);

  //squaring and summing the resultant matrices, which gives us the sharpness (or, gradient) for each pixel in imagedata, and taking the average score over the portion of image
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
    //Mat Gxy;
    //Sobel(img, Gxy, CV_64F, 1, 1, 3);
    //Scalar mean_grad = mean(abs(Gxy));
    return mean_grad;
}
cv::Scalar autofocus::vollath(cv::Mat img) {
    //Vollath's F4
    int sum1 = 0;
    int sum2 = 0;
    int h = img.rows; // 480
    int w = img.cols; // 16
    for (int i = 0; i < h - 1; i++) {
        for (int j = 0; j < w; j++) {
            sum1 = sum1 + static_cast<int>(img.at<uchar>(i, j)) * static_cast<int>(img.at<uchar>(i + 1, j)); //might need to flip i and j
        }
    }
    for (int i = 0; i < h - 2; i++) {
        for (int j = 0; j < w; j++) {
            sum2 = sum2 + static_cast<int>(img.at<uchar>(i, j)) * static_cast<int>(img.at<uchar>(i + 2, j));
        }
    }
    cv::Scalar mean_grad_vol = sum1 - sum2;
    return mean_grad_vol;
}
cv::Scalar autofocus::canny(cv::Mat img) {
    //Canny //
    cv::Mat edges;
    cv::Canny(img, edges, 25, 55, 3);
    cv::Scalar mean_grad_canny = mean(edges.mul(edges));
    return mean_grad_canny;
}


std::vector<double> autofocus::fitnormalcurve(std::vector<double> sharpnesscurve, int kernel) {
    // Fits a normal curve to the data. Should try to find a proper curve-fitting library... 
    std::vector<double> norm_curve;
    std::vector<double> sum_of_diffs;
    double std_dev = 70.0 * (sharpnesscurve.size() / 400.0); //determined experimentally using 304-length data, and approximately scaled for larger images. TODO: Tune properly!
    //double std_dev = 56.0 * (sharpnesscurve.size() / 304.0); //determined experimentally using 304-length data, and approximately scaled for larger images. Needs to be replaced! (was 58)
    double amplitude = (*max_element(begin(sharpnesscurve), end(sharpnesscurve)) - *min_element(begin(sharpnesscurve), end(sharpnesscurve))); 
    double offset = *min_element(begin(sharpnesscurve), end(sharpnesscurve));

    //Calculates sum_of_elems, a vector showing how well each normal curve with mean j fits grad
    for (int j = 1; j <= sharpnesscurve.size(); j++) {

        //Generates normal curve for given mean j
        std::vector<double> norm_curve_temp;
        for (int i = 0; i < sharpnesscurve.size(); i++) {
            //// creates sin curve
            //double sinResult = amplitude * sin(2 * M_PI * (0.5/275) * (i - j + 152) )  + offset;
            //if (sinResult < offset) { sinResult = offset; }
            double normResult = normpdf(i, j, std_dev);
            norm_curve_temp.push_back(normResult);
        }
        //scales so the max value (when i=j) is equal to the desired amplitude
        double factor = 1.0 / *max_element(begin(norm_curve_temp), end(norm_curve_temp));
        for (int i = 0; i < sharpnesscurve.size(); i++) {
            norm_curve_temp[i] = norm_curve_temp[i] * factor * amplitude + offset;
        }

        //Calculates difference between norm curve and sharpness curve            
        std::vector<double> differences;
        for (int z = 0; z < sharpnesscurve.size(); z++) {
            double diff = abs(sharpnesscurve[z] - norm_curve_temp[z]);
            differences.push_back(diff);
        }
        double sum_of_diff = std::accumulate(differences.begin(), differences.end(), decltype(differences)::value_type(0));
        sum_of_diffs.push_back(sum_of_diff);
    }

    //Picks the mean that has the lowest overall difference
    double mean = distance(begin(sum_of_diffs), min_element(begin(sum_of_diffs), end(sum_of_diffs)));

    //Generates final norm_curve
    for (int i = 0; i < sharpnesscurve.size(); i++) {
        //// creates sin curve
        //double sinResult = amplitude * sin(2 * M_PI * (0.5/275) * (i - j + 152) )  + offset;
        //if (sinResult < offset) { sinResult = offset; }
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

double autofocus::normpdf(double x, double u, double s) {
  const double ONE_OVER_SQRT_2PI = 0.39894228040143267793994605993438;
  return (ONE_OVER_SQRT_2PI / s) * exp(-0.5 * std::pow(((x - u) / s),2));
}

void autofocus::adjust_bestFocus(int val) {
  //TODO: should be in mutex
  desiredLocBestFocus = val;

}

