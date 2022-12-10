#include "autofocus.h"
#include "tiltedcam.h"
#include "lens.h"
#include "pid.h"

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

//Global variables 
std::atomic<bool> bAutofocusing = 0; //Flag that controls the autofocusing while() loop
std::atomic<bool> bNewImage = 0; //Flag that is 1 for when the buffer image is new, 0 when buffer image is old

bool bBestFocusFromFirstImage = 0; //Flag for whether you are finding or holding focus; later controlled by GUI

const long img_size = 640 * 480; //Replace with actual image size

unsigned char* img_buf = (unsigned char*)malloc(img_size); // Accessed by thread1 and thread2
unsigned char* img_get_buf = (unsigned char*)malloc(img_size); // Used by by image pulling thread
unsigned char* img_calc_buf = (unsigned char*)malloc(img_size); // Used by image analysis thread
std::mutex mtx; 

void autofocus::run () {
  //functions to execute when the hold focus button is clicked 

  tiltedcam tiltedcam; 
  lens lens; //Create a lens object


  //Should this happen when the tiltedcam object is initialized? (ie as a constructor?)
  if(!tiltedcam.initcam()) {
    std::cout << "failure to initalize tilted camera";
  } 
  tiltedcam::settings camsettings = tiltedcam.getsettings();

  //Should this happen when the motor object is initialized? (ie. as a constructor?)
  int PortToUse = 3; 
  if(!lens.initmotor(PortToUse)) {
    std::cout << "failure to initalize lens motor";
  } 

  // For testing, bAutofocusing is set here and flagged using time_autofocusing_s. Later, it will be controlled by Focus_Button
  bAutofocusing = 1; 
  double time_autofocusing_s;
  std::cout << "How long should the code run for (s)?\n";
  std::cin >> time_autofocusing_s;
  auto t1 = std::chrono::steady_clock::now();
  usleep(10000);
  auto t2 = std::chrono::steady_clock::now();
  auto s_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

  //Starting the capture video thread
  std::thread tCaptureVideo(autofocus::capturevideo);
  
  int testcount = 1;

  //The following variables assume imgWidth = 320; must be changed if this isn't the case.
  int center = 160; //Should be optimized for max. contrast 
  int moved = 1;
  int previous = center;
  int tol = 6; //Tolerance zone of pixels in the center of image where no lense movement is triggered

  int blink = 0; //Becomes 1 when a blink is detected
  int blinkframes = 15; //number of frames to ignore when blink is detected
  int imgcount = 0; //Keeps track of the number of images recieved and analyzed from the camera

  //// PID CONTROLLER
  double dt = 1.0 / 50.0; //time per frame. Assumes about 50Hz... could be set exactly inside the while loop
  double max = 3;  //maximum relative move the lens can be ordered to make. Set to +-3mm
  double min = -3; 
  double Kp = 0.0018 * 2; //From experimental testing; and doubled since we are using actual pixel width, not 640 width
  double Ki = 0;
  double Kd = 0; //Should try to add a small Kd; further testing required
  PID pid = PID(dt, max, min, Kp, Kd, Ki);
  
  while(bAutofocusing) {
    if (bNewImage) {
      { 
      std::lock_guard<std::mutex> lck{ mtx };
      img_calc_buf = img_buf;
      }
      bNewImage = 0;
      imgcount++;

      cv::Mat image = cv::Mat(camsettings.imgHeight, camsettings.imgWidth, CV_8U, img_calc_buf);
      cv::Mat resized;
      double scale = 0.5; //change to 0.5 to resize image to (0.5*current dimensions). INSTEAD, TRUNCATE IMAGE, OR REDUCE KERNEL!
      cv::resize(image, resized, cv::Size(), scale, scale); //function is fast; negligible speed difference if placed in while loop
      int locBestFocus = computebestfocus(resized, resized.rows, resized.cols);
            
      // //Print image with line at the location of best focus
      // cv::Point p1(locBestFocus, 0), p2(locBestFocus, resized.cols);
      // cv::line(resized, p1, p2, cv::Scalar(0, 0, 255), 2);
      // cv::namedWindow("Image With Loc of Best-Focus", cv::WINDOW_AUTOSIZE);
      // cv::imshow("Image With Loc of Best-Focus", resized);
      // cv::waitKey(0);

      // Past here could become a separate thread which takes the most updated locBestFocus and moves the motor
      
      // If holding focus, center becomes the current location of best-focus
      if (imgcount == 1) {
          if (bBestFocusFromFirstImage) {
              center = locBestFocus;
              previous = center;
          }
      }

      //// BLINK DETECTION. TODO: try removing 'moved' variable
      else if (moved == 0 && blink == 0 && abs(locBestFocus - previous) > (50)) { // if the location of best focus changes by more than 50 pixels with no move, it is a blink
          //Blink starts
          std::cout << "Frame ignored; blink detected\n";
          blink = 1;
      }
      else if (blink == 1) {
          std::cout << "Frame ignored; blink detected\n";
          blinkframes--;
          if (blinkframes == 0) {
              blinkframes = 15; //resets blinkframes
              blink = 0;
          }
      }


      else {
          //do nothing inside tol band
          if (abs(locBestFocus - center) <= tol) { // tol
              std::cout << s_int.count() << " ms, ";
              std::cout << "0\n";
              moved = 0;
          }

          else {
              // PID CONTROLLER
              double inc = pid.calculate(center, locBestFocus);
              inc = inc * -1.0;
              lens.mov_rel(inc);
              std::cout << s_int.count() << ", ";
              std::cout << inc << "\n";
              moved = 1;
          }
      }
      previous = locBestFocus;

    }
    t2 = std::chrono::steady_clock::now();
    s_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    if (s_int.count() > (time_autofocusing_s*1000)) {
        bAutofocusing = 0; // breaks loop when code runs longer than duration
        tCaptureVideo.join(); // blocks execution until CaptureVideo finishes
    }
  
  }
  std::cout << imgcount << "\n";
  
} 

void autofocus::capturevideo() {
  while(bAutofocusing) {

    //for testing
    cv::Mat image;
    image = cv::imread("/home/tom/projects/autofocus_v9/test/NoLine/img1748_294.png", 1 );
    if ( !image.data )
    {
        std::cout << "No image data \n";
    }

    {
    std::lock_guard<std::mutex> lck{ mtx };
    img_buf = image.data;
    }
    bNewImage = 1;
    //std::cout << "got an image" << "\n";
    usleep(16670); //is in microseconds
  }
}

int autofocus::computebestfocus (cv::Mat image, int imgHeight, int imgWidth) {
  //Specular reflection rejection
  cv::Mat thresh;
  cv::threshold(image, thresh, 140, 255, cv::THRESH_TRUNC);

  //Gaussian blurring
  cv::Mat blurred;
  cv::GaussianBlur(thresh, blurred, cv::Size(3,3),1,1,cv::BORDER_DEFAULT);

  //Computing the sharpness curve along the horizontal of the image 
  int kernel = 16; //must be an even number
  std::vector<double> sharpnesscurve = computesharpness(blurred, imgHeight, imgWidth, kernel);
  
  //Fitting a normal curve to the sharpness curve, to avoid local peaks in the data around vessel edges
  std::vector<double> sharpnesscurvenormalized = fitnormalcurve(sharpnesscurve, kernel);

//   //printing to text files for testing
//   std::string FileName = "null";
//   std::cout << "File name:\n";
//   std::cin >> FileName;
//   std::string TextFile = "/home/tom/projects/autofocus_v9/test/" + FileName + "_SharpnessCurve.txt";
//   std::string TextFile2 = "/home/tom/projects/autofocus_v9/test/" + FileName + "_FittedNorm.txt";
//   std::ofstream outputFile(TextFile);
//   std::ofstream outputFile2(TextFile2);
//   std::ostream_iterator<double> output_iterator(outputFile, ", ");
//   std::copy(sharpnesscurve.begin(), sharpnesscurve.end(), output_iterator);
//   outputFile << "\n";
//   std::ostream_iterator<double> output_iterator2(outputFile2, ", ");
//   std::copy(sharpnesscurvenormalized.begin(), sharpnesscurvenormalized.end(), output_iterator2);
//   outputFile2 << "\n";
//   outputFile.close();
//   outputFile2.close();

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

      cv::Scalar sharpness = robertscross(imageofinterest);
      double sharpnessscore = cv::sum(sharpness)[0];
      sharpnesscurve.push_back(sharpnessscore);
  }
return sharpnesscurve;
}


cv::Scalar autofocus::robertscross(cv::Mat imagedata) {
  // Creating Roberts Cross matrices
  cv::Mat matrixx = (cv::Mat_<double>(2, 2) << 1, 0, 0, -1);
  cv::Mat matrixy = (cv::Mat_<double>(2, 2) << 0, 1, -1, 0);

  //convolving imagedata with the matrices
  cv::Mat img_x, img_y;
  cv::filter2D(imagedata, img_x, -1, matrixx);
  cv::filter2D(imagedata, img_y, -1, matrixy);

  //squaring and summing the resultant matrices, which gives us the sharpness (or, gradient) for each pixel in imagedata, and taking the average score over the portion of image
  cv::Scalar mean_sharpness = mean(img_x.mul(img_x) + img_y.mul(img_y));
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

    double std_dev = 56.0 * (sharpnesscurve.size() / 304.0); //determined experimentally using 304-length data, and approximately scaled for larger images. Needs to be replaced! (was 58)
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