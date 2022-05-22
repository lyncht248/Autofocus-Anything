#include <iostream>
#include <tuple>
#include "C:\opencv-4.2.0\opencv\build\include\opencv2\opencv.hpp"
#include <windows.h>
//#include "plplot\plstream.h"
//#include <C:\Users\William\Desktop\Cambridge\IIB\IIB Project\c++ code\OpenCVTest\opencv2\opencv.hpp>
//#include <C:\Users\William\Desktop\Cambridge\IIB\IIB Project\c++ code\OpenCVTest\opencv2\build\include\opencv2\opencv.hpp>
#include <vector>
#include <valarray>
#include <chrono>
//#include "matplotlibcpp.h"
#include "autofocus.h"
#include <fstream>
//namespace plt = matplotlibcpp;
using namespace cv;
using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;


//std::tuple<int, int> img_size(Mat img) {
    //int h = img.rows;
    //int w = img.cols;
    //return std::make_tuple(h, w);
//}

Scalar tenengrad(Mat img) {
    Mat Gx, Gy;
    Sobel(img, Gx, CV_64F, 1, 0, 3);
    Sobel(img, Gy, CV_64F, 0, 1, 3);
    Scalar mean_grad = mean(Gx.mul(Gx) + Gy.mul(Gy));
    return mean_grad;
}

vector<int> compute_tene_grad(Mat img, int size) {
    int w = img.cols;
    int h = img.rows;
    int kernel_width = size;
    //Mat mask = Mat::zeros(Size(h,w), CV_64FC(1));
    //std::fill(h, w, 0);

    vector<int> grad;
    for (int i = 0; i <w-kernel_width; i++) {
        
        Rect roi(i, 0, kernel_width, h); // region of interest for computing sharpness
        Mat pixels = img(roi);
        //Mat pixels = img(cv::Rect(0, 0, kernel_width, h)).clone(); (another method of finding submatrix)
        Scalar check = tenengrad(pixels);
        int check1 = sum(check)[0];
        grad.push_back(check1);
        
    }
    
    auto focus = distance( begin(grad), max_element(begin(grad), end(grad)));
    grad.push_back(focus); //place location of max focus at end of grad
    return grad;

}

int sharpness(Mat img, double scale, ofstream &outputFile) {
    // This code resizes the image to 1/2, then multiplies the final value by 2

    int height = round(img.rows * scale);
    int width = round(img.cols * scale);
    Mat resized;
    resize(img, resized, Size(width, height), scale, scale, INTER_AREA);


    //Mat blurred;
    //GaussianBlur(resized, blurred, Size(3,3),1,1,BORDER_DEFAULT);


    //Mat equal;
    //equalizeHist(blurred, equal);


    vector<int> vec = compute_tene_grad(resized, 16);

    //WORKING ON THIS LINE
    std::ostream_iterator<int> output_iterator(outputFile, "\n");
    std::copy(vec.begin(), vec.end(), output_iterator);

    //cv::Point p1(vec[vec.size() - 1] / scale, 0), p2(vec[vec.size() - 1] / scale, 979);
    //cv::line(resized, p1, p2, cv::Scalar(0, 0, 255), 2);

    //cv::imshow("Window1", resized);
    //cv::waitKey(1);


    return vec[vec.size() -1]/ scale; //get last value from vector (This is location of max focus)


}

// int main(void) {
//     Mat image = imread("Graticule_picture.png");
//     cvtColor(image, image, COLOR_BGR2GRAY); //make sure image is greyscale

//     double scale = 0.5;
//     int centre = 320;

//     auto t1 = high_resolution_clock::now();

//     for (int i = 0; i < 20; i++) {
//         int sharp = sharpness(image, scale);
//         //Point p(sharp, 1);           // wanted to show a line where best focus is (not working for some reason)
//         //Point q(sharp, 200);
//         //int thickness = 1;
//         //Scalar colorLine(0, 255, 0);
//         //cv::line(image, p, q, colorLine, thickness);
//         //imshow("Source image", image);
//         //int l = waitKey(0); // Wait for a keystroke in the window
//         cout << i<<": ";
//         cout << sharp<<"\n";
//     }

//     auto t2 = high_resolution_clock::now();

//     /* Getting number of milliseconds as an integer. */
//     auto ms_int = duration_cast<milliseconds>(t2 - t1);

//     /* Getting number of milliseconds as a double. */
//     duration<double, std::milli> ms_double = t2 - t1;

//     std::cout << ms_int.count()/1000 << "s\n";
//     std::cout << 20/(ms_double.count()*0.001) << "Iters per second";


//     std::cout << "Program finished";

// }


