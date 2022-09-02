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

#include <cmath>
#include <numeric>

#define M_PI 3.14159265358979323846


//std::tuple<int, int> img_size(Mat img) {
    //int h = img.rows;
    //int w = img.cols;
    //return std::make_tuple(h, w);
//}


Scalar tenengrad(Mat img) {
    // Two sobel operations //
    Mat Gx, Gy;
    Sobel(img, Gx, CV_64F, 1, 0, 3);
    Sobel(img, Gy, CV_64F, 0, 1, 3);
    Scalar mean_grad = mean(Gx.mul(Gx) + Gy.mul(Gy));

    // One Sobel operation //
    //Mat Gxy;
    //Sobel(img, Gxy, CV_64F, 1, 1, 3);
    //Scalar mean_grad = mean(abs(Gxy));
    return mean_grad;

}
Scalar vollath(Mat img) {
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
    Scalar mean_grad_vol = sum1 - sum2;
    return mean_grad_vol;
}
Scalar canny(Mat img) {
    //Canny //
    Mat edges;
    Canny(img, edges, 25, 55, 3);
    Scalar mean_grad_canny = mean(edges.mul(edges));
    return mean_grad_canny;
}
Scalar RobertCross(Mat img) {
    //Robert Cross //
    Mat kernelx = (Mat_<double>(2, 2) << 1, 0, 0, -1);
    Mat kernely = (Mat_<double>(2, 2) << 0, 1, -1, 0);
    Mat img_x, img_y;
    filter2D(img, img_x, -1, kernelx);
    filter2D(img, img_y, -1, kernely);
    Scalar mean_grad_cross = mean(img_x.mul(img_x) + img_y.mul(img_y));
    return mean_grad_cross;
}





float Square(float value) {
    // Multiply value two times
    return value * value;
}

const double ONE_OVER_SQRT_2PI = 0.39894228040143267793994605993438;

double normpdf(double x, double u, double s) {
    return (ONE_OVER_SQRT_2PI / s) * exp(-0.5 * Square((x - u) / s));
}



vector<double> compute_tene_grad(Mat img, int size, ofstream& outputFile2) {
    int w = img.cols;
    int h = img.rows;
    int kernel_width = size;
    //Mat mask = Mat::zeros(Size(h,w), CV_64FC(1));
    //std::fill(h, w, 0);

    //vector<int> grad;
    //vector<int> grad_canny;
    //vector<int> grad_vollath;
    vector<double> grad_RobertCross;


    //for (int i = 0; i <w-kernel_width; i++) {
    //    Rect roi(i, 0, kernel_width, h); 
    //    Mat pixels = img(roi);
    //    //Mat pixels = img(cv::Rect(0, 0, kernel_width, h)).clone(); (another method of finding submatrix)
    //    Scalar check = tenengrad(pixels);
    //    int check1 = sum(check)[0];
    //    grad.push_back(check1);
    //}
    //for (int i = 0; i < w - kernel_width; i++) {
    //    Rect roi(i, 0, kernel_width, h); 
    //    Mat pixels = img(roi);
    //    //Mat pixels = img(cv::Rect(0, 0, kernel_width, h)).clone(); (another method of finding submatrix)
    //    Scalar check = canny(pixels);
    //    int check1 = sum(check)[0];
    //    grad_canny.push_back(check1);
    //}
    //for (int i = 0; i < w - kernel_width; i++) {
    //    Rect roi(i, 0, kernel_width, h); 
    //    Mat pixels = img(roi);
    //    //Mat pixels = img(cv::Rect(0, 0, kernel_width, h)).clone(); (another method of finding submatrix)
    //    Scalar check = vollath(pixels);
    //    int check1 = sum(check)[0];
    //    grad_vollath.push_back(check1);
    //}
    for (int i = 0; i < w - kernel_width; i++) {
        Rect roi(i, 0, kernel_width, h); 
        Mat pixels = img(roi);
        //Mat pixels = img(cv::Rect(0, 0, kernel_width, h)).clone(); (another method of finding submatrix)
        Scalar check = RobertCross(pixels);
        double check1 = sum(check)[0];
        grad_RobertCross.push_back(check1);
    }

    // Fits a normal curve to the data
    vector<double> norm_curve;
    vector<double> sum_of_diffs;

    double std_dev = 58; //determined experimentally... 
    double amplitude = (*max_element(begin(grad_RobertCross), end(grad_RobertCross)) - *min_element(begin(grad_RobertCross), end(grad_RobertCross))); 
    double offset = *min_element(begin(grad_RobertCross), end(grad_RobertCross));

    //Calculates sum_of_elems, a vector showing how well each normal curve with mean j fits grad
    for (int j = 1; j <= w - kernel_width; j++) {

        //Generates sin curve for given mean j
        vector<double> norm_curve_temp;
        for (int i = 0; i < w - kernel_width; i++) {
            //// creates sin curve
            //double sinResult = amplitude * sin(2 * M_PI * (0.5/275) * (i - j + 152) )  + offset;
            //if (sinResult < offset) { sinResult = offset; }
            double normResult = normpdf(i, j, std_dev);
            norm_curve_temp.push_back(normResult);
        }
        //scales so the max value (when i=j) is equal to the desired amplitude
        double factor = 1.0 / *max_element(begin(norm_curve_temp), end(norm_curve_temp));
        for (int i = 0; i < (w - kernel_width); i++) {
            norm_curve_temp[i] = norm_curve_temp[i] * factor * amplitude + offset;
        }

        //Calculates difference between sin_curve_temp and grad curve            
        vector<double> differences;
        for (int z = 0; z < w - kernel_width; z++) {
            double diff = abs(grad_RobertCross[z] - norm_curve_temp[z]);
            differences.push_back(diff);
        }
        double sum_of_diff = std::accumulate(differences.begin(), differences.end(), decltype(differences)::value_type(0));
        sum_of_diffs.push_back(sum_of_diff);
    }

    //Picks the mean that has the lowest overall difference
    double mean = distance(begin(sum_of_diffs), min_element(begin(sum_of_diffs), end(sum_of_diffs)));

    //Generates final sin_curve
    for (int i = 0; i < w - kernel_width; i++) {
        //// creates sin curve
        //double sinResult = amplitude * sin(2 * M_PI * (0.5/275) * (i - j + 152) )  + offset;
        //if (sinResult < offset) { sinResult = offset; }
        double normResult = normpdf(i, mean, std_dev);
        norm_curve.push_back(normResult);
    }
    double factor = 1.0 / *max_element(begin(norm_curve), end(norm_curve));
    for (int i = 0; i < (w - kernel_width); i++) {
        norm_curve[i] = norm_curve[i] * factor * amplitude + offset;
    }

    std::vector<double> dest(norm_curve.begin(), norm_curve.end());


    //auto focus = distance(begin(grad_RobertCross), max_element(begin(grad_RobertCross), end(grad_RobertCross)));
    //grad_RobertCross.push_back(focus); //place location of max focus at end of grad

    ////OUTPUTS SHARPNESS CURVE to OUTPUTFILE2
    //std::ostream_iterator<int> output_iterator2(outputFile2, ", ");
    //std::copy(grad.begin(), grad.end(), output_iterator2);
    //outputFile2 << "\n";
    ////OUTPUTS SHARPNESS CURVE to OUTPUTFILE3
    //std::ostream_iterator<int> output_iterator3(outputFile3, ", ");
    //std::copy(grad_canny.begin(), grad_canny.end(), output_iterator3);
    //outputFile3 << "\n";
    ////OUTPUTS SHARPNESS CURVE to OUTPUTFILE4
    //std::ostream_iterator<int> output_iterator4(outputFile4, ", ");
    //std::copy(grad_vollath.begin(), grad_vollath.end(), output_iterator4);
    //outputFile4 << "\n";


    ////outputs RobertCross curve to OUTPUTFILE
    //std::ostream_iterator<double> output_iterator2(outputFile2, ", ");
    //std::copy(grad_RobertCross.begin(), grad_RobertCross.end(), output_iterator2);
    //outputFile2 << "\n";


    //returns norm_curve with location of max focus at the end
    dest.push_back(mean);
    return dest;


    //auto focus = distance( begin(grad), max_element(begin(grad), end(grad)));
    //grad.push_back(focus); //place location of max focus at end of grad
    //return grad;

}

int sharpness(Mat img, double scale, ofstream &outputFile, ofstream& outputFile2) {
    // This code resizes the image to 1/2, then multiplies the final value by 2

    int height = round(img.rows * scale);
    int width = round(img.cols * scale);
    Mat resized;
    resize(img, resized, Size(width, height), scale, scale, INTER_AREA);

    //Specular reflection rejection
    Mat thresh;
    threshold(resized, thresh, 140, 255, THRESH_TRUNC); // threshold is 140. A lower threshold removes reflection beter, but some data in the image is lost 

    Mat blurred;
    GaussianBlur(resized, blurred, Size(3,3),1,1,BORDER_DEFAULT);  // put thresh instead of resized to use specular reflection removal


   //Mat equal;
    //equalizeHist(blurred, equal);


    vector<double> vec = compute_tene_grad(blurred, 16, outputFile2);

    ////OUTPUT NORMAL CURVE
    //std::ostream_iterator<double> output_iterator(outputFile, ", ");
    //std::copy(vec.begin(), vec.end(), output_iterator);
    //outputFile << "\n";

    //cv::imshow("Window1", resized);
    //cv::waitKey(1);


    return vec[vec.size() -1]/ scale; //get last value from vector (This is location of max focus)
}


