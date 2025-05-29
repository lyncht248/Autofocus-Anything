#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

#include "autofocus.hpp"
#include "tiltedcam.hpp"
#include "lens.hpp"
#include "logfile.hpp"

// Global calibration settings
const int NUM_TEST_IMAGES = 5;    // Number of images to capture for calibration
const int KERNEL_SIZE = 16;       // Same kernel size as in autofocus.cpp
const double MIN_STD_DEV = 0.05;  // Minimum std_dev factor to test
const double MAX_STD_DEV = 0.30;  // Maximum std_dev factor to test
const double STD_DEV_STEP = 0.01; // Step size for std_dev testing
const std::string RESULTS_FILE = "../output/calibration_results.txt";

// Function to calculate least square error between two curves
double calculateLeastSquareError(const std::vector<double>& curve1, const std::vector<double>& curve2) {
    if (curve1.size() != curve2.size()) {
        return -1.0;
    }
    
    double sumSquaredDiff = 0.0;
    for (size_t i = 0; i < curve1.size(); ++i) {
        double diff = curve1[i] - curve2[i];
        sumSquaredDiff += diff * diff;
    }
    
    return sumSquaredDiff;
}

// Add this to make our test class a friend of autofocus (like AutofocusTest already is)
class DeviceCalibrationTest;

// Update the autofocus.hpp file to add our test as a friend
// In autofocus.hpp, add this line after "friend class AutofocusTest;"
// friend class DeviceCalibrationTest;

class DeviceCalibrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize logger if not already done
        try {
            // Use the existing logger that's already set up in your app
            if (!logger) {
                // If no logger exists, create a basic one - adjust according to your actual logger setup
                logger = spdlog::basic_logger_mt("calibration", "logs/calibration.log");
                logger->set_level(spdlog::level::info);
            }
        } catch (const spdlog::spdlog_ex&) {
            // Logger already exists
        }
    }
    
    // Shared resources for tests
    std::unique_ptr<autofocus> af;
};

TEST_F(DeviceCalibrationTest, FindOptimalStdDevFactor) {
    // Create autofocus object
    af = std::make_unique<autofocus>();
    ASSERT_TRUE(af->initialize()) << "Failed to initialize autofocus system";
    
    // Access the tiltedcam object
    tiltedcam& camera = af->getTiltedCam();
    
    // Start the camera capture thread
    camera.startCaptureThread();
    
    // Wait for the camera to start capturing frames
    std::cout << "Waiting for camera to start capturing frames..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Scale factor for image processing
    double scale = 0.5;
    
    // Results vector to store std_dev factors and their errors
    std::vector<std::pair<double, double>> calibrationResults;
    
    // Collect images and sharpness curves
    std::vector<std::vector<double>> allSharpnessCurves;
    int imWidth = camera.getImageWidth();
    int imHeight = camera.getImageHeight();
    
    // Get several test images to ensure good calibration
    std::cout << "Capturing " << NUM_TEST_IMAGES << " test images for calibration...\n";
    
    // We need to compute the sharpness curves ourselves since computesharpness is private
    for (int img_idx = 0; img_idx < NUM_TEST_IMAGES; ++img_idx) {
        // Get frame from camera
        unsigned char* img_buffer = new unsigned char[imWidth * imHeight];
        
        // Try a few times to get a valid frame
        bool frameObtained = false;
        for (int attempt = 0; attempt < 5 && !frameObtained; attempt++) {
            frameObtained = camera.getLatestFrame(img_buffer, imWidth * imHeight);
            if (!frameObtained) {
                std::cout << "Attempt " << (attempt+1) << " to get frame failed, retrying..." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        
        ASSERT_TRUE(frameObtained) << "Failed to get frame from camera after multiple attempts";
        
        // Convert to OpenCV format and resize
        cv::Mat image(imHeight, imWidth, CV_8U, img_buffer);
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(), scale, scale);
        
        // Use blurring like in computeBestFocus
        cv::Mat blurred;
        cv::GaussianBlur(resized, blurred, cv::Size(3, 3), 1, 1, cv::BORDER_DEFAULT);
        
        // Calculate the sharpness curve manually since we can't call the private method
        // This is similar to what computesharpness does in autofocus.cpp
        cv::Mat sharpness_image;
        {
            // Roberts cross filter (similar to autofocus.cpp)
            cv::Mat temp_matrixx = (cv::Mat_<double>(2, 2) << 1, 0, 0, -1);
            cv::Mat temp_matrixy = (cv::Mat_<double>(2, 2) << 0, 1, -1, 0);
            
            cv::Mat img_x, img_y;
            cv::filter2D(blurred, img_x, -1, temp_matrixx);
            cv::filter2D(blurred, img_y, -1, temp_matrixy);
            
            cv::Mat img_x_squared, img_y_squared;
            cv::multiply(img_x, img_x, img_x_squared);
            cv::multiply(img_y, img_y, img_y_squared);
            cv::add(img_x_squared, img_y_squared, sharpness_image);
        }
        
        // ROI processing
        std::vector<double> sharpnessCurve;
        for (int i = 0; i < resized.cols - KERNEL_SIZE; i++) {
            cv::Rect roi(i, 0, KERNEL_SIZE, resized.rows);
            cv::Mat regionSharpnessImage = sharpness_image(roi);
            cv::Scalar regionSharpness = cv::mean(regionSharpnessImage);
            double regionSharpnessScore = regionSharpness[0];
            sharpnessCurve.push_back(regionSharpnessScore);
        }
        
        // Save the sharpness curve
        allSharpnessCurves.push_back(sharpnessCurve);
        
        // Save the image for reference
        std::string imagePath = "../output/calibration_image_" + std::to_string(img_idx) + ".png";
        cv::imwrite(imagePath, resized);
        
        std::cout << "Processed calibration image " << (img_idx + 1) << "/" << NUM_TEST_IMAGES << std::endl;
        
        delete[] img_buffer;
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Wait between frames
    }
    
    std::cout << "Completed image capture. Testing different std_dev values...\n";
    
    // Now we need to manually implement fitnormalcurve since it's also private
    // This is similar to what's in autofocus.cpp
    auto fitNormalCurve = [&](const std::vector<double>& sharpnessCurve, int kernel, double std_dev_factor) {
        std::vector<double> norm_curve;
        std::vector<double> sum_of_diffs;
        double std_dev = std_dev_factor * (sharpnessCurve.size());
        
        double amplitude = (*std::max_element(begin(sharpnessCurve), end(sharpnessCurve)) - 
                           *std::min_element(begin(sharpnessCurve), end(sharpnessCurve)));
        double offset = *std::min_element(begin(sharpnessCurve), end(sharpnessCurve));
        
        // Function to calculate normal PDF
        auto normpdf = [](double x, double u, double s) {
            const double ONE_OVER_SQRT_2PI = 0.39894228040143267793994605993438;
            return (ONE_OVER_SQRT_2PI / s) * exp(-0.5 * std::pow(((x - u) / s), 2));
        };
        
        // Find the optimal mean (similar to fitnormalcurve)
        for (int j = 1; j <= sharpnessCurve.size(); j++) {
            std::vector<double> norm_curve_temp;
            for (int i = 0; i < sharpnessCurve.size(); i++) {
                double normResult = normpdf(i, j, std_dev);
                norm_curve_temp.push_back(normResult);
            }
            
            double factor = 1.0 / *std::max_element(begin(norm_curve_temp), end(norm_curve_temp));
            for (int i = 0; i < sharpnessCurve.size(); i++) {
                norm_curve_temp[i] = norm_curve_temp[i] * factor * amplitude + offset;
            }
            
            std::vector<double> differences;
            for (int z = 0; z < sharpnessCurve.size(); z++) {
                double diff = std::abs(sharpnessCurve[z] - norm_curve_temp[z]);
                differences.push_back(diff);
            }
            double sum_of_diff = std::accumulate(differences.begin(), differences.end(), 
                                              decltype(differences)::value_type(0));
            sum_of_diffs.push_back(sum_of_diff);
        }
        
        double mean = std::distance(begin(sum_of_diffs), 
                                std::min_element(begin(sum_of_diffs), end(sum_of_diffs))) + 1;
        
        // Generate final curve
        for (int i = 0; i < sharpnessCurve.size(); i++) {
            double normResult = normpdf(i, mean, std_dev);
            norm_curve.push_back(normResult);
        }
        
        double factor = 1.0 / *std::max_element(begin(norm_curve), end(norm_curve));
        for (int i = 0; i < sharpnessCurve.size(); i++) {
            norm_curve[i] = norm_curve[i] * factor * amplitude + offset;
        }
        
        return norm_curve;
    };
    
    // Test different std_dev factors
    for (double std_dev_factor = MIN_STD_DEV; std_dev_factor <= MAX_STD_DEV; std_dev_factor += STD_DEV_STEP) {
        double totalError = 0.0;
        
        // Process each sharpness curve with the current std_dev
        for (const auto& sharpnessCurve : allSharpnessCurves) {
            // Fit normal curve with current std_dev factor using our local implementation
            std::vector<double> fittedCurve = fitNormalCurve(sharpnessCurve, KERNEL_SIZE, std_dev_factor);
            
            // Calculate error for this curve
            double error = calculateLeastSquareError(sharpnessCurve, fittedCurve);
            totalError += error;
            
            // Save visualization for key values
            if (&sharpnessCurve == &allSharpnessCurves[0] && 
                (std::abs(std_dev_factor - MIN_STD_DEV) < 0.001 || 
                 std::abs(std_dev_factor - 0.15) < 0.001 || 
                 std::abs(std_dev_factor - MAX_STD_DEV) < 0.001)) {
                
                // Save the curves for visualization
                std::string curveFile = "../output/curve_stddev_" + std::to_string(std_dev_factor) + ".txt";
                std::ofstream outFile(curveFile);
                if (outFile.is_open()) {
                    outFile << "# Original, Fitted\n";
                    for (size_t i = 0; i < sharpnessCurve.size(); ++i) {
                        outFile << sharpnessCurve[i] << ", " << fittedCurve[i] << "\n";
                    }
                    outFile.close();
                }
            }
        }
        
        // Average error across all images
        double avgError = totalError / allSharpnessCurves.size();
        
        // Store result
        calibrationResults.push_back(std::make_pair(std_dev_factor, avgError));
        
        std::cout << "Tested std_dev_factor = " << std_dev_factor 
                  << ", average error = " << avgError << std::endl;
    }
    
    // Find the best std_dev factor (with minimum error)
    auto bestResult = std::min_element(calibrationResults.begin(), calibrationResults.end(), 
                                    [](const auto& a, const auto& b) { return a.second < b.second; });
    
    double optimalStdDevFactor = bestResult->first;
    double minError = bestResult->second;
    
    std::cout << "Calibration complete!" << std::endl;
    std::cout << "Optimal std_dev_factor = " << optimalStdDevFactor 
              << " with error = " << minError << std::endl;
    
    // Save all results to file
    std::ofstream resultsFile(RESULTS_FILE);
    ASSERT_TRUE(resultsFile.is_open()) << "Could not save results to file";
    
    resultsFile << "# std_dev_factor, error\n";
    for (const auto& result : calibrationResults) {
        resultsFile << result.first << ", " << result.second << "\n";
    }
    resultsFile.close();
    
    // Verify the optimal value is within expected range
    EXPECT_GE(optimalStdDevFactor, MIN_STD_DEV);
    EXPECT_LE(optimalStdDevFactor, MAX_STD_DEV);
    
    std::cout << "IMPORTANT: Update the std_dev_factor in autofocus.cpp to " 
              << optimalStdDevFactor << std::endl;
    
    // Don't forget to stop the capture thread when done
    camera.stopCaptureThread();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 