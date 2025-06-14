#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>

class SharpnessAnalyzer {
public:
    SharpnessAnalyzer();
    ~SharpnessAnalyzer();
    
    // Analyze a frame and compute horizontal sharpness profile
    std::vector<double> analyzeFrame(const cv::Mat& frame);
    
    // Analyze a frame with raw data pointer (compatible with VidFrame)
    std::vector<double> analyzeFrame(const unsigned char* data, int width, int height);
    
    // Get the last computed sharpness values
    std::vector<double> getSharpnessValues();
    
private:
    std::vector<double> m_sharpnessValues;
    std::mutex m_mutex;
    const int m_numSegments = 10; // Number of horizontal segments to analyze
    
    // Helper method to calculate sharpness for a region
    double calculateSharpness(const cv::Mat& region);
}; 