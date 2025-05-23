#include "sharpness_analyzer.hpp"

SharpnessAnalyzer::SharpnessAnalyzer() {
    // Initialize with empty values
    m_sharpnessValues.resize(m_numSegments, 0.0);
}

SharpnessAnalyzer::~SharpnessAnalyzer() {
}

std::vector<double> SharpnessAnalyzer::analyzeFrame(const cv::Mat& frame) {
    if (frame.empty()) {
        return m_sharpnessValues;
    }
    
    // Resize for performance
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(), 0.25, 0.25); // Scale down to 1/4
    
    // Convert to grayscale if needed
    cv::Mat gray;
    if (resized.channels() == 3) {
        cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = resized;
    }
    
    // Calculate sharpness for each segment
    std::vector<double> newValues;
    newValues.resize(m_numSegments);
    
    int segmentWidth = gray.cols / m_numSegments;
    
    for (int i = 0; i < m_numSegments; i++) {
        int x = i * segmentWidth;
        int width = (i == m_numSegments - 1) ? gray.cols - x : segmentWidth;
        
        cv::Rect roi(x, 0, width, gray.rows);
        cv::Mat segment = gray(roi);
        
        newValues[i] = calculateSharpness(segment);
    }
    
    // Update stored values with mutex protection
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_sharpnessValues = newValues;
    }
    
    return newValues;
}

std::vector<double> SharpnessAnalyzer::analyzeFrame(const unsigned char* data, int width, int height) {
    if (!data || width <= 0 || height <= 0) {
        return m_sharpnessValues;
    }
    
    // Create OpenCV Mat from raw data (assuming 8-bit grayscale)
    cv::Mat frame(height, width, CV_8UC1, (void*)data);
    return analyzeFrame(frame);
}

std::vector<double> SharpnessAnalyzer::getSharpnessValues() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_sharpnessValues;
}

double SharpnessAnalyzer::calculateSharpness(const cv::Mat& region) {
    // Calculate gradient magnitude using Sobel operator
    cv::Mat gradX, gradY;
    cv::Sobel(region, gradX, CV_16S, 1, 0);
    cv::Sobel(region, gradY, CV_16S, 0, 1);
    
    cv::Mat absGradX, absGradY;
    cv::convertScaleAbs(gradX, absGradX);
    cv::convertScaleAbs(gradY, absGradY);
    
    cv::Mat grad;
    cv::addWeighted(absGradX, 0.5, absGradY, 0.5, 0, grad);
    
    // Return mean of gradient magnitude as sharpness
    cv::Scalar mean = cv::mean(grad);
    return mean[0];
} 