#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>

/**
 * @class SharpnessAnalyzer
 * @brief Analyzes the sharpness of image frames by computing horizontal sharpness profiles.
 * 
 * The SharpnessAnalyzer class provides methods to analyze image frames and compute sharpness
 * values for horizontal segments. It supports both OpenCV Mat objects and raw data pointers.
 */
class SharpnessAnalyzer {
public:
    /**
     * @brief Constructor for the SharpnessAnalyzer class.
     * 
     * Initializes the sharpness analyzer with default values.
     */
    SharpnessAnalyzer();

    /**
     * @brief Destructor for the SharpnessAnalyzer class.
     */
    ~SharpnessAnalyzer();

    /**
     * @brief Analyzes a frame and computes the horizontal sharpness profile.
     * 
     * Divides the frame into horizontal segments and calculates sharpness values for each segment.
     * @param frame Input image frame as an OpenCV Mat object.
     * @return Vector of sharpness values for each segment.
     */
    std::vector<double> analyzeFrame(const cv::Mat& frame);

    /**
     * @brief Analyzes a frame with raw data pointer.
     * 
     * Creates an OpenCV Mat from the raw data and computes the horizontal sharpness profile.
     * @param data Pointer to the raw image data.
     * @param width Width of the image.
     * @param height Height of the image.
     * @return Vector of sharpness values for each segment.
     */
    std::vector<double> analyzeFrame(const unsigned char* data, int width, int height);

    /**
     * @brief Gets the last computed sharpness values.
     * 
     * Returns the sharpness values computed during the last analysis.
     * @return Vector of sharpness values.
     */
    std::vector<double> getSharpnessValues();

private:
    std::vector<double> m_sharpnessValues; ///< Stores the sharpness values for each segment.
    std::mutex m_mutex; ///< Mutex for thread-safe access to sharpness values.
    const int m_numSegments = 10; ///< Number of horizontal segments to analyze.

    /**
     * @brief Helper method to calculate sharpness for a region.
     * 
     * Computes the sharpness of a region using gradient magnitude.
     * @param region Input image region as an OpenCV Mat object.
     * @return Sharpness value for the region.
     */
    double calculateSharpness(const cv::Mat& region);
};