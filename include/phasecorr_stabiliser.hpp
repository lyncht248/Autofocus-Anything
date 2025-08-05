/**
 * @file phasecorr_stabiliser.hpp
 * @brief Phase correlation-based stabilizer for video frames.
 */

#pragma once
#include <opencv2/opencv.hpp>

/**
 * @class PhaseCorrStabiliser
 * @brief A class for stabilizing video frames using phase correlation.
 *
 * This class provides methods to initialize a reference frame, compute the shift
 * between frames, and reset the reference frame. It uses OpenCV for image processing.
 */
class PhaseCorrStabiliser
{
public:
    /**
     * @brief Constructor for PhaseCorrStabiliser.
     */
    PhaseCorrStabiliser();

    /**
     * @brief Destructor for PhaseCorrStabiliser.
     */
    ~PhaseCorrStabiliser();

    /**
     * @brief Initializes the reference frame with the first frame.
     * @param frame The first frame to set as the reference.
     */
    void initReference(const cv::Mat &frame);

    /**
     * @brief Computes the shift for new frames relative to the reference frame.
     * @param frame The new frame to compute the shift for.
     * @return The computed shift as a 2D point.
     */
    cv::Point2f computeShift(const cv::Mat &frame);

    /**
     * @brief Resets the reference frame.
     */
    void reset();

private:
    cv::Mat referenceGray; /**< Grayscale version of the reference frame. */
    cv::Mat hannWindow; /**< Hanning window used for phase correlation. */
};