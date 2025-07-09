#pragma once
#include <opencv2/opencv.hpp>

class PhaseCorrStabiliser
{
public:
    PhaseCorrStabiliser();
    ~PhaseCorrStabiliser();

    // Initialize reference with the first frame
    void initReference(const cv::Mat &frame);

    // Compute shift for new frames
    cv::Point2f computeShift(const cv::Mat &frame);

    // Reset the reference
    void reset();

private:
    cv::Mat referenceGray;
    cv::Mat hannWindow;
};