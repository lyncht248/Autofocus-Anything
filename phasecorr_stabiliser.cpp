#include "phasecorr_stabiliser.hpp"

PhaseCorrStabiliser::PhaseCorrStabiliser() {}
PhaseCorrStabiliser::~PhaseCorrStabiliser() {}

void PhaseCorrStabiliser::initReference(const cv::Mat &frame) {
    // Convert to grayscale if input is BGR
    cv::Mat grayFrame;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
    } else {
        grayFrame = frame;
    }

    // Convert to floating point (CV_32F)
    grayFrame.convertTo(referenceGray, CV_32F);
    
    // Create Hanning window once during initialization
    if (hannWindow.empty()) {
        cv::createHanningWindow(hannWindow, referenceGray.size(), CV_32F);
    }
}

cv::Point2f PhaseCorrStabiliser::computeShift(const cv::Mat &frame) {
    // Convert to grayscale if input is BGR
    cv::Mat grayFrame;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
    } else {
        grayFrame = frame;
    }

    // Convert to floating point (CV_32F)
    cv::Mat currentGray;
    grayFrame.convertTo(currentGray, CV_32F);

    // Use OpenCV's phaseCorrelate with pre-computed Hanning window
    cv::Point2d shift = cv::phaseCorrelate(currentGray, referenceGray, hannWindow);

    // Update reference for next iteration
    currentGray.copyTo(referenceGray);

    return cv::Point2f((float)shift.x, (float)shift.y);
}

void PhaseCorrStabiliser::reset() {
    referenceGray.release();
    // Don't release hannWindow since it can be reused
}