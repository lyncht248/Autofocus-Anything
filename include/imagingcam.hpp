#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <functional>

/**
 * ImagingCam class handles the camera operations including ROI sharpness monitoring,
 * focus search, and depth mapping.
 */
class ImagingCam
{
public:
    /**
     * Constructor for ImagingCam.
     * Initializes the camera system and sets default parameters.
     * 
     * @param system Reference to the System object.
     */
    ImagingCam(System &system);

    /**
     * Destructor for ImagingCam.
     * Ensures all threads are stopped and resources are released.
     */
    ~ImagingCam();

    /**
     * Starts the monitoring thread for ROI sharpness analysis.
     */
    void start();

    /**
     * Stops the monitoring thread for ROI sharpness analysis.
     */
    void stop();

    /**
     * Sets the size of the Region of Interest (ROI).
     * 
     * @param width Width of the ROI.
     * @param height Height of the ROI.
     */
    void setROISize(int width, int height);

    /**
     * Sets the center coordinates of the Region of Interest (ROI).
     * 
     * @param centerX X-coordinate of the ROI center.
     * @param centerY Y-coordinate of the ROI center.
     */
    void setROICenter(int centerX, int centerY);

    /**
     * Retrieves the current ROI parameters.
     * 
     * @param centerX Reference to store the X-coordinate of the ROI center.
     * @param centerY Reference to store the Y-coordinate of the ROI center.
     * @param width Reference to store the width of the ROI.
     * @param height Reference to store the height of the ROI.
     */
    void getCurrentROI(int &centerX, int &centerY, int &width, int &height) const;

    /**
     * Starts the focus search process for the current ROI.
     */
    void startROIFocusSearch();

    /**
     * Checks if the focus search is currently active.
     * 
     * @return True if focus search is active, false otherwise.
     */
    bool isFocusSearchActive() const { return m_focusSearchActive.load(); }

    /**
     * Starts the depth mapping process.
     */
    void startDepthMapping();

    /**
     * Checks if the depth mapping process is currently active.
     * 
     * @return True if depth mapping is active, false otherwise.
     */
    bool isDepthMappingActive() const { return m_depthMappingActive.load(); }

    /**
     * Sets the callback function for Hold Focus mode updates.
     * 
     * @param callback Function to be called when Hold Focus mode changes.
     */
    void setHoldFocusCallback(HoldFocusCallback callback) { m_holdFocusCallback = callback; }

    /**
     * Sets the callback function for Best Focus position updates.
     * 
     * @param callback Function to be called when the best focus position is found.
     */
    void setBestFocusCallback(BestFocusCallback callback) { m_bestFocusCallback = callback; }

    /**
     * Sets the callback function for Search Complete updates.
     * 
     * @param callback Function to be called when the focus search is complete.
     */
    void setSearchCompleteCallback(SearchCompleteCallback callback) { m_searchCompleteCallback = callback; }

private:
    /**
     * Thread function that runs at 5Hz to monitor ROI sharpness.
     */
    void monitorThreadFunction();

    /**
     * Calculates the sharpness of a region using the Tenengrad method.
     * 
     * @param region Image region to calculate sharpness for.
     * @return Sharpness score of the region.
     */
    double calculateSharpness(const cv::Mat &region);

    /**
     * Extracts the Region of Interest (ROI) from a frame based on current settings.
     * 
     * @param frame Input frame to extract ROI from.
     * @return Extracted ROI as a cv::Mat object.
     */
    cv::Mat extractROI(const cv::Mat &frame);

    /**
     * Implements the focus search algorithm using golden section search.
     */
    void performFocusSearch();

    /**
     * Attempts to use the depth map for instant focus positioning if available.
     * 
     * @return True if depth map focus was successful, false otherwise.
     */
    bool tryDepthMapFocus();

    /**
     * Implements the depth mapping process.
     */
    void performDepthMapping();

    /**
     * Calculates a sharpness image using the Roberts Cross operator.
     * 
     * @param frame Input frame to calculate sharpness image for.
     * @return Sharpness image as a cv::Mat object.
     */
    cv::Mat calculateSharpnessImage(const cv::Mat &frame);

    /**
     * Calculates a sharpness image using the Tenengrad (Sobel) operator.
     * 
     * @param frame Input frame to calculate sharpness image for.
     * @return Sharpness image as a cv::Mat object.
     */
    cv::Mat calculateSharpnessImageTenengrad(const cv::Mat &frame);

    /**
     * Creates a smoothly padded frame to eliminate edge artifacts.
     * 
     * @param frame Input frame to pad.
     * @param validMask Mask indicating valid regions in the frame.
     * @return Padded frame as a cv::Mat object.
     */
    cv::Mat createSmoothedPaddedFrame(const cv::Mat &frame, const cv::Mat &validMask);

    /**
     * Performs local max suppression on the depth image to remove artifacts.
     * 
     * @param depthImage Depth image to process.
     * @param width Width of the depth image.
     * @param height Height of the depth image.
     */
    void performLocalMaxSuppression(std::vector<std::vector<std::pair<double, double>>> &depthImage,
                                    int width, int height);

    /**
     * Sets the desired focus position and waits for the lens to settle.
     * 
     * @param position Desired focus position.
     */
    void setFocusPositionAndWait(int position);

    /**
     * Sets the desired focus position and waits for the lens to settle for a custom time.
     * 
     * @param position Desired focus position.
     * @param settleTimeMs Time in milliseconds to wait for the lens to settle.
     */
    void setFocusPositionAndWaitLong(int position, int settleTimeMs);

    /**
     * Retrieves the current sharpness of the ROI.
     * 
     * @return Sharpness score of the current ROI.
     */
    double getCurrentROISharpness();

    /**
     * Retrieves multiple sharpness samples and returns the average for reliability.
     * 
     * @param numSamples Number of samples to average.
     * @return Averaged sharpness score.
     */
    double getAveragedROISharpness(int numSamples = SAMPLES_PER_POSITION);

    System &m_system;
    std::thread m_monitorThread;
    std::atomic<bool> m_running;

    // ROI parameters
    int m_roiWidth;
    int m_roiHeight;
    std::atomic<int> m_roiCenterX;
    std::atomic<int> m_roiCenterY;
    std::atomic<bool> m_useCustomCenter;

    // Timing
    std::chrono::steady_clock::time_point m_lastProcessTime;
    const std::chrono::milliseconds m_processInterval{200}; // 5Hz = 200ms

    // Focus search parameters
    std::atomic<bool> m_focusSearchActive;
    std::atomic<bool> m_focusSearchRequested;

    // Depth mapping parameters
    std::atomic<bool> m_depthMappingActive;
    std::atomic<bool> m_depthMappingRequested;

    // Search parameters
    static constexpr int FOCUS_MIN = 190;          // Minimum focus position
    static constexpr int FOCUS_MAX = 450;          // Maximum focus position
    static constexpr int FOCUS_TOLERANCE = 5;      // Good enough tolerance
    static constexpr int SETTLE_TIME_MS = 150;     // Reduced time for lens to settle
    static constexpr int SAMPLES_PER_POSITION = 3; // Multiple samples for reliability

    // Mutex for thread-safe ROI parameter access
    mutable std::mutex m_roiMutex;

    // Callback functions for GUI updates
    HoldFocusCallback m_holdFocusCallback;
    BestFocusCallback m_bestFocusCallback;
    SearchCompleteCallback m_searchCompleteCallback;
};