#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <functional>

class System; // Forward declaration

class ImagingCam
{
public:
    ImagingCam(System &system);
    ~ImagingCam();

    // Start/stop the monitoring thread
    void start();
    void stop();

    // Set ROI size (default 50x50)
    void setROISize(int width, int height);

    // Set ROI center coordinates (default is image center)
    void setROICenter(int centerX, int centerY);

    // Get current ROI parameters for display
    void getCurrentROI(int &centerX, int &centerY, int &width, int &height) const;

    // Start focus search for current ROI
    void startROIFocusSearch();

    // Check if focus search is currently active
    bool isFocusSearchActive() const { return m_focusSearchActive.load(); }

    // Start depth mapping process
    void startDepthMapping();

    // Check if depth mapping is currently active
    bool isDepthMappingActive() const { return m_depthMappingActive.load(); }

    // Callback functions for GUI integration
    using HoldFocusCallback = std::function<void(bool)>;
    using BestFocusCallback = std::function<void(int)>;
    using SearchCompleteCallback = std::function<void(bool)>;

    void setHoldFocusCallback(HoldFocusCallback callback) { m_holdFocusCallback = callback; }
    void setBestFocusCallback(BestFocusCallback callback) { m_bestFocusCallback = callback; }
    void setSearchCompleteCallback(SearchCompleteCallback callback) { m_searchCompleteCallback = callback; }

private:
    // Thread function that runs at 5Hz
    void monitorThreadFunction();

    // Calculate sharpness for a region using Tenengrad method
    double calculateSharpness(const cv::Mat &region);

    // Extract ROI from frame based on current settings
    cv::Mat extractROI(const cv::Mat &frame);

    // Focus search implementation using golden section search
    void performFocusSearch();

    // Use depth map for instant focus positioning if available
    bool tryDepthMapFocus();

    // Depth mapping implementation
    void performDepthMapping();

    // Calculate sharpness image using Roberts Cross operator
    cv::Mat calculateSharpnessImage(const cv::Mat &frame);

    // Calculate sharpness image using Tenengrad (Sobel) operator
    cv::Mat calculateSharpnessImageTenengrad(const cv::Mat &frame);

    // Create smoothly padded frame to eliminate edge artifacts
    cv::Mat createSmoothedPaddedFrame(const cv::Mat &frame, const cv::Mat &validMask);

    // Perform local max suppression on depth image to remove artifacts
    void performLocalMaxSuppression(std::vector<std::vector<std::pair<double, double>>> &depthImage,
                                    int width, int height);

    // Set desired focus position and wait for lens to settle SETTLE_TIME_MS
    void setFocusPositionAndWait(int position);

    // Set desired focus position and wait for lens to settle for a custom time
    void setFocusPositionAndWaitLong(int position, int settleTimeMs);

    // Get current ROI sharpness
    double getCurrentROISharpness();

    // Get multiple samples and return the average for more reliable measurement
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