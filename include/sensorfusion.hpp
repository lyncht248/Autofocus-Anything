/**

* @file sensorfusion.hpp
 * @brief Declares the `SensorFusion` class for managing camera operations.
 *
 * This file provides the declaration of the `SensorFusion` class, which
integrates with the `System`
 * object to handle camera functionalities such as ROI sharpness monitoring,
focus search, and depth
 * mapping. It includes thread-safe methods for updating camera parameters and
performing image analysis.
 */

#pragma once
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

class System; // Forward declaration

/**
 * @class SensorFusion
 * @brief Handles camera operations including ROI sharpness monitoring, focus
 * search, and depth mapping.
 *
 * This class integrates with the System object to manage camera functionalities
 * such as setting ROI, performing focus searches, and generating depth maps. It
 * also provides thread-safe methods for monitoring and updating camera
 * parameters.
 */
class SensorFusion {
public:
  /**
   * @brief Constructor for SensorFusion.
   *
   * Initializes the camera system and sets default parameters.
   *
   * @param system Reference to the System object.
   */
  SensorFusion(System &system);

  /**
   * @brief Destructor for SensorFusion.
   *
   * Ensures all threads are stopped and resources are released.
   */
  ~SensorFusion();

  /**
   * @brief Starts the monitoring thread for ROI sharpness analysis.
   */
  void start();

  /**
   * @brief Stops the monitoring thread for ROI sharpness analysis.
   */
  void stop();

  /**
   * @brief Sets the size of the Region of Interest (ROI).
   *
   * @param width Width of the ROI.
   * @param height Height of the ROI.
   */
  void setROISize(int width, int height);

  /**
   * @brief Sets the center coordinates of the Region of Interest (ROI).
   *
   * @param centerX X-coordinate of the ROI center.
   * @param centerY Y-coordinate of the ROI center.
   */
  void setROICenter(int centerX, int centerY);

  /**
   * @brief Retrieves the current ROI parameters.
   *
   * @param centerX Reference to store the X-coordinate of the ROI center.
   * @param centerY Reference to store the Y-coordinate of the ROI center.
   * @param width Reference to store the width of the ROI.
   * @param height Reference to store the height of the ROI.
   */
  void getCurrentROI(int &centerX, int &centerY, int &width, int &height) const;

  /**
   * @brief Starts the focus search process for the current ROI.
   */
  void startROIFocusSearch();

  /**
   * @brief Checks if the focus search is currently active.
   *
   * @return True if focus search is active, false otherwise.
   */
  bool isFocusSearchActive() const { return m_focusSearchActive.load(); }

  /**
   * @brief Starts the depth mapping process.
   */
  void startDepthMapping();

  /**
   * @brief Checks if the depth mapping process is currently active.
   *
   * @return True if depth mapping is active, false otherwise.
   */
  bool isDepthMappingActive() const { return m_depthMappingActive.load(); }

  // Callback functions for GUI integration
  using HoldFocusCallback = std::function<void(bool)>;
  using BestFocusCallback = std::function<void(int)>;
  using SearchCompleteCallback = std::function<void(bool)>;

  /**
   * @brief Sets the callback function for Hold Focus mode updates.
   *
   * @param callback Function to be called when Hold Focus mode changes.
   */
  void setHoldFocusCallback(HoldFocusCallback callback) {
    m_holdFocusCallback = callback;
  }

  /**
   * @brief Sets the callback function for Best Focus position updates.
   *
   * @param callback Function to be called when the best focus position is
   * found.
   */
  void setBestFocusCallback(BestFocusCallback callback) {
    m_bestFocusCallback = callback;
  }

  /**
   * @brief Sets the callback function for Search Complete updates.
   *
   * @param callback Function to be called when the focus search is complete.
   */
  void setSearchCompleteCallback(SearchCompleteCallback callback) {
    m_searchCompleteCallback = callback;
  }

private:
  /**
   * @brief Thread function that runs at 5Hz to monitor ROI sharpness.
   */
  void monitorThreadFunction();

  /**
   * @brief Calculates the sharpness of a region using the Tenengrad method.
   *
   * @param region Image region to calculate sharpness for.
   * @return Sharpness score of the region.
   */
  double calculateSharpness(const cv::Mat &region);

  /**
   * @brief Extracts the Region of Interest (ROI) from a frame based on current
   * settings.
   *
   * @param frame Input frame to extract ROI from.
   * @return Extracted ROI as a cv::Mat object.
   */
  cv::Mat extractROI(const cv::Mat &frame);

  /**
   * @brief Implements the focus search algorithm using golden section search.
   */
  void performFocusSearch();

  /**
   * @brief Attempts to use the depth map for instant focus positioning if
   * available.
   *
   * @return True if depth map focus was successful, false otherwise.
   */
  bool tryDepthMapFocus();

  /**
   * @brief Implements the depth mapping process.
   */
  void performDepthMapping();

  /**
   * @brief Calculates a sharpness image using the Roberts Cross operator.
   *
   * @param frame Input frame to calculate sharpness image for.
   * @return Sharpness image as a cv::Mat object.
   */
  cv::Mat calculateSharpnessImage(const cv::Mat &frame);

  /**
   * @brief Calculates a sharpness image using the Tenengrad (Sobel) operator.
   *
   * @param frame Input frame to calculate sharpness image for.
   * @return Sharpness image as a cv::Mat object.
   */
  cv::Mat calculateSharpnessImageTenengrad(const cv::Mat &frame);

  /**
   * @brief Creates a smoothly padded frame to eliminate edge artifacts.
   *
   * @param frame Input frame to pad.
   * @param validMask Mask indicating valid regions in the frame.
   * @return Padded frame as a cv::Mat object.
   */
  cv::Mat createSmoothedPaddedFrame(const cv::Mat &frame,
                                    const cv::Mat &validMask);

  /**
   * @brief Performs local max suppression on the depth image to remove
   * artifacts.
   *
   * @param depthImage Depth image to process.
   * @param width Width of the depth image.
   * @param height Height of the depth image.
   */
  void performLocalMaxSuppression(
      std::vector<std::vector<std::pair<double, double>>> &depthImage,
      int width, int height);

  /**
   * @brief Sets the desired focus position and waits for the lens to settle.
   *
   * @param position Desired focus position.
   */
  void setFocusPositionAndWait(int position);

  /**
   * @brief Sets the desired focus position and waits for the lens to settle for
   * a custom time.
   *
   * @param position Desired focus position.
   * @param settleTimeMs Time in milliseconds to wait for the lens to settle.
   */
  void setFocusPositionAndWaitLong(int position, int settleTimeMs);

  /**
   * @brief Retrieves the current sharpness of the ROI.
   *
   * @return Sharpness score of the current ROI.
   */
  double getCurrentROISharpness();

  /**
   * @brief Retrieves multiple sharpness samples and returns the average for
   * reliability.
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
  static constexpr int FOCUS_MIN = 190;      // Minimum focus position
  static constexpr int FOCUS_MAX = 450;      // Maximum focus position
  static constexpr int FOCUS_TOLERANCE = 5;  // Good enough tolerance
  static constexpr int SETTLE_TIME_MS = 150; // Reduced time for lens to settle
  static constexpr int SAMPLES_PER_POSITION =
      3; // Multiple samples for reliability

  // Mutex for thread-safe ROI parameter access
  mutable std::mutex m_roiMutex;

  // Callback functions for GUI updates
  HoldFocusCallback m_holdFocusCallback;
  BestFocusCallback m_bestFocusCallback;
  SearchCompleteCallback m_searchCompleteCallback;
};
