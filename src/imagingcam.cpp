#include "imagingcam.hpp"
#include "system.hpp"
#include "vidframe.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>

// External global variables from autofocus.cpp
extern int desiredLocBestFocus;
extern bool bHoldFocus;
extern bool bFindFocus;

ImagingCam::ImagingCam(System &system)
    : m_system(system), m_running(false), m_roiWidth(100), m_roiHeight(100), m_roiCenterX(-1), m_roiCenterY(-1), m_useCustomCenter(false), m_lastProcessTime(std::chrono::steady_clock::now()), m_focusSearchActive(false), m_focusSearchRequested(false)
{
}

ImagingCam::~ImagingCam()
{
    // Stop any active focus search first
    m_focusSearchActive = false;
    m_focusSearchRequested = false;

    // Wait a bit for any search threads to finish
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    stop();
}

void ImagingCam::start()
{
    if (m_running.load())
    {
        return; // Already running
    }

    m_running = true;
    m_monitorThread = std::thread(&ImagingCam::monitorThreadFunction, this);
    std::cout << "[Imaging Cam] Started ROI sharpness monitoring thread (5Hz, "
              << m_roiWidth << "x" << m_roiHeight << " ROI)" << std::endl;
}

void ImagingCam::stop()
{
    if (!m_running.load())
    {
        return; // Already stopped
    }

    m_running = false;
    if (m_monitorThread.joinable())
    {
        m_monitorThread.join();
    }
    std::cout << "[Imaging Cam] Stopped ROI sharpness monitoring thread" << std::endl;
}

void ImagingCam::setROISize(int width, int height)
{
    std::lock_guard<std::mutex> lock(m_roiMutex);
    m_roiWidth = width;
    m_roiHeight = height;
    std::cout << "[Imaging Cam] ROI size set to " << width << "x" << height << std::endl;
}

void ImagingCam::setROICenter(int centerX, int centerY)
{
    std::lock_guard<std::mutex> lock(m_roiMutex);
    m_roiCenterX = centerX;
    m_roiCenterY = centerY;
    m_useCustomCenter = true;
    std::cout << "[Imaging Cam] ROI center set to (" << centerX << ", " << centerY << ")" << std::endl;
}

void ImagingCam::getCurrentROI(int &centerX, int &centerY, int &width, int &height) const
{
    std::lock_guard<std::mutex> lock(m_roiMutex);
    centerX = m_roiCenterX.load();
    centerY = m_roiCenterY.load();
    width = m_roiWidth;
    height = m_roiHeight;
}

void ImagingCam::startROIFocusSearch()
{
    if (m_focusSearchActive.load())
    {
        std::cout << "[Imaging Cam] Focus search already in progress" << std::endl;
        return;
    }

    if (!m_useCustomCenter.load())
    {
        std::cout << "[Imaging Cam] No ROI selected, cannot start focus search" << std::endl;
        return;
    }

    // Ensure we have a valid frame before starting search
    VidFrame *testFrame = m_system.getFrame();
    if (!testFrame || !testFrame->data() || testFrame->size().x <= 0 || testFrame->size().y <= 0)
    {
        std::cout << "[Imaging Cam] No valid frame available, cannot start focus search" << std::endl;
        return;
    }

    std::cout << "[Imaging Cam] Starting ROI focus search..." << std::endl;
    m_focusSearchRequested = true;
}

void ImagingCam::monitorThreadFunction()
{
    while (m_running.load())
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastProcessTime);

        // Check if focus search is requested
        if (m_focusSearchRequested.load() && !m_focusSearchActive.load())
        {
            m_focusSearchRequested = false;
            m_focusSearchActive = true;

            try
            {
                // Perform focus search in a separate thread to avoid blocking monitoring
                std::thread searchThread(&ImagingCam::performFocusSearch, this);
                searchThread.detach();
            }
            catch (const std::exception &e)
            {
                std::cerr << "[Imaging Cam] Error starting focus search thread: " << e.what() << std::endl;
                m_focusSearchActive = false;
            }
        }

        if (elapsed >= m_processInterval)
        {
            m_lastProcessTime = now;

            // Get current frame from the system
            VidFrame *currentFrame = m_system.getFrame();

            // Add safety checks for null frame and data
            if (currentFrame && currentFrame->data() && currentFrame->size().x > 0 && currentFrame->size().y > 0)
            {
                try
                {
                    // Convert VidFrame to OpenCV Mat
                    cv::Mat frame(currentFrame->size().y, currentFrame->size().x, CV_8UC1,
                                  (void *)currentFrame->data());

                    if (!frame.empty())
                    {
                        // Extract ROI
                        cv::Mat roi = extractROI(frame);

                        if (!roi.empty())
                        {
                            // Calculate sharpness
                            double sharpness = calculateSharpness(roi);

                            // Print to console with timestamp and ROI info
                            auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                 std::chrono::system_clock::now().time_since_epoch())
                                                 .count();

                            int centerX, centerY, width, height;
                            getCurrentROI(centerX, centerY, width, height);

                            std::string searchStatus = m_focusSearchActive.load() ? " [SEARCHING]" : "";

                            std::cout << "[Imaging Cam] Time: " << timestamp
                                      << "ms, ROI(" << centerX << "," << centerY << "," << width << "x" << height
                                      << "), Sharpness: " << std::fixed << std::setprecision(2)
                                      << sharpness << searchStatus << std::endl;
                        }
                    }
                }
                catch (const std::exception &e)
                {
                    std::cerr << "[Imaging Cam] Error processing frame: " << e.what() << std::endl;
                }
            }
        }

        // Sleep for a short time to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void ImagingCam::performFocusSearch()
{
    std::cout << "[Imaging Cam] Starting gentle hill-climbing focus search" << std::endl;

    // Disable any existing autofocus modes first, then enable Hold Focus
    bFindFocus = false;
    bHoldFocus = true; // Enable Hold Focus directly via global variable

    // Notify GUI that Hold Focus is now enabled
    if (m_holdFocusCallback)
    {
        m_holdFocusCallback(true);
    }

    // Small delay to ensure Hold Focus is properly activated
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // Hill-climbing parameters with adaptive step sizing
    int stepSize = 20;             // Start with larger steps for speed
    const int MIN_STEP_SIZE = 5;   // Minimum step size for fine tuning
    const int MAX_STEPS = 50;      // Maximum number of steps to prevent infinite loops
    const int BACKTRACK_STEPS = 1; // Reduced backtrack for speed

    // Get current position and sharpness
    int currentPosition = desiredLocBestFocus;
    setFocusPositionAndWait(currentPosition);
    double currentSharpness = getAveragedROISharpness(); // Use averaged measurement

    std::cout << "[Imaging Cam] Starting position: " << currentPosition
              << ", initial sharpness: " << currentSharpness << std::endl;

    // Try moving in positive direction first
    int direction = 1; // 1 for positive, -1 for negative
    bool hasImproved = true;
    int steps = 0;
    int bestPosition = currentPosition;
    double bestSharpness = currentSharpness;
    int stepsFromBest = 0;

    while (hasImproved && steps < MAX_STEPS && m_running.load())
    {
        steps++;

        // Calculate next position using current step size
        int nextPosition = currentPosition + (direction * stepSize);

        // Clamp to valid range
        nextPosition = std::max(FOCUS_MIN, std::min(nextPosition, FOCUS_MAX));

        // If we hit the boundary, reverse direction and try the other way
        if (nextPosition == currentPosition)
        {
            if (direction == 1)
            {
                direction = -1;
                std::cout << "[Imaging Cam] Hit upper boundary, reversing direction" << std::endl;
                continue;
            }
            else
            {
                std::cout << "[Imaging Cam] Hit lower boundary, stopping search" << std::endl;
                break;
            }
        }

        // Move to next position and measure sharpness
        setFocusPositionAndWait(nextPosition);
        double nextSharpness = getAveragedROISharpness(); // Use averaged measurement

        std::cout << "[Imaging Cam] Step " << steps << ": position " << nextPosition
                  << " (step=" << stepSize << ", direction " << (direction > 0 ? "+" : "-")
                  << "), sharpness: " << nextSharpness << " (was " << currentSharpness << ")" << std::endl;

        // Check if we improved
        if (nextSharpness > currentSharpness)
        {
            // Improved! Continue in this direction
            currentPosition = nextPosition;
            currentSharpness = nextSharpness;

            // Update best position if this is the best so far
            if (nextSharpness > bestSharpness)
            {
                bestPosition = nextPosition;
                bestSharpness = nextSharpness;
                stepsFromBest = 0;
            }
            else
            {
                stepsFromBest++;
            }

            std::cout << "[Imaging Cam] Improved! Continuing in direction "
                      << (direction > 0 ? "+" : "-") << std::endl;
        }
        else
        {
            // Did not improve - try reducing step size for fine tuning
            if (stepSize > MIN_STEP_SIZE)
            {
                stepSize = stepSize / 2; // Halve the step size
                std::cout << "[Imaging Cam] No improvement, reducing step size to " << stepSize << std::endl;
                continue; // Try again with smaller step
            }

            // Already at minimum step size
            if (direction == 1)
            {
                // Try the negative direction, reset step size
                direction = -1;
                stepSize = 20; // Reset to larger step size for new direction
                std::cout << "[Imaging Cam] No improvement in positive direction, trying negative (step=" << stepSize << ")" << std::endl;
            }
            else
            {
                // Already tried both directions, we're done
                std::cout << "[Imaging Cam] No improvement in either direction, search complete" << std::endl;
                hasImproved = false;
            }
        }
    }

    // Go back to the best position, potentially backing off a few steps
    int finalPosition = bestPosition;
    if (stepsFromBest > 0)
    {
        // We moved past the peak, go back to the best position
        finalPosition = bestPosition;
        std::cout << "[Imaging Cam] Returning to best position: " << finalPosition << std::endl;
    }
    else if (steps >= 2)
    {
        // Back off a small amount from current position to be conservative
        finalPosition = currentPosition - (direction * MIN_STEP_SIZE * BACKTRACK_STEPS);
        finalPosition = std::max(FOCUS_MIN, std::min(finalPosition, FOCUS_MAX));
        std::cout << "[Imaging Cam] Backing off " << BACKTRACK_STEPS
                  << " small steps to position: " << finalPosition << std::endl;
    }

    // Set final optimal position
    setFocusPositionAndWait(finalPosition);
    double finalSharpness = getAveragedROISharpness(); // Final measurement with averaging for accuracy

    std::cout << "[Imaging Cam] Hill-climbing search complete! Final position: " << finalPosition
              << ", final sharpness: " << finalSharpness << " (after " << steps << " steps)" << std::endl;

    // Enable hold focus mode at the optimal position
    desiredLocBestFocus = finalPosition;
    bHoldFocus = true; // Keep Hold Focus enabled

    // Notify GUI of the new best focus position
    if (m_bestFocusCallback)
    {
        m_bestFocusCallback(finalPosition);
    }

    std::cout << "[Imaging Cam] Hold focus enabled at position " << finalPosition << std::endl;

    m_focusSearchActive = false;
}

void ImagingCam::setFocusPositionAndWait(int position)
{
    // Clamp position to valid range
    position = std::max(FOCUS_MIN, std::min(position, FOCUS_MAX));

    std::cout << "[Imaging Cam] Setting focus position to " << position
              << ", bHoldFocus=" << (bHoldFocus ? "true" : "false")
              << ", waiting " << SETTLE_TIME_MS << "ms..." << std::endl;

    // Set the desired focus position directly via global variable
    // This avoids thread-safety issues with GUI calls
    desiredLocBestFocus = position;

    // Notify GUI of the position change
    if (m_bestFocusCallback)
    {
        m_bestFocusCallback(position);
    }

    // Wait for lens to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
}

double ImagingCam::getCurrentROISharpness()
{
    VidFrame *currentFrame = m_system.getFrame();

    if (!currentFrame || !currentFrame->data() || currentFrame->size().x <= 0 || currentFrame->size().y <= 0)
    {
        std::cerr << "[Imaging Cam] No valid frame available for sharpness calculation" << std::endl;
        return 0.0;
    }

    try
    {
        // Convert VidFrame to OpenCV Mat
        cv::Mat frame(currentFrame->size().y, currentFrame->size().x, CV_8UC1,
                      (void *)currentFrame->data());

        // Extract ROI
        cv::Mat roi = extractROI(frame);

        if (roi.empty())
        {
            std::cerr << "[Imaging Cam] Failed to extract ROI for sharpness calculation" << std::endl;
            return 0.0;
        }

        // Calculate and return sharpness
        double sharpness = calculateSharpness(roi);
        return sharpness;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Imaging Cam] Error calculating ROI sharpness: " << e.what() << std::endl;
        return 0.0;
    }
}

double ImagingCam::getAveragedROISharpness(int numSamples)
{
    std::vector<double> samples;
    samples.reserve(numSamples);

    for (int i = 0; i < numSamples; i++)
    {
        double sharpness = getCurrentROISharpness();
        if (sharpness > 0.0)
        {
            samples.push_back(sharpness);
        }

        // Small delay between samples to get different frames
        if (i < numSamples - 1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60fps
        }
    }

    if (samples.empty())
    {
        return 0.0;
    }

    // Return the average
    double sum = 0.0;
    for (double sample : samples)
    {
        sum += sample;
    }
    return sum / samples.size();
}

cv::Mat ImagingCam::extractROI(const cv::Mat &frame)
{
    if (frame.empty())
    {
        return cv::Mat();
    }

    int centerX, centerY;

    if (m_useCustomCenter.load())
    {
        centerX = m_roiCenterX.load();
        centerY = m_roiCenterY.load();
    }
    else
    {
        // Use frame center as default
        centerX = frame.cols / 2;
        centerY = frame.rows / 2;
    }

    int roiX = centerX - m_roiWidth / 2;
    int roiY = centerY - m_roiHeight / 2;

    // Ensure ROI is within frame bounds
    roiX = std::max(0, std::min(roiX, frame.cols - m_roiWidth));
    roiY = std::max(0, std::min(roiY, frame.rows - m_roiHeight));

    // Ensure ROI dimensions are valid
    int actualWidth = std::min(m_roiWidth, frame.cols - roiX);
    int actualHeight = std::min(m_roiHeight, frame.rows - roiY);

    if (actualWidth <= 0 || actualHeight <= 0)
    {
        return cv::Mat();
    }

    cv::Rect roiRect(roiX, roiY, actualWidth, actualHeight);
    return frame(roiRect).clone(); // Clone to ensure data ownership
}

double ImagingCam::calculateSharpness(const cv::Mat &region)
{
    if (region.empty())
    {
        return 0.0;
    }

    // Use Tenengrad method (similar to autofocus.cpp)
    cv::Mat gradX, gradY;
    cv::Sobel(region, gradX, CV_16S, 1, 0, 3); // 3x3 Sobel for X direction
    cv::Sobel(region, gradY, CV_16S, 0, 1, 3); // 3x3 Sobel for Y direction

    // Square the gradients and sum them (Tenengrad formula: Gx² + Gy²)
    cv::Mat gradX_squared, gradY_squared;
    cv::multiply(gradX, gradX, gradX_squared);
    cv::multiply(gradY, gradY, gradY_squared);

    cv::Mat tenengrad;
    cv::add(gradX_squared, gradY_squared, tenengrad);

    // Return the mean of the Tenengrad values as sharpness score
    cv::Scalar meanSharpness = cv::mean(tenengrad);
    return meanSharpness[0];
}