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
    : m_system(system), m_running(false), m_roiWidth(150), m_roiHeight(150),
      m_roiCenterX(-1), m_roiCenterY(-1), m_useCustomCenter(false),
      m_lastProcessTime(std::chrono::steady_clock::now()),
      m_focusSearchActive(false), m_focusSearchRequested(false)
{
    std::cout << "[Imaging Cam] Ready. Double-click on video to set ROI and start sharpness analysis." << std::endl;
    // Remove any automatic thread start - it will be started on double-click
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

            // Only process frames if a custom ROI center has been set
            if (m_useCustomCenter.load())
            {
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
        }

        // Sleep for a short time to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void ImagingCam::performFocusSearch()
{
    std::cout << "\n[Focus Search] Starting search..." << std::endl;

    // Disable any existing autofocus modes first, then enable Hold Focus
    bFindFocus = false;
    bHoldFocus = true; // Enable Hold Focus directly via global variable

    // Notify GUI that Hold Focus is now enabled
    if (m_holdFocusCallback)
    {
        m_holdFocusCallback(true);
    }

    // Small delay to ensure Hold Focus is properly activated
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Determine the best starting position for the search
    int searchStartPosition;

    // Try to get the current actual focus position from the system
    // If we have a reasonable current position, use it; otherwise default to 400
    if (desiredLocBestFocus >= FOCUS_MIN && desiredLocBestFocus <= FOCUS_MAX && desiredLocBestFocus != 0)
    {
        searchStartPosition = desiredLocBestFocus;
        std::cout << "[Focus Search] Starting from current position: " << searchStartPosition << std::endl;
    }
    else
    {
        searchStartPosition = 400;
        std::cout << "[Focus Search] Using default start position: " << searchStartPosition << std::endl;
    }

    // Hill-climbing parameters with adaptive step sizing
    int stepSize = 20;             // Start with larger steps for speed
    const int MIN_STEP_SIZE = 3;   // Minimum step size for fine tuning
    const int MAX_STEPS = 40;      // Maximum number of steps to prevent infinite loops
    const int BACKTRACK_STEPS = 3; // Reduced backtrack for speed

    // Use our captured starting position instead of the global variable
    int initialPosition = searchStartPosition;
    int currentPosition = initialPosition;

    // If starting from default position (400), use longer settle time since this could be a big move
    bool usingDefaultPosition = (searchStartPosition == 400 &&
                                 (desiredLocBestFocus < FOCUS_MIN || desiredLocBestFocus > FOCUS_MAX || desiredLocBestFocus == 0));

    if (usingDefaultPosition)
    {
        std::cout << "[Imaging Cam] Using default start position, waiting extra time for big move..." << std::endl;
        setFocusPositionAndWaitLong(currentPosition, 400); // 400ms settle time
    }
    else
    {
        setFocusPositionAndWait(currentPosition); // Normal 150ms settle time
    }

    double initialSharpness = getAveragedROISharpness(); // Use averaged measurement
    double currentSharpness = initialSharpness;

    // Set initial position as best position
    int bestPosition = currentPosition;
    double bestSharpness = initialSharpness;

    // If initial sharpness is very low, don't bother searching
    if (initialSharpness < 10.0)
    {
        std::cout << "[Focus Search] Initial sharpness too low, aborting" << std::endl;
        if (m_searchCompleteCallback)
        {
            m_searchCompleteCallback(false);
        }
        m_focusSearchActive = false;
        return;
    }

    // Improved algorithm: coarse search + fine refinement around peaks
    int steps = 0;
    bool foundPeakRegion = false;

    // Phase 1: Coarse search to find general peak region
    // std::cout << "[Imaging Cam] Phase 1: Coarse search with step size " << stepSize << std::endl;

    // First, try both directions from starting position to get oriented
    int leftPos = currentPosition - stepSize;
    int rightPos = currentPosition + stepSize;
    leftPos = std::max(FOCUS_MIN, std::min(leftPos, FOCUS_MAX));
    rightPos = std::max(FOCUS_MIN, std::min(rightPos, FOCUS_MAX));

    std::cout << "[Focus Search] Measuring positions: " << leftPos << " (left), "
              << currentPosition << " (current), " << rightPos << " (right)" << std::endl;

    double leftSharpness = 0, rightSharpness = 0;

    // Measure left position if valid
    if (leftPos != currentPosition && steps < MAX_STEPS)
    {
        steps++;
        setFocusPositionAndWait(leftPos);
        leftSharpness = getAveragedROISharpness();
        // std::cout << "[Imaging Cam] Step " << steps << ": position " << leftPos

        if (leftSharpness > bestSharpness)
        {
            bestPosition = leftPos;
            bestSharpness = leftSharpness;
        }
        std::cout << "[Focus Search] Left sharpness: " << leftSharpness << std::endl;
    }

    // Measure right position if valid
    if (rightPos != currentPosition && steps < MAX_STEPS)
    {
        steps++;
        setFocusPositionAndWait(rightPos);
        rightSharpness = getAveragedROISharpness();

        if (rightSharpness > bestSharpness)
        {
            bestPosition = rightPos;
            bestSharpness = rightSharpness;
        }
        std::cout << "[Focus Search] Right sharpness: " << rightSharpness << std::endl;
    }

    // Determine search direction based on initial measurements
    int direction = 0;
    if (rightSharpness > currentSharpness && rightSharpness > leftSharpness)
    {
        direction = 1; // Go right
        currentPosition = rightPos;
        currentSharpness = rightSharpness;
        std::cout << "[Imaging Cam] Moving right (positive direction)" << std::endl;
    }
    else if (leftSharpness > currentSharpness && leftSharpness > rightSharpness)
    {
        direction = -1; // Go left
        currentPosition = leftPos;
        currentSharpness = leftSharpness;
        std::cout << "[Imaging Cam] Moving left (negative direction)" << std::endl;
    }
    else
    {
        std::cout << "[Focus Search] Current position appears best, starting refinement" << std::endl;
        foundPeakRegion = true;
    }

    // Continue coarse search in chosen direction
    while (!foundPeakRegion && steps < MAX_STEPS && m_running.load())
    {
        int nextPosition = currentPosition + (direction * stepSize);
        nextPosition = std::max(FOCUS_MIN, std::min(nextPosition, FOCUS_MAX));

        if (nextPosition == currentPosition)
        {
            std::cout << "[Focus Search] Hit boundary at " << currentPosition << std::endl;
            foundPeakRegion = true;
            break;
        }

        steps++;
        setFocusPositionAndWait(nextPosition);
        double nextSharpness = getAveragedROISharpness();
        std::cout << "[Focus Search] Step " << steps << ": pos=" << nextPosition
                  << ", sharpness=" << nextSharpness << std::endl;

        if (nextSharpness > currentSharpness)
        {
            // Keep going in this direction
            currentPosition = nextPosition;
            currentSharpness = nextSharpness;

            if (nextSharpness > bestSharpness)
            {
                bestPosition = nextPosition;
                bestSharpness = nextSharpness;
            }
            std::cout << "[Focus Search] Improved to " << initialSharpness << std::endl;
        }
        else
        {
            std::cout << "[Focus Search] No improvement, starting refinement" << std::endl;
            foundPeakRegion = true;
        }
    }

    // Phase 2: Fine refinement around the best position found
    if (foundPeakRegion && steps < MAX_STEPS)
    {
        // std::cout << "[Imaging Cam] Phase 2: Fine refinement around position " << bestPosition << std::endl;

        // Start refinement from the best position found so far
        currentPosition = bestPosition;
        setFocusPositionAndWait(currentPosition);
        currentSharpness = getAveragedROISharpness();

        // Use progressively smaller step sizes for refinement
        int refineStep = stepSize / 2;
        while (refineStep >= MIN_STEP_SIZE && steps < MAX_STEPS && m_running.load())
        {
            // std::cout << "[Imaging Cam] Refining with step size " << refineStep << std::endl;
            bool improved = false;

            // Try both directions with current refine step
            for (int dir = -1; dir <= 1; dir += 2) // -1 and +1
            {
                if (steps >= MAX_STEPS)
                    break;

                int testPos = currentPosition + (dir * refineStep);
                testPos = std::max(FOCUS_MIN, std::min(testPos, FOCUS_MAX));

                if (testPos != currentPosition)
                {
                    steps++;
                    setFocusPositionAndWait(testPos);
                    double testSharpness = getAveragedROISharpness();

                    if (testSharpness > currentSharpness)
                    {
                        currentPosition = testPos;
                        currentSharpness = testSharpness;
                        improved = true;

                        if (testSharpness > bestSharpness)
                        {
                            bestPosition = testPos;
                            bestSharpness = testSharpness;
                        }

                        // std::cout << "[Imaging Cam] Improved during refinement!" << std::endl;
                        break; // Try this direction further before reducing step size
                    }
                }
            }

            if (!improved)
            {
                // No improvement at this step size, try smaller steps
                refineStep = refineStep / 2;
            }
        }
    }

    // Always return to the best position found during the search
    int finalPosition = bestPosition;

    // Safety check: if the best position found is significantly worse than initial,
    // return to initial position
    if (bestPosition != initialPosition && bestSharpness < (initialSharpness * 0.8))
    {
        finalPosition = initialPosition;
    }
    else
    {
        // std::cout << "[Imaging Cam] Returning to best position found: " << finalPosition << std::endl;
    }

    // Set final optimal position
    setFocusPositionAndWait(finalPosition);
    double finalSharpness = getAveragedROISharpness(); // Final measurement with averaging for accuracy

    // std::cout << "[Imaging Cam] Hill-climbing search complete! Final position: " << finalPosition
    //           << ", final sharpness: " << finalSharpness << " (initial: " << initialSharpness
    //           << ", best found: " << bestSharpness << ", after " << steps << " steps)" << std::endl;

    // Enable hold focus mode at the optimal position
    desiredLocBestFocus = finalPosition;
    bHoldFocus = true; // Keep Hold Focus enabled

    // Notify GUI of the new best focus position
    if (m_bestFocusCallback)
    {
        m_bestFocusCallback(finalPosition);
    }

    // Determine if search was successful (any meaningful improvement from initial)
    bool searchSuccessful = (bestSharpness > initialSharpness * 1.05); // 5% improvement threshold

    // Notify GUI that search is complete so it can turn the target red
    if (m_searchCompleteCallback)
    {
        m_searchCompleteCallback(searchSuccessful);
    }

    std::cout << "[Imaging Cam] Hold focus enabled at position " << finalPosition
              << " (search " << (searchSuccessful ? "successful" : "marginal") << ")" << std::endl;

    m_focusSearchActive = false;
}

void ImagingCam::setFocusPositionAndWait(int position)
{
    setFocusPositionAndWaitLong(position, SETTLE_TIME_MS);
}

void ImagingCam::setFocusPositionAndWaitLong(int position, int settleTimeMs)
{
    // Clamp position to valid range
    position = std::max(FOCUS_MIN, std::min(position, FOCUS_MAX));

    // std::cout << "[Imaging Cam] Setting focus position to " << position
    //           << ", bHoldFocus=" << (bHoldFocus ? "true" : "false")
    //           << ", waiting " << settleTimeMs << "ms..." << std::endl;

    // Set the desired focus position directly via global variable
    // This avoids thread-safety issues with GUI calls
    desiredLocBestFocus = position;

    // Don't notify GUI during focus search to prevent feedback loop
    // Only notify GUI when search is complete
    if (!m_focusSearchActive.load() && m_bestFocusCallback)
    {
        m_bestFocusCallback(position);
    }

    // Wait for lens to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(settleTimeMs));
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