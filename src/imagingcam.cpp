#include "imagingcam.hpp"
#include "system.hpp"
#include "vidframe.hpp"
#include "main.hpp"
#include "sdlwindow.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <glibmm.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>

// External global variables from autofocus.cpp
extern int desiredLocBestFocus;
extern bool bHoldFocus;
extern bool bFindFocus;
extern std::atomic<double> currentMeasuredFocus;

ImagingCam::ImagingCam(System &system)
    : m_system(system), m_running(false), m_roiWidth(150), m_roiHeight(150),
      m_roiCenterX(-1), m_roiCenterY(-1), m_useCustomCenter(false),
      m_lastProcessTime(std::chrono::steady_clock::now()),
      m_focusSearchActive(false), m_focusSearchRequested(false),
      m_depthMappingActive(false), m_depthMappingRequested(false)
{
    std::cout << "[Imaging Cam] Ready. Double-click on video to set ROI and start sharpness analysis." << std::endl;
    // Remove any automatic thread start - it will be started on double-click
}
ImagingCam::~ImagingCam()
{
    // Stop any active focus search and depth mapping first
    m_focusSearchActive = false;
    m_focusSearchRequested = false;
    m_depthMappingActive = false;
    m_depthMappingRequested = false;

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

void ImagingCam::startDepthMapping()
{
    if (m_depthMappingActive.load())
    {
        std::cout << "[Imaging Cam] Depth mapping already in progress" << std::endl;
        return;
    }

    // Ensure we have a valid frame before starting depth mapping
    VidFrame *testFrame = m_system.getFrame();
    if (!testFrame || !testFrame->data() || testFrame->size().x <= 0 || testFrame->size().y <= 0)
    {
        std::cout << "[Imaging Cam] No valid frame available, cannot start depth mapping" << std::endl;
        return;
    }

    // Start the monitoring thread if it's not already running
    if (!m_running.load())
    {
        start();
        std::cout << "[Imaging Cam] Started monitoring thread for depth mapping" << std::endl;
    }

    std::cout << "[Imaging Cam] Starting depth mapping..." << std::endl;
    m_depthMappingRequested = true;
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

        // Check if depth mapping is requested
        if (m_depthMappingRequested.load() && !m_depthMappingActive.load())
        {
            m_depthMappingRequested = false;
            m_depthMappingActive = true;

            try
            {
                // Perform depth mapping in a separate thread to avoid blocking monitoring
                std::thread depthThread(&ImagingCam::performDepthMapping, this);
                depthThread.detach();
            }
            catch (const std::exception &e)
            {
                std::cerr << "[Imaging Cam] Error starting depth mapping thread: " << e.what() << std::endl;
                m_depthMappingActive = false;
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

                                // Print to console with timestamp and ACTUAL analyzed ROI coordinates
                                auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                     std::chrono::system_clock::now().time_since_epoch())
                                                     .count();

                                // Get the original ROI coordinates
                                int origCenterX, origCenterY, width, height;
                                getCurrentROI(origCenterX, origCenterY, width, height);

                                // Calculate the actual analyzed coordinates (same logic as extractROI)
                                int actualCenterX = origCenterX;
                                int actualCenterY = origCenterY;
                                double stabOffsetX, stabOffsetY;
                                bool stabActive = m_system.getStabilizationOffset(stabOffsetX, stabOffsetY);
                                if (stabActive)
                                {
                                    actualCenterX = static_cast<int>(origCenterX - stabOffsetX);
                                    actualCenterY = static_cast<int>(origCenterY - stabOffsetY);
                                }

                                std::string searchStatus = m_focusSearchActive.load() ? " [SEARCHING]" : "";

                                std::cout << "[Imaging Cam] Time: " << timestamp
                                          << "ms, ROI(" << actualCenterX << "," << actualCenterY << "," << width << "x" << height
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

    // Try to use depth map for instant focus first
    if (tryDepthMapFocus())
    {
        // Depth map focus was successful, we're done!
        m_focusSearchActive = false;
        return;
    }

    // No depth map available or no valid data in ROI, proceed with hill-climbing search
    std::cout << "[Focus Search] No depth map available, proceeding with hill-climbing search..." << std::endl;

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

        // Apply stabilization offset to track the same feature when stabilization is active
        double stabOffsetX, stabOffsetY;
        if (m_system.getStabilizationOffset(stabOffsetX, stabOffsetY))
        {
            int originalX = centerX, originalY = centerY;
            // Adjust ROI coordinates OPPOSITE to stabilization offset to track the same feature
            // (if frame rendering moves down, ROI analysis should move up to stay on same feature)
            centerX = static_cast<int>(centerX - stabOffsetX);
            centerY = static_cast<int>(centerY - stabOffsetY);
        }
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

void ImagingCam::performLocalMaxSuppression(std::vector<std::vector<std::pair<double, double>>> &depthImage,
                                            int width, int height)
{
    const int REGION_SIZE = 8;                // 8x8 pixel regions - fairly fine grid as requested
    const double FOCUS_TOLERANCE = 10.0;      // ±5.0 focus positions as requested
    const double MIN_FOCUS_SEPARATION = 30.0; // Minimum separation to consider as different peak
    const double SECOND_PEAK_THRESHOLD = 0.7; // Second peak must be at least 70% of max sharpness

    int regionsProcessed = 0;
    int pixelsSuppressed = 0;
    int totalValidPixelsBefore = 0;

    // Count initial valid pixels
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (depthImage[y][x].first > 0)
            {
                totalValidPixelsBefore++;
            }
        }
    }

    std::cout << "[Depth Mapping] Starting local max suppression on "
              << totalValidPixelsBefore << " valid pixels..." << std::endl;

    // Process each region
    for (int regionY = 0; regionY < height; regionY += REGION_SIZE)
    {
        for (int regionX = 0; regionX < width; regionX += REGION_SIZE)
        {
            regionsProcessed++;

            // Define region boundaries
            int endX = std::min(regionX + REGION_SIZE, width);
            int endY = std::min(regionY + REGION_SIZE, height);

            // Find all valid pixels in this region
            std::vector<std::tuple<int, int, double, double>> validPixels; // x, y, sharpness, focus

            for (int y = regionY; y < endY; y++)
            {
                for (int x = regionX; x < endX; x++)
                {
                    const auto &pixel = depthImage[y][x];
                    if (pixel.first > 0) // Valid pixel with depth data
                    {
                        validPixels.push_back({x, y, pixel.first, pixel.second});
                    }
                }
            }

            if (validPixels.empty())
                continue;

            // Find pixel with maximum sharpness (primary peak)
            auto maxIt = std::max_element(validPixels.begin(), validPixels.end(),
                                          [](const auto &a, const auto &b)
                                          {
                                              return std::get<2>(a) < std::get<2>(b); // Compare sharpness values
                                          });

            double maxSharpness = std::get<2>(*maxIt);
            double primaryFocus = std::get<3>(*maxIt);

            // Look for potential second peak (different depth layer in same region)
            double secondaryFocus = -1.0;
            double secondarySharpness = 0.0;

            for (const auto &pixel : validPixels)
            {
                double focus = std::get<3>(pixel);
                double sharpness = std::get<2>(pixel);

                // Check if this could be a second peak
                if (std::abs(focus - primaryFocus) >= MIN_FOCUS_SEPARATION &&
                    sharpness >= maxSharpness * SECOND_PEAK_THRESHOLD &&
                    sharpness > secondarySharpness)
                {
                    secondaryFocus = focus;
                    secondarySharpness = sharpness;
                }
            }

            // Now suppress pixels that don't match either peak
            for (int y = regionY; y < endY; y++)
            {
                for (int x = regionX; x < endX; x++)
                {
                    auto &pixel = depthImage[y][x];
                    if (pixel.first > 0) // Valid pixel
                    {
                        double focusPos = pixel.second;
                        bool keepPixel = (std::abs(focusPos - primaryFocus) <= FOCUS_TOLERANCE);

                        // Also keep pixels that match secondary peak if it exists
                        if (secondaryFocus > 0)
                        {
                            keepPixel = keepPixel || (std::abs(focusPos - secondaryFocus) <= FOCUS_TOLERANCE);
                        }

                        if (!keepPixel)
                        {
                            pixel.first = -1.0; // Suppress this pixel
                            pixel.second = -1.0;
                            pixelsSuppressed++;
                        }
                    }
                }
            }
        }
    }

    int totalValidPixelsAfter = totalValidPixelsBefore - pixelsSuppressed;
    double suppressionRate = (double)pixelsSuppressed / totalValidPixelsBefore * 100.0;

    std::cout << "[Depth Mapping] Local max suppression complete: processed "
              << regionsProcessed << " regions (" << REGION_SIZE << "x" << REGION_SIZE << " pixels each)" << std::endl;
    std::cout << "[Depth Mapping] Suppressed " << pixelsSuppressed << " pixels ("
              << std::fixed << std::setprecision(1) << suppressionRate << "%), "
              << totalValidPixelsAfter << " pixels remaining" << std::endl;
}

void ImagingCam::performDepthMapping()
{
    std::cout << "\n[Depth Mapping] Starting depth mapping process..." << std::endl;

    // Note: Clearing and setup is now done in System::onGetDepthsClicked()
    // Just ensure other focus modes are disabled for this process
    bFindFocus = false;

    // Step 1: Set focus to leftmost position (Hold Focus already enabled in onGetDepthsClicked)
    std::cout << "[Depth Mapping] Setting focus to position " << FOCUS_MIN << "..." << std::endl;

    // Set focus position to minimum
    desiredLocBestFocus = FOCUS_MIN;

    // Update GUI with starting focus position
    if (m_bestFocusCallback)
    {
        m_bestFocusCallback(FOCUS_MIN);
    }

    // Wait for lens to settle (300ms)
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // XY stabilization is already enabled in System::onGetDepthsClicked()
    std::cout << "[Depth Mapping] XY stabilization already enabled" << std::endl;

    // Small delay to ensure stabilization is active
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Get frame dimensions from first frame
    VidFrame *firstFrame = m_system.getFrame();
    if (!firstFrame || !firstFrame->data())
    {
        std::cout << "[Depth Mapping] Failed to get initial frame, aborting" << std::endl;
        m_depthMappingActive = false;
        return;
    }

    int originalWidth = firstFrame->size().x;
    int originalHeight = firstFrame->size().y;

    // Calculate reduced dimensions (1/4 in each dim = 1/16 of original size)
    int reducedWidth = originalWidth / 4;
    int reducedHeight = originalHeight / 4;

    // Initialize depth image with reduced dimensions
    std::vector<std::vector<std::pair<double, double>>> depthImage(
        reducedHeight, std::vector<std::pair<double, double>>(reducedWidth, {-1.0, -1.0}));

    // Store all sharpness values for adaptive thresholding
    std::vector<std::tuple<int, int, double, double>> allPixelData; // x, y, sharpness, focus_position

    // Constants for depth mapping
    const int FOCUS_START = FOCUS_MIN;
    const int FOCUS_END = FOCUS_MAX;
    const int FOCUS_STEP = 5;

    // Reserve space for all pixel data (estimate)
    allPixelData.reserve(reducedWidth * reducedHeight * ((FOCUS_END - FOCUS_START) / FOCUS_STEP + 1));

    // Process each focus position
    for (int focusPos = FOCUS_START; focusPos <= FOCUS_END; focusPos += FOCUS_STEP)
    {
        if (!m_running.load() || !m_depthMappingActive.load())
        {
            std::cout << "[Depth Mapping] Stopped early at focus position " << focusPos << std::endl;
            break;
        }

        std::cout << "[Depth Mapping] Processing focus position " << focusPos << " ("
                  << (focusPos - FOCUS_START) / FOCUS_STEP + 1 << "/"
                  << (FOCUS_END - FOCUS_START) / FOCUS_STEP + 1 << ")" << std::endl;

        // Set new focus position
        desiredLocBestFocus = focusPos;
        if (m_bestFocusCallback)
        {
            m_bestFocusCallback(focusPos);
        }

        // Wait for lens to settle (40ms for subsequent positions)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Get current frame and stabilization offset
        VidFrame *currentFrame = m_system.getFrame();
        if (!currentFrame || !currentFrame->data())
        {
            std::cout << "[Depth Mapping] Failed to get frame at position " << focusPos << ", skipping" << std::endl;
            continue;
        }

        double stabOffsetX, stabOffsetY;
        bool hasStabOffset = m_system.getStabilizationOffset(stabOffsetX, stabOffsetY);

        // Convert frame to OpenCV Mat
        cv::Mat originalFrame(originalHeight, originalWidth, CV_8UC1, currentFrame->data());

        // Create a stabilized frame by cropping and padding to maintain original dimensions
        cv::Mat stabilizedFrame = cv::Mat::zeros(originalHeight, originalWidth, CV_8UC1);

        // Create a mask to track valid (non-padded) regions
        cv::Mat validMask = cv::Mat::zeros(originalHeight, originalWidth, CV_8UC1);

        if (hasStabOffset)
        {
            // Calculate the region to extract to get the same physical content
            // If camera moved right by offsetX, extract from left by offsetX to get same content
            int extractX = static_cast<int>(-stabOffsetX);
            int extractY = static_cast<int>(-stabOffsetY);

            // Calculate valid source region in original frame
            int srcStartX = std::max(0, extractX);
            int srcStartY = std::max(0, extractY);
            int srcEndX = std::min(originalWidth, extractX + originalWidth);
            int srcEndY = std::min(originalHeight, extractY + originalHeight);

            // Calculate corresponding destination region in stabilized frame
            int dstStartX = srcStartX - extractX;
            int dstStartY = srcStartY - extractY;

            int validWidth = srcEndX - srcStartX;
            int validHeight = srcEndY - srcStartY;

            if (validWidth > 0 && validHeight > 0)
            {
                // Extract valid region from original frame
                cv::Rect srcRect(srcStartX, srcStartY, validWidth, validHeight);
                cv::Mat validRegion = originalFrame(srcRect);

                // Copy to appropriate position in stabilized frame (rest remains zeros)
                cv::Rect dstRect(dstStartX, dstStartY, validWidth, validHeight);
                validRegion.copyTo(stabilizedFrame(dstRect));

                // Mark this region as valid in the mask
                validMask(dstRect).setTo(255);
            }
            // If no valid region, stabilizedFrame and validMask remain all zeros
        }
        else
        {
            // No stabilization offset, copy full frame and mark all as valid
            originalFrame.copyTo(stabilizedFrame);
            validMask.setTo(255);
        }

        // Resize both the stabilized frame and valid mask to reduced dimensions
        cv::Mat resizedFrame, resizedMask;
        cv::resize(stabilizedFrame, resizedFrame, cv::Size(reducedWidth, reducedHeight));
        cv::resize(validMask, resizedMask, cv::Size(reducedWidth, reducedHeight));

        // Apply sophisticated padding to eliminate edge artifacts
        cv::Mat maskedFrame = createSmoothedPaddedFrame(resizedFrame, resizedMask);

        // Calculate sharpness image using RobertsCross operator on the masked frame
        cv::Mat sharpnessImage = calculateSharpnessImage(maskedFrame);

        // Get actual measured focus position from autofocus system
        double actualMeasuredFocus = currentMeasuredFocus.load();

        // Process each pixel in the sharpness image
        for (int y = 0; y < reducedHeight; y++)
        {
            for (int x = 0; x < reducedWidth; x++)
            {
                // Only process pixels that are in valid (non-padded) regions
                if (resizedMask.at<uchar>(y, x) > 128) // Valid region
                {
                    double sharpness = sharpnessImage.at<float>(y, x);

                    // Store all pixel data for later adaptive thresholding
                    if (sharpness > 0) // Only store non-zero sharpness values
                    {
                        allPixelData.push_back({x, y, sharpness, actualMeasuredFocus});
                    }
                }
            }
        }
    }

    std::cout << "[Depth Mapping] Focus scanning complete! Applying adaptive thresholding..." << std::endl;

    // Apply adaptive thresholding using 80th percentile
    if (!allPixelData.empty())
    {
        // Create a vector of just the sharpness values for percentile calculation
        std::vector<double> sharpnessValues;
        sharpnessValues.reserve(allPixelData.size());
        for (const auto &pixel : allPixelData)
        {
            sharpnessValues.push_back(std::get<2>(pixel)); // Extract sharpness value
        }

        // Find 80th percentile using nth_element (O(n) average case, much faster than sorting)
        const double PERCENTILE = 0.8; // 80th percentile
        size_t percentileIndex = static_cast<size_t>(sharpnessValues.size() * PERCENTILE);
        if (percentileIndex >= sharpnessValues.size())
            percentileIndex = sharpnessValues.size() - 1;

        std::nth_element(sharpnessValues.begin(),
                         sharpnessValues.begin() + percentileIndex,
                         sharpnessValues.end());
        double adaptiveThreshold = sharpnessValues[percentileIndex];

        std::cout << "[Depth Mapping] Adaptive threshold (80th percentile): " << adaptiveThreshold
                  << " (from " << allPixelData.size() << " candidate pixels)" << std::endl;

        // Now apply the adaptive threshold and populate the depth image
        int pixelsAboveThreshold = 0;
        for (const auto &pixelData : allPixelData)
        {
            int x = std::get<0>(pixelData);
            int y = std::get<1>(pixelData);
            double sharpness = std::get<2>(pixelData);
            double focusPosition = std::get<3>(pixelData);

            if (sharpness >= adaptiveThreshold)
            {
                auto &currentPixel = depthImage[y][x];
                if (currentPixel.first < 0 || sharpness > currentPixel.first)
                {
                    currentPixel.first = sharpness;
                    currentPixel.second = focusPosition;
                }
                pixelsAboveThreshold++;
            }
        }

        std::cout << "[Depth Mapping] Kept " << pixelsAboveThreshold << " pixels above adaptive threshold ("
                  << (100.0 * pixelsAboveThreshold / allPixelData.size()) << "% of candidates)" << std::endl;
    }

    // Clear pixel data to free memory
    allPixelData.clear();
    allPixelData.shrink_to_fit();

    std::cout << "[Depth Mapping] Depth mapping complete! Generated " << reducedWidth << "x"
              << reducedHeight << " depth map" << std::endl;

    // Perform local max suppression to clean up artifacts
    performLocalMaxSuppression(depthImage, reducedWidth, reducedHeight);

    // Count valid pixels and export to CSV BEFORE moving data to avoid segfault
    int totalValidPixels = 0;

    // Export depth map to CSV file first (before moving the data)
    try
    {
        // Create output directory if it doesn't exist
        std::string outputDir = "../output";

        std::cout << "[Depth Mapping] Creating output directory: " << outputDir << std::endl;
        std::filesystem::create_directories(outputDir);

        if (std::filesystem::exists(outputDir))
        {
            std::cout << "[Depth Mapping] Output directory exists/created successfully" << std::endl;
        }
        else
        {
            std::cout << "[Depth Mapping] ERROR: Failed to create output directory" << std::endl;
            m_depthMappingActive = false;
            return;
        }

        // Use specific filename as requested
        std::string filename = outputDir + "/depth_map.csv";

        std::cout << "[Depth Mapping] Attempting to create CSV file: " << filename << std::endl;

        std::ofstream csvFile(filename);
        if (csvFile.is_open())
        {
            // Write header
            csvFile << "x,y,sharpness,focus_position\n";

            // Write data and count valid pixels in one pass
            for (int y = 0; y < reducedHeight; y++)
            {
                for (int x = 0; x < reducedWidth; x++)
                {
                    const auto &pixel = depthImage[y][x];
                    if (pixel.first > 0) // Valid pixel with depth data
                    {
                        totalValidPixels++;
                        // Use the actual measured focus position stored in the depth map
                        double actualFocusPosition = pixel.second;

                        csvFile << x << "," << y << "," << pixel.first << ","
                                << actualFocusPosition << "\n";
                    }
                }
            }
            csvFile.flush(); // Ensure data is written
            csvFile.close();

            std::cout << "[Depth Mapping] Successfully exported " << totalValidPixels << " depth points to: " << filename << std::endl;
        }
        else
        {
            std::cout << "[Depth Mapping] ERROR: Could not create CSV file: " << filename << std::endl;
            std::cout << "[Depth Mapping] Check directory permissions and disk space" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[Depth Mapping] ERROR: Exception during CSV export: " << e.what() << std::endl;
    }

    std::cout << "[Depth Mapping] Total valid pixels with depth data: " << totalValidPixels << std::endl;

    // Transfer depth map data to SDL window for visualization BEFORE moving to system storage
    extern SDLWindow::SDLWin *childwin;
    if (childwin)
    {
        try
        {
            SDLWindow::transferDepthMapData(childwin, depthImage, reducedWidth, reducedHeight);
            std::cout << "[Depth Mapping] Depth map data transferred to SDL window for visualization" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cout << "[Depth Mapping] ERROR: Exception during SDL transfer: " << e.what() << std::endl;
        }
    }
    else
    {
        std::cout << "[Depth Mapping] SDL window not available for depth map transfer" << std::endl;
    }

    // Now safely store depth map in system (after CSV export and SDL transfer)
    System::DepthMapData &depthData = m_system.currentDepthMap;
    depthData.depthImage = std::move(depthImage); // Safe to move now
    depthData.width = reducedWidth;
    depthData.height = reducedHeight;
    depthData.isValid = true;

    // pause 100ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // return desiredLocBestFocus to the 'Find Focus' position (max sharpness)
    desiredLocBestFocus = 350;
    if (m_bestFocusCallback)
    {
        m_bestFocusCallback(350);
    }

    std::cout << "[Depth Mapping] Depth map data stored in system successfully" << std::endl;

    m_depthMappingActive = false;
}

cv::Mat ImagingCam::createSmoothedPaddedFrame(const cv::Mat &frame, const cv::Mat &validMask)
{
    if (frame.empty() || validMask.empty())
    {
        return frame.clone();
    }

    cv::Mat result = frame.clone();

    // Method 1: Use inpainting for sophisticated filling of invalid regions
    cv::Mat invalidMask = ~validMask; // Invert mask to mark regions to fill

    // Dilate the invalid mask slightly to ensure we catch edge pixels
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::Mat dilatedInvalidMask;
    cv::dilate(invalidMask, dilatedInvalidMask, kernel);

    // Use Navier-Stokes based inpainting for natural-looking fill
    cv::Mat inpaintedFrame;
    cv::inpaint(result, dilatedInvalidMask, inpaintedFrame, 3, cv::INPAINT_NS);

    // Method 2: Create distance-based blending for smooth transitions
    cv::Mat distanceTransform;
    cv::distanceTransform(validMask, distanceTransform, cv::DIST_L2, 3);

    // Normalize distance transform to create blending weights
    cv::Mat blendWeights;
    cv::normalize(distanceTransform, blendWeights, 0.0, 1.0, cv::NORM_MINMAX);

    // Create transition zone near boundaries (within 5 pixels of edge)
    cv::Mat transitionZone;
    cv::threshold(blendWeights, transitionZone, 0.0, 1.0, cv::THRESH_BINARY);
    cv::threshold(blendWeights, transitionZone, 5.0, 1.0, cv::THRESH_BINARY_INV);

    // Apply Gaussian blur to transition zones only
    cv::Mat blurredFrame;
    cv::GaussianBlur(inpaintedFrame, blurredFrame, cv::Size(5, 5), 1.5);

    // Blend original and blurred versions based on distance from edge
    cv::Mat finalFrame;
    blendWeights.convertTo(blendWeights, CV_32F);

    // Convert frames to float for blending
    cv::Mat inpaintedFloat, blurredFloat;
    inpaintedFrame.convertTo(inpaintedFloat, CV_32F);
    blurredFrame.convertTo(blurredFloat, CV_32F);

    // Create smooth blend: use original far from edges, blurred near edges
    cv::Mat blendWeights3;
    cv::normalize(blendWeights, blendWeights3, 0.3, 1.0, cv::NORM_MINMAX); // Minimum 30% original

    finalFrame = blendWeights3.mul(inpaintedFloat) + (1.0 - blendWeights3).mul(blurredFloat);

    // Convert back to original type
    finalFrame.convertTo(result, frame.type());

    // Final step: Apply very light additional smoothing only at the boundary regions
    cv::Mat boundary;
    cv::erode(validMask, boundary, kernel);
    boundary = validMask - boundary; // Get boundary pixels only

    if (cv::countNonZero(boundary) > 0)
    {
        cv::Mat lightlyBlurred;
        cv::GaussianBlur(result, lightlyBlurred, cv::Size(3, 3), 0.8);
        lightlyBlurred.copyTo(result, boundary);
    }

    return result;
}

cv::Mat ImagingCam::calculateSharpnessImage(const cv::Mat &frame)
{
    if (frame.empty())
    {
        return cv::Mat();
    }

    // Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(2.0);
    clahe->setTilesGridSize(cv::Size(4, 4));
    cv::Mat clahe_enhanced;
    clahe->apply(frame, clahe_enhanced);

    // Gaussian Blur
    cv::Mat blurred;
    cv::GaussianBlur(clahe_enhanced, blurred, cv::Size(3, 3), 1, 1, cv::BORDER_DEFAULT);

    // Compute sharpness using Roberts Cross operator
    cv::Mat roberts_kernelx = (cv::Mat_<double>(2, 2) << 1, 0, 0, -1);
    cv::Mat roberts_kernely = (cv::Mat_<double>(2, 2) << 0, 1, -1, 0);

    cv::Mat img_x, img_y;
    cv::filter2D(blurred, img_x, CV_16S, roberts_kernelx);
    cv::filter2D(blurred, img_y, CV_16S, roberts_kernely);

    // Square the gradients and sum them
    cv::Mat img_x_squared, img_y_squared;
    cv::multiply(img_x, img_x, img_x_squared);
    cv::multiply(img_y, img_y, img_y_squared);

    cv::Mat sum_xy;
    cv::add(img_x_squared, img_y_squared, sum_xy);

    // Convert to float for precision
    cv::Mat sharpnessFloat;
    sum_xy.convertTo(sharpnessFloat, CV_32F);

    return sharpnessFloat;
}

cv::Mat ImagingCam::calculateSharpnessImageTenengrad(const cv::Mat &frame)
{
    if (frame.empty())
    {
        return cv::Mat();
    }

    // Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(2.0);
    clahe->setTilesGridSize(cv::Size(4, 4));
    cv::Mat clahe_enhanced;
    clahe->apply(frame, clahe_enhanced);

    // Gaussian Blur
    cv::Mat blurred;
    cv::GaussianBlur(clahe_enhanced, blurred, cv::Size(3, 3), 1, 1, cv::BORDER_DEFAULT);

    // Compute sharpness using Tenengrad method (Sobel operators)
    cv::Mat gradX, gradY;
    cv::Sobel(blurred, gradX, CV_16S, 1, 0, 3); // 3x3 Sobel for X direction
    cv::Sobel(blurred, gradY, CV_16S, 0, 1, 3); // 3x3 Sobel for Y direction

    // Square the gradients and sum them (Tenengrad formula: Gx² + Gy²)
    cv::Mat gradX_squared, gradY_squared;
    cv::multiply(gradX, gradX, gradX_squared);
    cv::multiply(gradY, gradY, gradY_squared);

    cv::Mat tenengrad;
    cv::add(gradX_squared, gradY_squared, tenengrad);

    // Convert to float for precision
    cv::Mat sharpnessFloat;
    tenengrad.convertTo(sharpnessFloat, CV_32F);

    return sharpnessFloat;
}

bool ImagingCam::tryDepthMapFocus()
{
    // Check if depth map is available in the system
    const auto &depthData = m_system.currentDepthMap;
    if (!depthData.isValid || depthData.depthImage.empty())
    {
        return false; // No depth map available
    }

    // Get current ROI parameters
    int roiCenterX, roiCenterY, roiWidth, roiHeight;
    getCurrentROI(roiCenterX, roiCenterY, roiWidth, roiHeight);

    if (roiCenterX < 0 || roiCenterY < 0)
    {
        return false; // No valid ROI set
    }

    // Map ROI coordinates to depth map coordinates (depth map is 1/4 resolution)
    int depthCenterX = roiCenterX / 4;
    int depthCenterY = roiCenterY / 4;
    int depthRoiWidth = std::max(1, roiWidth / 4);
    int depthRoiHeight = std::max(1, roiHeight / 4);

    // Calculate depth map ROI bounds
    int depthStartX = std::max(0, depthCenterX - depthRoiWidth / 2);
    int depthStartY = std::max(0, depthCenterY - depthRoiHeight / 2);
    int depthEndX = std::min(depthData.width, depthStartX + depthRoiWidth);
    int depthEndY = std::min(depthData.height, depthStartY + depthRoiHeight);

    // Find pixel with maximum sharpness in the ROI
    double maxSharpness = -1.0;
    double bestFocusPosition = -1.0;
    int validPixelsInROI = 0;

    for (int y = depthStartY; y < depthEndY; y++)
    {
        for (int x = depthStartX; x < depthEndX; x++)
        {
            const auto &pixel = depthData.depthImage[y][x];
            if (pixel.first > 0) // Valid pixel with depth data
            {
                validPixelsInROI++;
                if (pixel.first > maxSharpness)
                {
                    maxSharpness = pixel.first;
                    bestFocusPosition = pixel.second;
                }
            }
        }
    }

    if (validPixelsInROI == 0 || bestFocusPosition < 0)
    {
        std::cout << "[Imaging Cam] No valid depth data in ROI region, falling back to search" << std::endl;
        return false; // No valid depth data in ROI
    }

    // Clamp focus position to valid range
    int targetFocus = static_cast<int>(std::round(bestFocusPosition));
    targetFocus = std::max(FOCUS_MIN, std::min(targetFocus, FOCUS_MAX));

    std::cout << "[Imaging Cam] Using depth map: found " << validPixelsInROI
              << " valid pixels in ROI, max sharpness: " << maxSharpness
              << " at focus position: " << targetFocus << std::endl;

    // Set focus position using depth map data
    desiredLocBestFocus = targetFocus;

    // Update GUI with the new focus position
    if (m_bestFocusCallback)
    {
        m_bestFocusCallback(targetFocus);
    }

    // Wait for lens to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

    // Enable hold focus mode at the optimal position
    bHoldFocus = true;

    // Notify GUI that Hold Focus is enabled
    if (m_holdFocusCallback)
    {
        m_holdFocusCallback(true);
    }

    // Notify GUI that search is complete (always successful when using depth map)
    if (m_searchCompleteCallback)
    {
        m_searchCompleteCallback(true);
    }

    std::cout << "[Imaging Cam] Instant focus complete using depth map at position "
              << targetFocus << std::endl;

    return true; // Success
}