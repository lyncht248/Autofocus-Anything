#include "autofocus.hpp"
#include "pid.hpp"
#include "tiltedcam.hpp"
#include "lens.hpp"
#include "logfile.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <atomic>
#include <mutex>

// Global variables to match actual implementation in autofocus.cpp
extern std::atomic<double> mmToMove;
extern bool bNewMoveRel;
extern int desiredLocBestFocus;

/**
 * @brief Simple PID tuning application that uses the actual autofocus code
 * 
 * This standalone application:
 * 1. Tests different Kp values on a step change (130 to 530 pixels)
 * 2. Uses the same control mechanism as the real autofocus.cpp
 * 3. Records the step response for analysis
 * 4. Generates data for plotting
 */
int main(int argc, char **argv) {
    // Initialize logger if not already done
    try {
        if (!logger) {
            logger = spdlog::basic_logger_mt("pid_tuning", "logs/pid_tuning.log");
            logger->set_level(spdlog::level::info);
        }
    } catch (const spdlog::spdlog_ex&) {
        // Logger already exists
    }

    // Create output directory
    system("mkdir -p ../output");

    // Initialize autofocus system
    autofocus focusController;
    if (!focusController.initialize()) {
        std::cerr << "Failed to initialize autofocus system!" << std::endl;
        return 1;
    }

    // Allow system to stabilize
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Get references to camera and lens
    tiltedcam& camera = focusController.getTiltedCam();
    lens& lensController = focusController.getLens();

    // Kp values to test (from smaller to larger)
    std::vector<double> kpValues = {0.0006, 0.0009, 0.0012, 0.0015, 0.0018, 0.0021};
    
    // Test parameters
    const int startFocus = 130;
    const int targetFocus = 530;
    const int tolerance = 6;  // Same tolerance used in autofocus.cpp
    const int testTimeout = 20000;  // 20 seconds in milliseconds
    
    // Results storage
    struct TestResult {
        double kp;
        std::vector<int> focusLocations;
        std::vector<double> moveCommands;
        std::vector<double> timestamps;
        int settlingTime;
        bool hasOvershoot;
        bool reachedTarget;
    };
    
    std::vector<TestResult> allResults;
    
    // Move lens to start position
    std::cout << "Moving lens to starting position..." << std::endl;
    lensController.setDesiredLensPosition(-9.1);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Scale factor for image processing
    double scale = 0.5;
    
    // Image buffer setup
    const int imgWidth = camera.getImageWidth();
    const int imgHeight = camera.getImageHeight();
    const long img_size = imgWidth * imgHeight;
    unsigned char* buffer = new unsigned char[img_size];
    
    // Create a separate thread for camera frame processing to maintain 60fps timing
    std::atomic<bool> stopCapture(false);
    std::atomic<int> currentFocusPos(0);
    std::mutex focusMutex;

    // Dedicated thread to get frames at maximum speed
    std::thread frameThread([&]() {
        unsigned char* threadBuffer = new unsigned char[img_size];
        
        while (!stopCapture.load()) {
            if (camera.getLatestFrame(threadBuffer, img_size)) {
                cv::Mat image(imgHeight, imgWidth, CV_8U, threadBuffer);
                cv::Mat resized;
                cv::resize(image, resized, cv::Size(), scale, scale);
                
                int focusPosition = focusController.computeBestFocus(resized, resized.rows, resized.cols);
                
                // Update the focus position that the main thread can read
                std::lock_guard<std::mutex> lock(focusMutex);
                currentFocusPos.store(focusPosition);
            }
            
            // Don't spin too fast - give a small yield
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        
        delete[] threadBuffer;
    });
    
    // For each Kp value to test
    for (double kp : kpValues) {
        std::cout << "\n===== TESTING Kp = " << kp << " =====" << std::endl;
        
        TestResult result;
        result.kp = kp;
        result.hasOvershoot = false;
        result.reachedTarget = false;
        result.settlingTime = 0;
        
        // Configure PID controller using the same parameters as autofocus.cpp
        // but with our test Kp value
        double dt = 1.0 / 60.0;  // time per frame (60Hz)
        double max = 3.0;        // maximum relative move (±3mm)
        double min = -3.0;
        double ki = 0.0;         // no integral gain
        double kd = 0.0;         // no derivative gain for cleaner test
        
        PID pid(dt, max, min, kp, kd, ki);
        
        // First ensure we're close to the starting focus
        std::cout << "Positioning to initial focus location..." << std::endl;
        int currentFocus = 0;
        int targetInitialFocus = startFocus;
        
        // Coarse positioning
        for (int attempts = 0; attempts < 15; attempts++) {
            // Get current focus position
            if (!camera.getLatestFrame(buffer, img_size)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
            
            cv::Mat image(imgHeight, imgWidth, CV_8U, buffer);
            cv::Mat resized;
            cv::resize(image, resized, cv::Size(), scale, scale);
            
            currentFocus = focusController.computeBestFocus(resized, resized.rows, resized.cols);
            std::cout << "Current focus: " << currentFocus << " pixels" << std::endl;
            
            if (std::abs(currentFocus - targetInitialFocus) < 20) {
                std::cout << "Close to initial position" << std::endl;
                break;
            }
            
            // Calculate movement using PID
            double moveAmount = pid.calculate(targetInitialFocus, currentFocus) * -1.0;
            
            // Use the global variables the way autofocus.cpp does
            mmToMove = moveAmount;
            bNewMoveRel = true;
            
            std::cout << "Moving lens by " << moveAmount << "mm" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        
        std::cout << "Ready to perform step response test!" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Set the initial focus position
        desiredLocBestFocus = startFocus;
        
        // Record start time
        auto startTime = std::chrono::steady_clock::now();
        
        // Variables to track step response
        bool prevCmdPositive = false;
        int withinTolCount = 0;
        bool settled = false;
        
        // Initial data point
        result.focusLocations.push_back(currentFocus);
        result.moveCommands.push_back(0.0);
        result.timestamps.push_back(0.0);
        
        // Change target to generate step input
        desiredLocBestFocus = targetFocus;
        std::cout << "Step input applied - new target: " << targetFocus << " pixels" << std::endl;
        
        // Track response for specified duration
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - startTime).count() < testTimeout) {
            
            // Get current focus position from the atomic variable
            int currentFocus = currentFocusPos.load();
            
            // Record time
            auto currentTime = std::chrono::steady_clock::now();
            double elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                    currentTime - startTime).count();
            
            // Calculate movement using PID
            double moveAmount = pid.calculate(targetFocus, currentFocus) * -1.0;
            
            // Use the global variables the way autofocus.cpp does
            mmToMove = moveAmount;
            bNewMoveRel = true;
            
            // Only log every few frames to reduce overhead
            static int frameCounter = 0;
            if (frameCounter++ % 4 == 0) {
                std::cout << "Time: " << elapsedMs << "ms, Focus: " << currentFocus 
                          << ", Target: " << targetFocus << ", Move: " << moveAmount << std::endl;
            }
            
            // Record data
            result.focusLocations.push_back(currentFocus);
            result.moveCommands.push_back(moveAmount);
            result.timestamps.push_back(elapsedMs);
            
            // Check for sign change (indicates overshoot)
            bool currCmdPositive = moveAmount > 0;
            if (result.moveCommands.size() > 1 && 
                currCmdPositive != prevCmdPositive && 
                std::abs(moveAmount) > 0.001) {
                result.hasOvershoot = true;
                std::cout << "Sign change detected at " << elapsedMs << "ms!" << std::endl;
            }
            prevCmdPositive = currCmdPositive;
            
            // Check if within tolerance band
            if (std::abs(currentFocus - desiredLocBestFocus) <= tolerance) {
                withinTolCount++;
                if (withinTolCount >= 3 && !settled) { // 3 consecutive readings within tolerance
                    settled = true;
                    result.settlingTime = static_cast<int>(elapsedMs);
                    result.reachedTarget = true;
                    std::cout << "Settled within tolerance at " << result.settlingTime << "ms" << std::endl;
                }
            } else {
                withinTolCount = 0;
            }
            
            // Use a consistent timing interval to match the 60Hz rate
            //std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60Hz
        }
        
        // Store result
        allResults.push_back(result);
        
        // Output results
        std::cout << "\nResults for Kp = " << kp << ":" << std::endl;
        std::cout << "  Reached target: " << (result.reachedTarget ? "Yes" : "No") << std::endl;
        std::cout << "  Settling time: " << result.settlingTime << " ms" << std::endl;
        std::cout << "  Overshoot detected: " << (result.hasOvershoot ? "Yes" : "No") << std::endl;
        
        // Reset lens position and wait between tests
        lensController.setDesiredLensPosition(-9.1);
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    
    // Clean up
    delete[] buffer;
    stopCapture.store(true);
    if (frameThread.joinable()) {
        frameThread.join();
    }
    
    // Find the best Kp value (highest without overshoot)
    double bestKp = 0.0;
    int bestSettlingTime = std::numeric_limits<int>::max();
    
    for (const auto& result : allResults) {
        if (!result.hasOvershoot && result.reachedTarget) {
            if (result.kp > bestKp || 
                (result.kp == bestKp && result.settlingTime < bestSettlingTime)) {
                bestKp = result.kp;
                bestSettlingTime = result.settlingTime;
            }
        }
    }
    
    // Generate plot
    std::cout << "\nGenerating plot data..." << std::endl;
    
    // Create data files for each result
    std::vector<std::string> plotData;
    
    // Create a data file for the step input
    std::ofstream stepInput("../output/step_input.csv");
    stepInput << "Time,DesiredFocus\n";
    
    if (!allResults.empty()) {
        for (size_t i = 0; i < allResults[0].timestamps.size(); i++) {
            double time = allResults[0].timestamps[i];
            int desiredFocus = (time > 0) ? targetFocus : startFocus; // Step input
            stepInput << time << "," << desiredFocus << "\n";
        }
    }
    stepInput.close();
    
    // Create data files for each result
    for (size_t i = 0; i < allResults.size(); i++) {
        std::string filename = "../output/kp_" + std::to_string(allResults[i].kp) + ".csv";
        std::ofstream dataFile(filename);
        dataFile << "Time,FocusLocation,MoveCommand\n";
        
        for (size_t j = 0; j < allResults[i].timestamps.size(); j++) {
            dataFile << allResults[i].timestamps[j] << ","
                     << allResults[i].focusLocations[j] << ","
                     << allResults[i].moveCommands[j] << "\n";
        }
        dataFile.close();
        plotData.push_back(filename);
    }
    
    // Generate Python plotting script
    std::ofstream plotScript("../output/generate_plot.py");
    plotScript << "import matplotlib.pyplot as plt\n";
    plotScript << "import pandas as pd\n";
    plotScript << "import numpy as np\n\n";
    
    // Load data
    plotScript << "# Load step input\n";
    plotScript << "step_data = pd.read_csv('step_input.csv')\n\n";
    
    // Load response data
    plotScript << "# Load response data\n";
    for (size_t i = 0; i < allResults.size(); i++) {
        plotScript << "data_" << i << " = pd.read_csv('" 
                   << plotData[i].substr(plotData[i].find_last_of('/') + 1) << "')\n";
    }
    
    // Create plot
    plotScript << "\n# Create plot\n";
    plotScript << "fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)\n";
    plotScript << "fig.suptitle('PID Controller Step Response with Different Kp Values', fontsize=16)\n\n";
    
    // Plot step input
    plotScript << "# Plot step input\n";
    plotScript << "ax1.plot(step_data['Time'], step_data['DesiredFocus'], 'k--', linewidth=2, label='Desired Focus')\n\n";
    
    // Find index of best result
    plotScript << "# Find best result\n";
    plotScript << "best_kp = " << bestKp << "\n";
    plotScript << "best_index = -1\n";
    plotScript << "for i, kp in enumerate([";
    for (size_t i = 0; i < allResults.size(); i++) {
        plotScript << allResults[i].kp;
        if (i < allResults.size() - 1) plotScript << ", ";
    }
    plotScript << "]):\n";
    plotScript << "    if kp == best_kp:\n";
    plotScript << "        best_index = i\n\n";
    
    // Plot each response
    plotScript << "# Plot focus location responses\n";
    plotScript << "for i in range(" << allResults.size() << "):\n";
    plotScript << "    data = locals()[f'data_{i}']\n";
    plotScript << "    kp = [" << allResults[0].kp;
    for (size_t i = 1; i < allResults.size(); i++) {
        plotScript << ", " << allResults[i].kp;
    }
    plotScript << "][i]\n";
    plotScript << "    has_overshoot = [" << (allResults[0].hasOvershoot ? "True" : "False");
    for (size_t i = 1; i < allResults.size(); i++) {
        plotScript << ", " << (allResults[i].hasOvershoot ? "True" : "False");
    }
    plotScript << "][i]\n";
    
    plotScript << "    linewidth = 3 if i == best_index else 1.5\n";
    plotScript << "    alpha = 1.0 if i == best_index else 0.7\n";
    plotScript << "    label = f'Kp={kp}'\n";
    plotScript << "    if i == best_index:\n";
    plotScript << "        label += ' (Best)'\n";
    plotScript << "    if has_overshoot:\n";
    plotScript << "        label += ' (Overshoot)'\n";
    plotScript << "    ax1.plot(data['Time'], data['FocusLocation'], linewidth=linewidth, alpha=alpha, label=label)\n";
    
    // Plot tolerance bands
    plotScript << "\n# Plot tolerance bands\n";
    plotScript << "ax1.axhspan(" << targetFocus << "-" << tolerance << ", " 
               << targetFocus << "+" << tolerance
               << ", color='green', alpha=0.2, label='Tolerance Band')\n";
    
    // Plot move commands
    plotScript << "\n# Plot move commands\n";
    plotScript << "for i in range(" << allResults.size() << "):\n";
    plotScript << "    data = locals()[f'data_{i}']\n";
    plotScript << "    kp = [" << allResults[0].kp;
    for (size_t i = 1; i < allResults.size(); i++) {
        plotScript << ", " << allResults[i].kp;
    }
    plotScript << "][i]\n";
    plotScript << "    linewidth = 3 if i == best_index else 1.5\n";
    plotScript << "    alpha = 1.0 if i == best_index else 0.7\n";
    plotScript << "    ax2.plot(data['Time'], data['MoveCommand'], linewidth=linewidth, alpha=alpha, label=f'Kp={kp}')\n";
    
    // Add zero line to move commands
    plotScript << "ax2.axhline(y=0, color='r', linestyle='-', alpha=0.3)\n";
    
    // Set labels and legends
    plotScript << "\n# Set labels and legends\n";
    plotScript << "ax1.set_ylabel('Focus Location (pixels)')\n";
    plotScript << "ax1.set_title('Focus Location Response')\n";
    plotScript << "ax1.legend(loc='best')\n";
    plotScript << "ax1.grid(True)\n\n";
    
    plotScript << "ax2.set_xlabel('Time (ms)')\n";
    plotScript << "ax2.set_ylabel('Move Command (mm)')\n";
    plotScript << "ax2.set_title('PID Controller Output')\n";
    plotScript << "ax2.grid(True)\n\n";
    
    // Annotations for best result
    if (bestKp > 0.0) {
        plotScript << "# Add annotation for best result\n";
        plotScript << "ax1.annotate(f'Best Kp: " << bestKp 
                   << " (Settling time: " << bestSettlingTime 
                   << " ms)', xy=(0.5, 0.02), xycoords='axes fraction', "
                   << "bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.5), "
                   << "ha='center', fontsize=12)\n\n";
    }
    
    // Save figure
    plotScript << "plt.tight_layout()\n";
    plotScript << "plt.savefig('pid_step_response.png', dpi=300)\n";
    plotScript << "plt.show()\n";
    
    plotScript.close();
    
    // Output recommendation
    std::cout << "\n===== PID TUNING RESULTS =====" << std::endl;
    if (bestKp > 0.0) {
        std::cout << "Recommended Kp value: " << bestKp << std::endl;
        std::cout << "Settling time: " << bestSettlingTime << " ms" << std::endl;
        std::cout << "Update the Kp value in autofocus.cpp to " << bestKp << std::endl;
        
        // Log the result to a file for reference
        std::ofstream resultFile("../output/pid_tuning_results.txt");
        resultFile << "Recommended PID Parameters for Autofocus Controller\n";
        resultFile << "===================================================\n";
        resultFile << "Proportional gain (Kp): " << bestKp << "\n";
        resultFile << "Integral gain (Ki): 0.0\n";
        resultFile << "Derivative gain (Kd): 0.0\n";
        resultFile << "Settling time: " << bestSettlingTime << " ms\n";
        resultFile << "Tested on step change from " << startFocus << " to " << targetFocus << " pixels\n";
        resultFile.close();
    } else {
        std::cout << "No suitable Kp value found that meets all criteria." << std::endl;
        std::cout << "Try testing with different Kp values." << std::endl;
    }
    
    std::cout << "\nRun the following to generate the plot:" << std::endl;
    std::cout << "cd ../output && python generate_plot.py" << std::endl;
    
    return 0;
} 