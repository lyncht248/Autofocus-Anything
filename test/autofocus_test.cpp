#include "autofocus.hpp"
#include "gtest/gtest.h"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <sys/select.h>
#include <thread>
#include <vector>

/**
 * @class AutofocusTest
 * @brief Test fixture for autofocus tests
 */
class AutofocusTest : public ::testing::Test {
protected:
  autofocus *focusController;

  void SetUp() override { focusController = new autofocus(); }

  void TearDown() override { delete focusController; }
};

/**
 * @brief Generate a lookup table for mm/pixel ratios at different lens
 * positions and focus locations
 *
 * This test creates a comprehensive lookup table for the feedforward controller
 * by:
 * 1. Starting at different lens positions (-12mm, -10.74mm, -8mm, -6mm, -4mm,
 * -2mm)
 * 2. At each position, scanning ±2.5mm around the starting position with 0.1mm
 * increments
 * 3. Taking multiple measurements at each point to ensure accuracy
 * 4. Calculating mm/pixel ratios for each lens position and focus location
 * combination
 */
TEST_F(AutofocusTest, GenerateMmPixelLookupTable) {
  // Skip if initialization fails
  if (!focusController->initialize()) {
    GTEST_SKIP() << "Skipping test because autofocus initialization failed";
  }

  // Allow system to stabilize
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Get reference to camera and lens
  auto &tiltedcam = focusController->getTiltedCam();
  auto &lens = focusController->getLens();

  // Start the camera capture thread
  tiltedcam.startCaptureThread();

  // Image buffer setup
  const int imgWidth = tiltedcam.getImageWidth();
  const int imgHeight = tiltedcam.getImageHeight();
  const long img_size = imgWidth * imgHeight;
  unsigned char *buffer = new unsigned char[img_size];

  double scale = 0.5; // Same scale used in autofocus::run

  // Starting lens positions to test (updated for current bounds: MIN=-15.0,
  // MAX=0.0) Using more conservative positions to avoid boundary issues
  // TEMPORARILY TESTING ONLY -10.74mm POSITION
  std::vector<double> startingPositions = {-10.74};

  // Structure to store results
  struct MeasurementPoint {
    double lensPosition;
    int focusLocation;
    double mmPerPixel;
    int measurementCount;
  };

  std::vector<MeasurementPoint> lookupTable;

  // Create raw measurements CSV file to save all lens position and focus
  // location pairs
  std::ofstream rawDataFile("raw_measurements.csv");
  if (!rawDataFile.is_open()) {
    std::cerr << "Error: Could not create raw_measurements.csv" << std::endl;
    delete[] buffer;
    FAIL() << "Failed to create raw measurements CSV file";
    return;
  }
  rawDataFile << "LensPosition,FocusLocation" << std::endl;

  // For each starting position
  for (double startPos : startingPositions) {
    std::cout << "\n\n===== STARTING POSITION: " << startPos
              << "mm =====" << std::endl;

    // Use 0.1mm steps for all positions, scanning ±2.5mm around each starting
    // position
    double stepSize = 0.1;
    double scanRange = 2.5; // ±2.5mm around starting position

    std::cout << "Using 0.1mm steps, scanning ±" << scanRange
              << "mm around starting position" << std::endl;

    // Move lens to starting position
    lens.setDesiredLensPosition(startPos);
    std::this_thread::sleep_for(
        std::chrono::seconds(3)); // Allow extra time for lens to settle

    // Verify the lens actually moved to the starting position
    double actualStartPos = lens.getLensPosition();
    std::cout << "Moved to starting position - Expected: " << startPos
              << "mm, Actual: " << actualStartPos << "mm" << std::endl;

    // Display current best focus location and wait for user confirmation
    int currentFocus = -1;
    bool isReady = false;

    std::cout << "\nAdjust manually until focus is satisfactory. Displaying "
                 "current focus:"
              << std::endl;
    std::cout << "Press ENTER when ready to proceed: " << std::endl;

    // Display focus location every second until user confirms
    while (!isReady) {
      // Capture frame with retry logic
      bool frameObtained = false;
      for (int attempt = 0; attempt < 3 && !frameObtained; attempt++) {
        frameObtained = tiltedcam.getLatestFrame(buffer, img_size);
        if (!frameObtained) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }

      if (!frameObtained) {
        std::cout << "Failed to capture frame after 3 attempts" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }

      // Process image to find best focus
      cv::Mat image = cv::Mat(imgHeight, imgWidth, CV_8U, buffer);
      cv::Mat resized;
      cv::resize(image, resized, cv::Size(), scale, scale);

      currentFocus = focusController->computeBestFocusEigenLM(resized, resized.rows,
                                                       resized.cols);

      std::cout << "Current best focus location: " << currentFocus << " pixels";
      if (currentFocus >= 300 && currentFocus <= 400) {
        std::cout << " (GOOD - in ideal range)";
      } else {
        std::cout << " (ideal is 300-400)";
      }
      std::cout << std::endl;

      // Check if user input is available without blocking
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(STDIN_FILENO, &readfds);

      struct timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000; // 100ms timeout

      int ready = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);

      if (ready > 0) {
        // Input is available, read a single character without waiting for Enter
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
          // Accept Enter (newline) as confirmation
          if (c == '\n') {
            isReady = true;
            if (currentFocus >= 300 && currentFocus <= 400) {
              std::cout << "Focus is in the ideal range (300-400)."
                        << std::endl;
            } else {
              std::cout
                  << "Focus is outside ideal range, but proceeding anyway."
                  << std::endl;
            }
          }
        }
      } else {
        // No input available, continue displaying focus
        std::this_thread::sleep_for(std::chrono::milliseconds(900));
      }
    }

    // Scan from starting position toward more negative positions (startPos -
    // scanRange)
    double minScanPos = startPos - scanRange;
    double maxScanPos = startPos + scanRange;

    // Ensure we don't exceed lens limits
    minScanPos = std::max(minScanPos, -14.5); // Stay within MIN_POSITION limit
    maxScanPos = std::min(maxScanPos, -0.5);  // Stay within MAX_POSITION limit

    std::cout << "\nScanning from " << startPos << "mm to " << minScanPos
              << "mm with " << stepSize << "mm steps..." << std::endl;

    // Start from the actual starting position, not the reported lens position
    double lensPos = startPos;

    // Verify we're actually at the starting position
    double actualPos = lens.getLensPosition();
    std::cout << "Expected position: " << startPos
              << "mm, Actual position: " << actualPos << "mm" << std::endl;

    while (lensPos > minScanPos) {
      // Store the current position in our lookup table (average of 3
      // measurements)
      MeasurementPoint point = {lensPos, 0, 0.0, 0};

      // Take 3 measurements at this position
      for (int i = 0; i < 3; i++) {
        // Retry frame capture
        bool frameObtained = false;
        for (int attempt = 0; attempt < 5 && !frameObtained; attempt++) {
          frameObtained = tiltedcam.getLatestFrame(buffer, img_size);
          if (!frameObtained) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
          }
        }

        if (!frameObtained) {
          std::cout << "Failed to capture frame after 5 attempts" << std::endl;
          continue;
        }

        cv::Mat image = cv::Mat(imgHeight, imgWidth, CV_8U, buffer);
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(), scale, scale);

        int focus = focusController->computeBestFocusEigenLM(resized, resized.rows,
                                                      resized.cols);

        // Add this measurement to our average
        point.focusLocation += focus;
        point.measurementCount++;

        // Wait briefly between measurements
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }

      // Calculate average focus location
      if (point.measurementCount > 0) {
        point.focusLocation /= point.measurementCount;

        // Store this point
        lookupTable.push_back(point);

        // Write raw measurement to CSV file
        rawDataFile << lensPos << "," << point.focusLocation << std::endl;

        std::cout << "Lens position: " << lensPos
                  << "mm, Focus location: " << point.focusLocation << " pixels"
                  << std::endl;

        // Update our tracking variable
        currentFocus = point.focusLocation;
      }

      // Move lens by -stepSize (toward more negative positions)
      lensPos -= stepSize;
      lens.setDesiredLensPosition(lensPos);

      // Wait for lens to settle
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Reset lens to starting position for second scan
    lens.setDesiredLensPosition(startPos);
    std::this_thread::sleep_for(
        std::chrono::seconds(3)); // Allow extra time for lens to settle

    // Second scan: from starting position toward more positive positions
    // (startPos + scanRange)
    std::cout << "\nScanning from " << startPos << "mm to " << maxScanPos
              << "mm with " << stepSize << "mm steps..." << std::endl;

    // Start from the actual starting position again
    lensPos = startPos;

    // Verify we're back at the starting position
    actualPos = lens.getLensPosition();
    std::cout << "Expected position: " << startPos
              << "mm, Actual position: " << actualPos << "mm" << std::endl;

    while (lensPos < maxScanPos) {
      // Store the current position in our lookup table (average of 3
      // measurements)
      MeasurementPoint point = {lensPos, 0, 0.0, 0};

      // Take 3 measurements at this position
      for (int i = 0; i < 3; i++) {
        // Retry frame capture
        bool frameObtained = false;
        for (int attempt = 0; attempt < 5 && !frameObtained; attempt++) {
          frameObtained = tiltedcam.getLatestFrame(buffer, img_size);
          if (!frameObtained) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
          }
        }

        if (!frameObtained) {
          std::cout << "Failed to capture frame after 5 attempts" << std::endl;
          continue;
        }

        cv::Mat image = cv::Mat(imgHeight, imgWidth, CV_8U, buffer);
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(), scale, scale);

        int focus = focusController->computeBestFocusEigenLM(resized, resized.rows,
                                                      resized.cols);

        // Add this measurement to our average
        point.focusLocation += focus;
        point.measurementCount++;

        // Wait briefly between measurements
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }

      // Calculate average focus location
      if (point.measurementCount > 0) {
        point.focusLocation /= point.measurementCount;

        // Store this point
        lookupTable.push_back(point);

        // Write raw measurement to CSV file
        rawDataFile << lensPos << "," << point.focusLocation << std::endl;

        std::cout << "Lens position: " << lensPos
                  << "mm, Focus location: " << point.focusLocation << " pixels"
                  << std::endl;

        // Update our tracking variable
        currentFocus = point.focusLocation;
      }

      // Move lens by +stepSize (toward less negative positions, closer to 0)
      lensPos += stepSize;
      lens.setDesiredLensPosition(lensPos);

      // Wait for lens to settle
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  // Calculate mm/pixel ratios for each focus location and lens position
  std::cout << "\n===== CALCULATING MM/PIXEL RATIOS =====" << std::endl;

  // Group measurements by lens position range (approximately)
  std::map<int, std::vector<MeasurementPoint>> lensPositionGroups;

  for (const auto &point : lookupTable) {
    // Group by rounded lens position (to nearest 0.5mm)
    int key = static_cast<int>(std::round(point.lensPosition * 2));
    lensPositionGroups[key].push_back(point);
  }

  // For each lens position group, calculate the mm/pixel relationship
  for (auto &[key, points] : lensPositionGroups) {
    // Sort points by focus location
    std::sort(points.begin(), points.end(),
              [](const MeasurementPoint &a, const MeasurementPoint &b) {
                return a.focusLocation < b.focusLocation;
              });

    // Calculate ratios between adjacent points
    for (size_t i = 0; i < points.size() - 1; i++) {
      int pixelDiff = points[i + 1].focusLocation - points[i].focusLocation;
      double mmDiff =
          std::abs(points[i + 1].lensPosition - points[i].lensPosition);

      if (pixelDiff != 0) {
        double ratio = mmDiff / std::abs(pixelDiff);
        points[i].mmPerPixel = ratio;

        // Also assign to the original point in the lookupTable
        for (auto &tablePoint : lookupTable) {
          if (std::abs(tablePoint.lensPosition - points[i].lensPosition) <
                  0.001 &&
              tablePoint.focusLocation == points[i].focusLocation) {
            tablePoint.mmPerPixel = ratio;
            break;
          }
        }
      }
    }
  }

  // Output the lookup table in a more useful format
  std::cout << "\n===== MM/PIXEL LOOKUP TABLE =====" << std::endl;
  std::cout << "Lens Position (mm)\tFocus Location (px)\tMM/Pixel Ratio"
            << std::endl;

  // Close the raw measurements file
  rawDataFile.close();

  // Save lookup table to file in a better format for code implementation
  std::ofstream outFile("mm_pixel_lookup_table.csv");
  if (!outFile.is_open()) {
    std::cerr << "Error: Could not create mm_pixel_lookup_table.csv"
              << std::endl;
    delete[] buffer;
    FAIL() << "Failed to create output CSV file";
    return;
  }
  outFile << "LensPosition,FocusLocation,MmPerPixel" << std::endl;

  // Also create a 2D lookup table file that's easier to use in code
  std::ofstream lookupTableFile("feedforward_lookup_table.h");
  if (!lookupTableFile.is_open()) {
    std::cerr << "Error: Could not create feedforward_lookup_table.h"
              << std::endl;
    outFile.close();
    delete[] buffer;
    FAIL() << "Failed to create output header file";
    return;
  }
  lookupTableFile
      << "// Autogenerated lookup table for feedforward controller\n\n";
  lookupTableFile << "#ifndef FEEDFORWARD_LOOKUP_TABLE_H\n";
  lookupTableFile << "#define FEEDFORWARD_LOOKUP_TABLE_H\n\n";
  lookupTableFile << "#include <map>\n";
  lookupTableFile << "#include <utility>\n\n";
  lookupTableFile << "// Map from (int focusLocation, double lensPosition) to "
                     "mm/pixel ratio\n";
  lookupTableFile
      << "const std::map<std::pair<int, int>, double> MM_PIXEL_RATIO = {\n";

  // Group data by focus location (rounded to nearest 10 pixels) and lens
  // position (rounded to nearest 0.5mm)
  std::map<std::pair<int, int>, std::vector<double>> groupedRatios;

  for (const auto &point : lookupTable) {
    if (point.mmPerPixel > 0) {
      // Output to original CSV and console
      outFile << point.lensPosition << "," << point.focusLocation << ","
              << point.mmPerPixel << std::endl;

      std::cout << point.lensPosition << "\t\t" << point.focusLocation << "\t\t"
                << point.mmPerPixel << std::endl;

      // Group data for the 2D lookup table
      int focusKey =
          static_cast<int>(std::round(point.focusLocation / 10.0) * 10);
      int lensKey = static_cast<int>(std::round(point.lensPosition * 2));
      groupedRatios[{focusKey, lensKey}].push_back(point.mmPerPixel);
    }
  }

  // Output the averaged ratios to the lookup table header
  for (const auto &[key, ratios] : groupedRatios) {
    // Calculate average ratio for this combination
    double avgRatio = 0.0;
    if (!ratios.empty()) {
      avgRatio =
          std::accumulate(ratios.begin(), ratios.end(), 0.0) / ratios.size();
    }

    lookupTableFile << "    {{" << key.first << ", " << key.second << "}, "
                    << avgRatio << "},\n";
  }

  lookupTableFile << "};\n\n";
  lookupTableFile << "#endif // FEEDFORWARD_LOOKUP_TABLE_H\n";

  // Close files and check for errors
  outFile.close();
  lookupTableFile.close();

  if (rawDataFile.fail()) {
    std::cerr << "Error: Failed to write to raw_measurements.csv" << std::endl;
  } else {
    std::cout << "\nSuccessfully created raw_measurements.csv" << std::endl;
  }

  if (outFile.fail()) {
    std::cerr << "Error: Failed to write to mm_pixel_lookup_table.csv"
              << std::endl;
  } else {
    std::cout << "Successfully created mm_pixel_lookup_table.csv" << std::endl;
  }

  if (lookupTableFile.fail()) {
    std::cerr << "Error: Failed to write to feedforward_lookup_table.h"
              << std::endl;
  } else {
    std::cout << "Successfully created feedforward_lookup_table.h" << std::endl;
  }

  // Clean up
  delete[] buffer;

  // Not making assertions as this is a measurement test
  SUCCEED();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}