#include "mock_notification.hpp"
#include "tiltedcam.hpp"
#include "gtest/gtest.h"
#include <chrono>
#include <thread>
#include <vector>
#include <numeric>
#include <cstring>

/**
 * @class TiltedCamTest
 * @brief Test fixture for tiltedcam tests
 */
class TiltedCamTest : public ::testing::Test {
protected:
    tiltedcam* camera;
    
    void SetUp() override {
        camera = new tiltedcam();
    }
    
    void TearDown() override {
        delete camera;
    }
};

/**
 * @brief Test camera initialization
 * 
 * Verifies that the camera can be initialized successfully.
 */
TEST_F(TiltedCamTest, Initialization) {
    EXPECT_TRUE(camera->initialize()) << "Camera initialization failed";
}

/**
 * @brief Test camera settings
 * 
 * Verifies that camera settings can be set and retrieved correctly.
 */
TEST_F(TiltedCamTest, CameraSettings) {
    ASSERT_TRUE(camera->initialize()) << "Camera initialization failed";
    
    // Test gain
    long initialGain = camera->getGain();
    long newGain = initialGain + 10;
    camera->setGain(newGain);
    EXPECT_EQ(camera->getGain(), newGain) << "Failed to set gain";
    
    // Test exposure
    long initialExposure = camera->getExposure();
    long newExposure = 20000; // 20ms
    camera->setExposure(newExposure);
    EXPECT_EQ(camera->getExposure(), newExposure) << "Failed to set exposure";
    
    // Test gamma
    long initialGamma = camera->getGamma();
    long newGamma = initialGamma + 10;
    camera->setGamma(newGamma);
    EXPECT_EQ(camera->getGamma(), newGamma) << "Failed to set gamma";
    
    // Test high speed mode
    long initialHSM = camera->getHighSpeedMode();
    long newHSM = initialHSM == 0 ? 1 : 0;
    camera->setHighSpeedMode(newHSM);
    EXPECT_EQ(camera->getHighSpeedMode(), newHSM) << "Failed to set high speed mode";
    
    // Reset to initial values
    camera->setGain(initialGain);
    camera->setExposure(initialExposure);
    camera->setGamma(initialGamma);
    camera->setHighSpeedMode(initialHSM);
}

/**
 * @brief Test image dimensions
 * 
 * Verifies that the camera reports the expected image dimensions.
 */
TEST_F(TiltedCamTest, ImageDimensions) {
    ASSERT_TRUE(camera->initialize()) << "Camera initialization failed";
    
    // Expected dimensions based on initialization in tiltedcam.cpp
    EXPECT_EQ(camera->getImageWidth(), 1280) << "Unexpected image width";
    EXPECT_EQ(camera->getImageHeight(), 960) << "Unexpected image height";
    EXPECT_EQ(camera->getBins(), 1) << "Unexpected binning value";
    EXPECT_EQ(camera->getImageType(), ASI_IMG_RAW8) << "Unexpected image type";
}

/**
 * @brief Test frame capture
 * 
 * Verifies that the camera can capture frames.
 */
TEST_F(TiltedCamTest, FrameCapture) {
    ASSERT_TRUE(camera->initialize()) << "Camera initialization failed";
    
    // Start capture thread
    camera->startCaptureThread();
    
    // Allow time for the camera to warm up (5 seconds)
    std::cout << "Waiting for camera warm-up period (2 seconds)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Allocate buffer for frame
    int width = camera->getImageWidth();
    int height = camera->getImageHeight();
    long bufferSize = width * height; // Assuming 8-bit images
    unsigned char* buffer = new unsigned char[bufferSize];
    
    // Try to get a frame
    bool frameReceived = camera->getLatestFrame(buffer, bufferSize);
    EXPECT_TRUE(frameReceived) << "Failed to receive a frame";
    
    // Check that the buffer contains valid image data (not all zeros or all the same value)
    if (frameReceived) {
        bool allSame = true;
        unsigned char firstPixel = buffer[0];
        
        // Check first 1000 pixels
        for (int i = 1; i < 1000 && i < bufferSize; i++) {
            if (buffer[i] != firstPixel) {
                allSame = false;
                break;
            }
        }
        
        EXPECT_FALSE(allSame) << "Image data appears to be invalid (all pixels have the same value)";
    }
    
    // Clean up
    delete[] buffer;
    camera->stopCaptureThread();
}

/**
 * @brief Test frame rate
 * 
 * Verifies that the camera can capture frames at or near 60 fps.
 */
TEST_F(TiltedCamTest, FrameRate) {
    ASSERT_TRUE(camera->initialize()) << "Camera initialization failed";
    
    // Set exposure for 60 fps (16.67ms max)
    camera->setExposure(16000); // 16ms
    camera->setHighSpeedMode(1); // Enable high speed mode
    
    // Start capture thread
    camera->startCaptureThread();
    
    // Allow time for the thread to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Allocate buffer for frame
    int width = camera->getImageWidth();
    int height = camera->getImageHeight();
    long bufferSize = width * height; // Assuming 8-bit images
    unsigned char* buffer = new unsigned char[bufferSize];
    
    // Measure frame rate over 3 seconds
    const int testDurationMs = 3000;
    const int sampleIntervalMs = 5; // Check for new frames every 5ms
    int frameCount = 0;
    std::vector<double> frameTimes;
    
    auto startTime = std::chrono::high_resolution_clock::now();
    auto lastFrameTime = startTime;
    auto currentTime = startTime;
    
    while (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count() < testDurationMs) {
        if (camera->getLatestFrame(buffer, bufferSize)) {
            frameCount++;
            
            // Calculate time since last frame
            auto now = std::chrono::high_resolution_clock::now();
            double frameTimeMs = std::chrono::duration_cast<std::chrono::microseconds>(now - lastFrameTime).count() / 1000.0;
            frameTimes.push_back(frameTimeMs);
            lastFrameTime = now;
        }
        
        // Sleep for a short time to avoid hammering the CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(sampleIntervalMs));
        currentTime = std::chrono::high_resolution_clock::now();
    }
    
    // Calculate actual test duration (in case of slight timing variations)
    double actualDurationSec = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count() / 1000.0;
    
    // Calculate frame rate
    double frameRate = frameCount / actualDurationSec;
    
    // Calculate frame time statistics
    double avgFrameTimeMs = 0;
    double minFrameTimeMs = 1000000;
    double maxFrameTimeMs = 0;
    
    if (!frameTimes.empty()) {
        avgFrameTimeMs = std::accumulate(frameTimes.begin(), frameTimes.end(), 0.0) / frameTimes.size();
        minFrameTimeMs = *std::min_element(frameTimes.begin(), frameTimes.end());
        maxFrameTimeMs = *std::max_element(frameTimes.begin(), frameTimes.end());
    }
    
    // Clean up
    delete[] buffer;
    camera->stopCaptureThread();
    
    // Output detailed results
    std::cout << "Frame rate test results:" << std::endl;
    std::cout << "  Frames captured: " << frameCount << std::endl;
    std::cout << "  Test duration: " << actualDurationSec << " seconds" << std::endl;
    std::cout << "  Frame rate: " << frameRate << " fps" << std::endl;
    std::cout << "  Average frame time: " << avgFrameTimeMs << " ms" << std::endl;
    std::cout << "  Min frame time: " << minFrameTimeMs << " ms" << std::endl;
    std::cout << "  Max frame time: " << maxFrameTimeMs << " ms" << std::endl;
    
    // Verify frame rate is at least 55 fps (allowing for some overhead)
    EXPECT_GE(frameRate, 55.0) << "Frame rate is below the target of 60 fps";
    
    // Verify max frame time is consistent with 60 fps
    EXPECT_LE(maxFrameTimeMs, 22.0) << "Maximum frame time exceeds 22ms, inconsistent with 60 fps";
}

/**
 * @brief Test error handling
 * 
 * Verifies that the camera handles error conditions gracefully.
 */
TEST_F(TiltedCamTest, ErrorHandling) {
    ASSERT_TRUE(camera->initialize()) << "Camera initialization failed";
    
    // Test with null buffer
    EXPECT_FALSE(camera->getLatestFrame(nullptr, 1000)) << "getLatestFrame should fail with null buffer";
    
    // Test with incorrect buffer size
    unsigned char buffer[10];
    EXPECT_FALSE(camera->getLatestFrame(buffer, 10)) << "getLatestFrame should fail with incorrect buffer size";
    
    // Test getting frame before starting capture thread
    int width = camera->getImageWidth();
    int height = camera->getImageHeight();
    long bufferSize = width * height;
    unsigned char* properBuffer = new unsigned char[bufferSize];
    
    EXPECT_FALSE(camera->getLatestFrame(properBuffer, bufferSize)) 
        << "getLatestFrame should fail before starting capture thread";
    
    // Clean up
    delete[] properBuffer;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 