#include "lens.hpp"
#include "gtest/gtest.h"
#include <chrono>
#include <thread>
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <cmath>

/**
 * @class LensTest
 * @brief Test fixture for lens tests
 */
class LensTest : public ::testing::Test {
protected:
    lens* lensController;
    
    void SetUp() override {
        lensController = new lens();
    }
    
    void TearDown() override {
        delete lensController;
    }
};

/**
 * @brief Test initialization of the lens controller
 * 
 * Verifies the lens controller initializes successfully and establishes
 * communication with the Xeryon hardware.
 */
TEST_F(LensTest, InitializationTest) {
    EXPECT_TRUE(lensController->initialize());
}

/**
 * @brief Test getting and setting lens position
 * 
 * Verifies that setting the lens position actually moves the lens,
 * and that getDesiredLensPosition and getLensPosition return expected values.
 */
TEST_F(LensTest, PositionGetAndSetTest) {
    // Initialize the lens controller
    ASSERT_TRUE(lensController->initialize());
    
    // Sleep to ensure initialization is complete
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Test position setting and getting
    double testPosition = -9.0; // Valid position within bounds
    
    // Set desired position
    lensController->setDesiredLensPosition(testPosition);
    
    // Allow time for the lens to move
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Get desired position - should match what we set
    double desiredPos = lensController->getDesiredLensPosition();
    EXPECT_NEAR(testPosition, desiredPos, 0.1);
    
    // Get actual position - should be close to desired position after movement
    double actualPos = lensController->getLensPosition();
    EXPECT_NEAR(testPosition, actualPos, 0.2);
}

/**
 * @brief Test movement within and outside bounds
 * 
 * Verifies that the lens can move within its defined bounds,
 * but will not move beyond MIN_POSITION and MAX_POSITION.
 */
TEST_F(LensTest, BoundaryMovementTest) {
    // Initialize the lens controller
    ASSERT_TRUE(lensController->initialize());
    
    // Sleep to ensure initialization is complete
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Test positions within bounds
    double validPosition1 = -10.0;
    double validPosition2 = -5.0;
    
    // Set to first valid position
    lensController->setDesiredLensPosition(validPosition1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    EXPECT_NEAR(validPosition1, lensController->getLensPosition(), 0.2);
    
    // Set to second valid position
    lensController->setDesiredLensPosition(validPosition2);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    EXPECT_NEAR(validPosition2, lensController->getLensPosition(), 0.2);
    
    // Attempt to move beyond MAX_POSITION (0.0)
    double invalidPositionMax = 5.0; // Beyond max bound
    
    lensController->setDesiredLensPosition(invalidPositionMax);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Position should remain at valid limit or not move significantly
    double newPos = lensController->getLensPosition();
    EXPECT_LE(newPos, 0.1); // Should not exceed MAX_POSITION + small tolerance
    
    // Return to valid position
    lensController->setDesiredLensPosition(validPosition2);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Attempt to move beyond MIN_POSITION (-14.9)
    double invalidPositionMin = -20.0; // Beyond min bound
    
    lensController->setDesiredLensPosition(invalidPositionMin);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Position should remain at valid limit or not move significantly
    newPos = lensController->getLensPosition();
    EXPECT_GE(newPos, -15.0); // Should not exceed MIN_POSITION - small tolerance
}

/**
 * @brief Test return to start functionality
 * 
 * Tests the ability to set a return position and move back to it.
 */
TEST_F(LensTest, ReturnToStartTest) {
    // Initialize the lens controller
    ASSERT_TRUE(lensController->initialize());
    
    // Sleep to ensure initialization is complete
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Set a custom return position
    double customReturnPosition = -8.5;
    lensController->setReturnPosition(customReturnPosition);
    
    // Move to some other position first
    lensController->setDesiredLensPosition(-5.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Return to start position
    lensController->returnToStart();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Verify we are at the return position
    double actualPos = lensController->getLensPosition();
    EXPECT_NEAR(customReturnPosition, actualPos, 0.2);
}

/**
 * @brief Test DPOS streaming
 * 
 * Tests that the lens position getters return expected values.
 * The getDesiredLensPosition() method now handles INFO mode changes internally.
 */
TEST_F(LensTest, DPOSStreamingTest) {
    // Initialize the lens controller
    ASSERT_TRUE(lensController->initialize());
    
    // Sleep to ensure initialization is complete
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Set a test position
    double testPosition = -9.5;
    lensController->setDesiredLensPosition(testPosition);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Verify actual position
    double actualPos = lensController->getLensPosition();
    EXPECT_NEAR(testPosition, actualPos, 0.2);
    
    // Get desired position - now should work due to internal INFO mode changes
    double desiredPos = lensController->getDesiredLensPosition();
    EXPECT_NEAR(testPosition, desiredPos, 0.1);
    
    // Set a different position
    double newPosition = -7.5;
    lensController->setDesiredLensPosition(newPosition);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Check both actual and desired positions
    actualPos = lensController->getLensPosition();
    EXPECT_NEAR(newPosition, actualPos, 0.2);
    
    desiredPos = lensController->getDesiredLensPosition();
    EXPECT_NEAR(newPosition, desiredPos, 0.1);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 