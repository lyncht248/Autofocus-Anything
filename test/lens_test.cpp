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

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 