#include "logfile.hpp"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/null_sink.h>

// // Prints logs from tiltedcam_test.cpp to the console
// std::shared_ptr<spdlog::logger> logger = []() {
//     auto console_logger = spdlog::stdout_color_mt("test_logger");
//     console_logger->set_pattern("[%H:%M:%S] [thread %t] %v");
//     return console_logger;
// }();

// Create a silent logger that discards all messages
std::shared_ptr<spdlog::logger> logger = []() {
    auto null_sink = std::make_shared<spdlog::sinks::null_sink_mt>();
    auto null_logger = std::make_shared<spdlog::logger>("null_logger", null_sink);
    return null_logger;
}();