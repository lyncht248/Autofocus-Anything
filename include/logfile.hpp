/**
 * @file logfile.hpp
 * @brief Declares logging utilities for the application.
 * 
 * This file provides the declaration of a global `spdlog` logger for logging application events.
 * It includes necessary headers for file-based logging and integrates with the `spdlog` library.
 */

#ifndef HVIGTK_LOGFILE_H
#define HVIGTK_LOGFILE_H

#include <fstream>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

extern std::shared_ptr<spdlog::logger> logger;

#endif
