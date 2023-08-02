#ifndef HVIGTK_LOGFILE_H
#define HVIGTK_LOGFILE_H

#include <fstream>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

extern std::shared_ptr<spdlog::logger> logger;

#endif
