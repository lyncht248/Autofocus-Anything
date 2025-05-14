#include "main.hpp"
#include <fstream>
#include <iostream>
#include <gtkmm.h>
#include <unistd.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include "glibmm/main.h"
#include "sdlwindow.hpp"
#include "version.hpp"
#include "system.hpp"
#include "autofocus.hpp"

#include <opencv2/core/ocl.hpp>
#include <thread>

#include <memory>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

std::shared_ptr<spdlog::logger> logger;

SDLWindow::SDLWin *childwin = nullptr;
std::atomic<bool> bAutofocusing = false; // Flag that controls the autofocusing while() loop

int gtkAppLocationX = 42;
int gtkAppLocationY = 10;

int main(int argc, char **argv) 
{
    // // Creates the logger file output and sets the pattern
    // auto now = std::chrono::system_clock::now();
    // auto in_time_t = std::chrono::system_clock::to_time_t(now);
    // std::stringstream ss;
    // std::tm buf;
    // std::tm* tm = localtime_r(&in_time_t, &buf);
    // ss << std::put_time(tm, "%Y%m%d_%H:%M:%S") << "_hvikgtk.log";
    // std::string filename = "/home/hvi/Desktop/HVI-log-report/" + ss.str();
    // logger = spdlog::basic_logger_mt("basic_logger", filename);
    logger = spdlog::stdout_color_mt("console");
    logger->set_pattern("[%H:%M:%S] [thread %t] %v");    

    // Spawns the SDL window, which is done in a separate process
    childwin = SDLWindow::sdlwin_open();
    if (!childwin) {
        return 1;
    }

    // // Checks if OpenCL is available on the system, and if so, prints out the device information
    // if (cv::ocl::haveOpenCL()) {
    //     cv::ocl::setUseOpenCL(true);
    //     cv::ocl::Device dev = cv::ocl::Device::getDefault();
    //     if (dev.available()) {
    //         logger->info("[Main] Device Name: {}", dev.name());
    //         logger->info("[Main] Device Type: {}", dev.type());
    //         logger->info("[Main] Device Vendor: {}", dev.vendorName());
    //         logger->info("[Main] Device Version: {}", dev.version());
    //         logger->info("[Main] Device OpenCL Version: {}", dev.OpenCLVersion());
    //         logger->info("[Main] Device Driver Version: {}", dev.driverVersion());
    //     } else {
    //         logger->info("[Main] No OpenCL device available.");
    //     }
    // } else {
    //     logger->info("[Main] OpenCL is not available on this system.");
    // }

    // Creates a GTK application object
    auto app = Gtk::Application::create(argc, argv, HVIGTK_APPID); 
    logger->info("[Main] GTK application object created");

    // Sets to dark theme, which is useful for operator to see contrast better
    Glib::RefPtr<Gtk::Settings> settings = Gtk::Settings::get_default();
    if(settings) {
        settings->property_gtk_application_prefer_dark_theme() = true;
    }

    // Initializes the GTK thread system and sets the GTK thread to default (?)
    if(!Glib::thread_supported()) Glib::thread_init();
    Glib::MainContext::get_default()->push_thread_default();
    logger->info("[Main] GTK thread system initialized and GTK thread set to default");

    // Creates a System object to run the logic behind the app's main window
    System system(argc, argv);
    logger->info("[Main] System object finished creating");

    // Opens the GTK application GUI and stops main() execution. system.getWindow() returns a pointer to the MainWindow object created in system.cc
    int out = app->run(system.getWindow() );
    logger->info("[Main] app->run loop is finished executing");

    // Closes the SDL window
    SDLWindow::sdlwin_close(childwin);
    logger->info("[Main] sdlwin_close is closed");
    
    // Flush logger before exit
    logger->flush();
    
    // Returns the exit code of the GTK application
    logger->info("[Main] main.cpp about to complete");
    return 0;
}

