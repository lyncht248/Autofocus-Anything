#include "main.hpp"
#include <fstream>
#include <iostream>
#include <gtkmm.h>
#include <unistd.h>

#include "glibmm/main.h"
#include "sdlwindow.hpp"
#include "version.hpp"
#include "system.hpp"
#include "logfile.hpp"
#include "autofocus.hpp"

#include <thread>

char workingdir[4096] = "";
const char *hvigtk_startdir = workingdir;

std::ofstream hvigtk_logfile("hvigtk.log");

SDLWindow::SDLWin *childwin = nullptr;
std::atomic<bool> bAutofocusing = 0; //Flag that controls the autofocusing while() loop

Glib::Dispatcher m_errorSignal;

int gtkAppLocationX = 42;
int gtkAppLocationY = 10;

int main(int argc, char **argv) 
{
    // Spawns the SDL window, which is done in a separate process
	childwin = SDLWindow::sdlwin_open();
	if (!childwin)
		return 1;
    if ( !getcwd(workingdir, 4096) )
    {
        workingdir[0] = 0;
    }

    // Creates a GTK application object
    auto app = Gtk::Application::create(argc, argv, HVIGTK_APPID); 

    //Sets to dark theme, which is useful for operator to see contrast better
    Glib::RefPtr<Gtk::Settings> settings = Gtk::Settings::get_default();
    if(settings) {
        settings->property_gtk_application_prefer_dark_theme() = true;
    }

    // Initializes the GTK thread system and sets the GTK thread to default (?)
	if(!Glib::thread_supported() ) Glib::thread_init();
	Glib::MainContext::get_default()->push_thread_default();

    // Creates a System object to run the logic behind the app's main window
    System system(argc, argv);

    // // Creates an Autofocus object to run the autofocusing logic (perhaps should be done in system object)
    // autofocus AF;

    // //Starts an autofocusing thread (should be replaced by constructor for the AF object, I believe)
    // std::thread tAutofocus(&autofocus::run2, &AF);

    //Opens the GTK application GUI and stops main() execution. system.getWindow() returns a pointer to the MainWindow object created in system.cc
    int out = app->run(system.getWindow() );
	hvigtk_logfile << "app-> run loop finished executing in main.cpp";

    //Closes logfile when GTK application exits
	hvigtk_logfile.close();

    //Closes the SDL window
	SDLWindow::sdlwin_close(childwin);
    hvigtk_logfile << "sdlwin_close in main.cpp executed" << std::endl;

    // // Stops the autofocus thread. This should be a destructor in the autofocus object
    // if (tAutofocus.joinable() ) {
    //     tAutofocus.join(); // Stops the CaptureVideo thread too. This should be a destructor in the autofocus object instead
    //     std::cout << "Autofocus closed correctly" << std::endl;
    // }

    //Returns the exit code of the GTK application
	hvigtk_logfile << "main.cpp about to return 1";
	return 0;
}

