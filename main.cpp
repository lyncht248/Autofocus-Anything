#include "main.hpp"
#include <fstream>
#include <iostream>
#include <gtkmm.h>
#include <unistd.h>

#include "glibmm/main.h"
#include "version.hpp"
#include "system.hpp"
#include "logfile.hpp"

char workingdir[4096] = "";
const char *hvigtk_startdir = workingdir;

std::ofstream hvigtk_logfile("hvigtk.log");

class A
{
public:
	A(int x) : x(x)
	{
	}
	int x;
};

int main(int argc, char **argv) 
{
    if ( !getcwd(workingdir, 4096) )
    {
        workingdir[0] = 0;
    }
    auto app = Gtk::Application::create(argc, argv, HVIGTK_APPID);
    
	Glib::MainContext::get_default()->push_thread_default();

    System system(argc, argv);
	
    int out = app->run(system.getWindow() );
	hvigtk_logfile.close();
	return out;
}
