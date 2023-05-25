#ifndef HVIGTK_MAIN_H
#define HVIGTK_MAIN_H

//#include "autofocus.hpp"
#include "sdlwincommon.hpp"
#include <atomic>
#include <gtkmm.h>

#define HVIGTK_STAB_LIM 100000.0

extern const char *hvigtk_startdir;

#include "sdlwindow.hpp"
extern SDLWindow::SDLWin *childwin;

//Global Variables (minimize)
extern Glib::Dispatcher m_errorSignal;
extern int gtkAppLocationX;
extern int gtkAppLocationY;

#endif
