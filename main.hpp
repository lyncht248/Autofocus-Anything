#ifndef HVIGTK_MAIN_H
#define HVIGTK_MAIN_H

//#include "autofocus.hpp"
#include "sdlwincommon.hpp"
#include <atomic>
#define HVIGTK_STAB_LIM 100000.0

extern const char *hvigtk_startdir;

#include "sdlwindow.hpp"
extern SDLWindow::SDLWin *childwin;

extern int center;
extern std::atomic<bool> bAutofocusing; //Flag that controls the autofocusing while() loop

#endif
