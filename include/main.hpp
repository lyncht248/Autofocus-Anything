/**
 * @file main.hpp
 * @brief Header file for the main application logic.
 * 
 * This file defines global variables and includes necessary headers for the
 * main application logic, including SDL and GTK integration.
 */

#ifndef HVIGTK_MAIN_H
#define HVIGTK_MAIN_H

//#include "autofocus.hpp"
#include "sdlwincommon.hpp"
#include <atomic>
#include <gtkmm.h>

/**
 * @brief Stabilization limit constant.
 * 
 * Defines the stabilization limit for the application.
 */
#define HVIGTK_STAB_LIM 100000.0

/**
 * @brief Starting directory for the application.
 * 
 * Specifies the directory where the application starts.
 */
extern const char *hvigtk_startdir;

#include "sdlwindow.hpp"

/**
 * @brief Pointer to the SDL window object.
 * 
 * Represents the SDL window used for multimedia rendering.
 */
extern SDLWindow::SDLWin *childwin;

/**
 * @brief X-coordinate of the GTK application window.
 * 
 * Specifies the horizontal position of the GTK application window.
 */
extern int gtkAppLocationX;

/**
 * @brief Y-coordinate of the GTK application window.
 * 
 * Specifies the vertical position of the GTK application window.
 */
extern int gtkAppLocationY;

#endif
