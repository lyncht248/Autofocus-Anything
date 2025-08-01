/**
 * @file sdlwinchild.hpp
 * @brief Header file for SDL child process management.
 * 
 * This file defines the interface for the SDL child process, which handles rendering
 * and user interactions in a separate process to avoid conflicts with the main thread.
 */

#ifndef SDLWINCHILD_H
#define SDLWINCHILD_H

#include "sdlwincommon.hpp"

/**
 * @namespace SDLWindow
 * @brief Namespace for SDL window-related functionality.
 * 
 * The SDLWindow namespace includes functions and variables for managing SDL windows
 * and rendering operations in a child process.
 */
namespace SDLWindow
{
    /**
     * @var sdlwin
     * @brief Pointer to the shared SDL window structure.
     * 
     * This variable is used to access the shared memory structure for the SDL window.
     */
    extern SDLWin *sdlwin;

    /**
     * @brief Entry point for the SDL child process.
     * 
     * Initializes the SDL environment, creates the window and renderer, and handles
     * events and rendering in a separate process.
     * @return Exit code for the child process.
     */
    int child_main();
}

#endif
