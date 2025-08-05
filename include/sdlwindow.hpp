/**
 * @file sdlwindow.hpp
 * @brief Declares the `SDLWindow` namespace for managing SDL windows and rendering operations.
 * 
 * This file provides the declaration of the `SDLWindow` namespace, which includes functions for
 * creating, managing, and rendering SDL windows. It also handles depth map data, user interactions,
 * and thread-safe operations.
 */

#ifndef SDLWINDOW_H
#define SDLWINDOW_H

#include <SDL.h>
#include <pthread.h>
#include <vector>
#include "sdlwincommon.hpp"

/**
 * @namespace SDLWindow
 * @brief Provides functions for managing SDL windows and rendering operations.
 * 
 * The SDLWindow namespace includes functions for creating, managing, and rendering SDL windows,
 * as well as handling depth map data and user interactions.
 */
namespace SDLWindow
{
    /**
     * @brief Locks the command mutex for the SDL window.
     * 
     * Ensures thread-safe access to the SDL window commands.
     * @param sdlwin Pointer to the SDL window structure.
     */
    static inline void lockcmd(SDLWin *sdlwin)
    {
        pthread_mutex_lock(&sdlwin->mutex);
        while (sdlwin->command)
            pthread_cond_wait(&sdlwin->hasResponse, &sdlwin->mutex);
    }

    /**
     * @brief Unlocks the command mutex for the SDL window.
     * 
     * Releases the mutex after thread-safe access.
     * @param sdlwin Pointer to the SDL window structure.
     */
    static inline void unlockcmd(SDLWin *sdlwin)
    {
        pthread_mutex_unlock(&sdlwin->mutex);
    }

    /**
     * @brief Waits for a response from the SDL window.
     * 
     * Blocks until a response is received.
     * @param sdlwin Pointer to the SDL window structure.
     * @return Response from the SDL window.
     */
    static inline Response waitForResponse(SDLWin *sdlwin)
    {
        sdlwin->response = RE_NONE;
        while (!sdlwin->response)
            pthread_cond_wait(&sdlwin->hasResponse, &sdlwin->mutex);
        return sdlwin->response;
    }

    /**
     * @brief Opens a new SDL window.
     * 
     * Creates a new SDL window and initializes shared memory.
     * @return Pointer to the newly created SDL window structure.
     */
    extern SDLWin *sdlwin_open();

    /**
     * @brief Closes an SDL window.
     * 
     * Terminates the SDL window process and cleans up resources.
     * @param win Pointer to the SDL window structure.
     */
    extern void sdlwin_close(SDLWin *win);

    /**
     * @brief Creates a new frame in the SDL window.
     * 
     * Initializes a frame with the specified dimensions.
     * @param win Pointer to the SDL window structure.
     * @param width Width of the frame.
     * @param height Height of the frame.
     * @return Response indicating the success or failure of the operation.
     */
    extern Response createFrame(SDLWin *win, int width, int height);

    /**
     * @brief Renders a grayscale frame in the SDL window.
     * 
     * Updates the frame with the provided buffer data.
     * @param win Pointer to the SDL window structure.
     * @param buf Pointer to the buffer data.
     * @param n Size of the buffer data.
     * @param xoff Horizontal offset for rendering.
     * @param yoff Vertical offset for rendering.
     * @return Response indicating the success or failure of the operation.
     */
    extern Response renderFrameG8(SDLWin *win, const void *buf = nullptr, size_t n = 0, int xoff = 0, int yoff = 0);

    /**
     * @brief Sets the raster position in the SDL window.
     * 
     * Updates the raster position with the specified coordinates.
     * @param win Pointer to the SDL window structure.
     * @param x Horizontal position.
     * @param y Vertical position.
     */
    extern void setRaster(SDLWin *win, int x = 0, int y = 0);

    /**
     * @brief Uploads data to the SDL window.
     * 
     * Transfers buffer data to the SDL window.
     * @param win Pointer to the SDL window structure.
     * @param buf Pointer to the buffer data.
     * @param n Size of the buffer data.
     * @param lock Whether to lock the mutex during the operation.
     * @param i Index for the buffer.
     * @param off Offset for the buffer data.
     * @return Updated offset after the upload.
     */
    extern unsigned long long upload(SDLWin *win, const void *buf, size_t n, bool lock = true, int i = 0, unsigned long long off = 0);

    /**
     * @brief Updates the depth map in the SDL window.
     * 
     * Transfers depth map data to the SDL window and updates its parameters.
     * @param win Pointer to the SDL window structure.
     * @param buf Pointer to the depth map data.
     * @param width Width of the depth map.
     * @param height Height of the depth map.
     * @param pitch Pitch of the depth map.
     * @return Response indicating the success or failure of the operation.
     */
    extern Response updateMap(SDLWin *win, const void *buf, int width, int height, int pitch);

    /**
     * @brief Sets whether the depth map is displayed in the SDL window.
     * 
     * Updates the display state of the depth map.
     * @param win Pointer to the SDL window structure.
     * @param value True to display the depth map, false to hide it.
     */
    extern void setShowingMap(SDLWin *win, bool value);

    /**
     * @brief Resets the zoom parameters in the SDL window.
     * 
     * Resets the zoom factor and offsets to their default values.
     * @param win Pointer to the SDL window structure.
     */
    extern void resetZoom(SDLWin *win);

    /**
     * @brief Raises the SDL window.
     * 
     * Brings the SDL window to the foreground.
     * @param win Pointer to the SDL window structure.
     */
    extern void raise(SDLWin *win);

    /**
     * @brief Unraises the SDL window.
     * 
     * Sends the SDL window to the background.
     * @param win Pointer to the SDL window structure.
     */
    extern void unraise(SDLWin *win);

    /**
     * @brief Hides the SDL window.
     * 
     * Makes the SDL window invisible.
     * @param win Pointer to the SDL window structure.
     */
    extern void hide(SDLWin *win);

    /**
     * @brief Moves the SDL window to a new position.
     * 
     * Updates the position of the SDL window.
     * @param win Pointer to the SDL window structure.
     * @param x New horizontal position.
     * @param y New vertical position.
     */
    extern void move(SDLWin *win, int x, int y);

    /**
     * @brief Transfers depth map data to the SDL window.
     * 
     * Updates the depth map data and parameters in the SDL window.
     * @param win Pointer to the SDL window structure.
     * @param depthImage Depth map data as a 2D vector.
     * @param width Width of the depth map.
     * @param height Height of the depth map.
     */
    extern void transferDepthMapData(SDLWin *win, const std::vector<std::vector<std::pair<double, double>>> &depthImage, int width, int height);

    /**
     * @brief Clears the depth map data in the SDL window.
     * 
     * Resets the depth map data and parameters to their default values.
     * @param win Pointer to the SDL window structure.
     */
    extern void clearDepthMapData(SDLWin *win);

    /**
     * @brief Forces a texture update for the depth map.
     * 
     * Ensures the depth map texture is updated with the latest data.
     */
    extern void forceDepthMapTextureUpdate();

    /**
     * @brief Quits the SDL window.
     * 
     * Terminates the SDL window process.
     * @param win Pointer to the SDL window structure.
     */
    extern void quit(SDLWin *win);
}

#endif
