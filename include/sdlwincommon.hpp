/**
 * @file sdlwincommon.hpp
 * @brief Common definitions and structures for SDL window management.
 */

#ifndef SDLWINCOMMON_H
#define SDLWINCOMMON_H

#include <SDL.h>
#include <pthread.h>
#include <string>

/**
 * @def MDATASZ
 * @brief Size of metadata arrays.
 */
#define MDATASZ 1024

/**
 * @def BDATASZ
 * @brief Size of binary data arrays.
 */
#define BDATASZ 10485760

/**
 * @class System
 * @brief Forward declaration for the System class.
 */
class System;

namespace SDLWindow
{
    /**
     * @class ldata_t
     * @brief Union for storing various lightweight data types used in SDL window operations.
     *
     * This union provides storage for various primitive data types.
     */
    typedef union
    {
        bool d_bool; /**< Boolean data type. */
        char d_char; /**< Character data type. */
        unsigned char d_uchar; /**< Unsigned character data type. */
        int d_int; /**< Integer data type. */
        unsigned int d_uint; /**< Unsigned integer data type. */
        long d_long; /**< Long integer data type. */
        unsigned long d_ulong; /**< Unsigned long integer data type. */
        float d_float; /**< Floating-point data type. */
        double d_double; /**< Double-precision floating-point data type. */
    } ldata_t;

    /**
     * @typedef mdata_t
     * @brief Array for metadata storage.
     */
    typedef unsigned char mdata_t[MDATASZ];

    /**
     * @typedef bdata_t
     * @brief Array for binary data storage.
     */
    typedef unsigned char bdata_t[BDATASZ];

    /**
     * @enum Command
     * @brief Enumeration of commands for SDL window operations.
     */
    enum Command
    {
        CMD_NONE = 0, /**< No command. */
        CMD_DRAW_LINES, /**< Command to draw lines. */
        CMD_CREATE_FRAME, /**< Command to create a frame. */
        CMD_RENDER_G8, /**< Command to render grayscale images. */
        CMD_UPDATE_MAP, /**< Command to update the map. */
        CMD_RENDER_MAP, /**< Command to render the map. */
        CMD_RAISE, /**< Command to raise the window. */
        CMD_UNRAISE, /**< Command to unraise the window. */
        CMD_HIDE, /**< Command to hide the window. */
        CMD_MOVE, /**< Command to move the window. */
        CMD_QUIT, /**< Command to quit the application. */
        CMD_RESET_ZOOM /**< Command to reset the zoom level. */
    };

    /**
     * @enum Response
     * @brief Enumeration of responses for SDL window operations.
     */
    enum Response
    {
        RE_NONE = 0, /**< No response. */
        RE_EFRAME, /**< Response indicating an error frame. */
        RE_ERROR, /**< Response indicating an error. */
        RE_OK /**< Response indicating success. */
    };

    /**
     * @struct SDLWin
     * @brief Structure for managing SDL window state and operations.
     *
     * This structure contains fields for managing commands, responses, raster positions,
     * zoom levels, ROI display, stabilization offsets, and depth mapping.
     */
    struct SDLWin
    {
        int pid; /**< Process ID associated with the SDL window. */
        pthread_mutex_t mutex; /**< Mutex for synchronizing access. */
        pthread_cond_t hasCommand, hasResponse; /**< Condition variable for command and response availability. */
        Command command; /**< Current command for the SDL window. */
        Response response; /**< Current response from the SDL window. */
        SDL_Rect raster; /**< Raster position and dimensions. */
        ldata_t lcmd[8], lre[8], luser[16]; /**< Lightweight command, response, and user data. */
        mdata_t mcmd[3], mre[3], muser[4]; /**< Metadata for commands, responses, and user data. */
        bdata_t bcmd, buser; /**< Binary data for commands and user data. */
        double zoomFactor; /**< Zoom factor for the SDL window. */
        double zoomOffsetX; /**< X-offset for zoom. */
        double zoomOffsetY; /**< Y-offset for zoom. */
		// Add ROI display parameters
        System *system; /**< Pointer to the System object. */
        bool showROI; /**< Flag indicating whether to show the ROI. */
        int roiCenterX, roiCenterY; /**< X- and Y-coordinate of the ROI center. */
        int roiWidth, roiHeight; /**< Width and height of the ROI. */
        bool searchComplete; /**< Flag indicating whether the search is complete. */

        // Scale bar parameters
        double micronsPerPixel = 1.0; /**< Microns per pixel scaling factor for distance measurements. */
        bool showScaleBar; /**< Flag indicating whether to show the scale bar. */

		// Stabilization offset for ROI display
		double stabOffsetX, stabOffsetY; /**< X- and Y-offset for stabilization. */
        bool stabActive; /**< Flag indicating whether stabilization is active. */

		// Depth mapping display parameters
		bool showDepthMap; /**< Flag indicating whether to show the depth map. */
        bool hasDepthMap; /**< Flag indicating whether a depth map is available. */

		// Depth map data - simple types safe for shared memory
        int depthMapWidth; /**< Width of the depth map. */
        int depthMapHeight; /**< Height of the depth map. */

		// Dynamic focus range from actual depth map data
        float focusMin; /**< Minimum focus range. */
        float focusMax; /**< Maximum focus range. */

        // The size of the depth image - increased to accommodate typical camera resolutions
        // Store actual focus positions for each pixel, negative values = no data
        static constexpr int MAX_DEPTH_WIDTH = 512; /**< Maximum supported width for depth maps (was 256). */
        static constexpr int MAX_DEPTH_HEIGHT = 384; /**< Maximum supported height for depth maps. (was 192) */
        float focusPositions[MAX_DEPTH_HEIGHT][MAX_DEPTH_WIDTH]; /**< Focus positions for depth mapping. */
        bool depthMapReady; /**< Flag indicating whether the depth map is ready. */
        int depthDataVersion; /**< Track when depth data changes. Version of the depth data. */
    };
}

#endif
