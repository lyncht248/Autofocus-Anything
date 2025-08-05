/**
 * @file vidframe.hpp
 * @brief Data structures for video frames.
 */

#ifndef HVIGTK_VIDFRAME_H
#define HVIGTK_VIDFRAME_H

#include <cvd/image.h>
#include <VimbaCPP/Include/VimbaCPP.h>

/*
 * Data structures for frames (definitions in system.cpp)
*/

/**
 * @struct Frame
 * @brief Represents a video frame with metadata.
 *
 * This structure contains the buffer and metadata for a video frame, including
 * dimensions, pixel format, and offsets.
 */
struct Frame
{
public:
    /**
     * @brief Default constructor for Frame.
     */
    Frame();

    /**
     * @brief Copy constructor for Frame.
     *
     * Creates a deep copy of the frame buffer and metadata.
     * @param other The Frame object to copy.
     */
    Frame(const Frame &other);

    /**
     * @brief Destructor for Frame.
     *
     * Cleans up the allocated buffer.
     */
    ~Frame();

		/**
		 * @brief Pointer to the frame buffer.
		 */
		VmbUchar_t *buffer;
		/**
		 * @brief Size. width, height, X-offset, Y-offset of the frame buffer.
		 */
		VmbUint32_t bufsize, width, height, xoff, yoff;
		/**
		 * @brief Pixel format of the frame.
		 */
		VmbPixelFormatType pixf;
};

//(I think) Image has data management, so you must delete, but BasicImage does not, so don't need to delete
/**
 * @typedef VidFrame
 * @brief Alias for CVD::BasicImage using VmbUchar_t.
 */
using VidFrame = CVD::BasicImage<VmbUchar_t>;

/**
 * @typedef IVidFrame
 * @brief Alias for CVD::Image using VmbUchar_t.
 */
using IVidFrame = CVD::Image<VmbUchar_t>;

#endif
