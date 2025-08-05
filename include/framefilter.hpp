/**
 * @file framefilter.hpp
 * @brief Abstract base class for frame filters.
 */

#ifndef HVIGTK_FRAMEFILTER_H
#define HVIGTK_FRAMEFILTER_H

#include <cairomm/cairomm.h>

/**
 * @class FrameFilter
 * @brief Abstract base class for applying filters to video frames.
 *
 * This class provides an interface for drawing filters on video frames using Cairo surfaces.
 */
class FrameFilter
{
public:
    /**
     * @brief Pure virtual method for drawing on a frame.
     *
     * This method must be implemented by derived classes to apply specific filters
     * to the provided Cairo image surface.
     *
     * @param sfc Reference to the Cairo image surface to draw on.
     */
    virtual void draw(::Cairo::RefPtr<::Cairo::ImageSurface> sfc) = 0;
};

#endif
