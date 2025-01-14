#ifndef HVIGTK_FRAMEFILTER_H
#define HVIGTK_FRAMEFILTER_H

#include <cairomm/cairomm.h>

class FrameFilter
{
public:
	virtual void draw(::Cairo::RefPtr< ::Cairo::ImageSurface> sfc) = 0;
};

#endif
