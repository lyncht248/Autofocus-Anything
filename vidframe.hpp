#ifndef HVIGTK_VIDFRAME_H
#define HVIGTK_VIDFRAME_H

#include <cvd/image.h>
#include <VimbaCPP/Include/VimbaCPP.h>

/*
 * Data structures for frames (definitions in system.cpp)
*/

struct Frame
{
	public: 
		Frame();
		Frame(const Frame &other);
		~Frame();

		VmbUchar_t *buffer;
		VmbUint32_t bufsize, width, height, xoff, yoff;
		VmbPixelFormatType pixf;
};

//(I think) Image has data management, so you must delete, but BasicImage does not, so don't need to delete
using VidFrame = CVD::BasicImage<VmbUchar_t>;
using IVidFrame = CVD::Image<VmbUchar_t>;

#endif
