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
		// If nothing passed to struct Frame, buffer is null
		Frame();
		
		// If a Frame is passed to struct Frame, create matching frame
		Frame(const Frame &other);

		~Frame();

		VmbUchar_t *buffer;
		VmbUint32_t bufsize, width, height, xoff, yoff;
		VmbPixelFormatType pixf;
};

using VidFrame = CVD::Image<VmbUchar_t>;

/*
class VidFrame : public CVD::Image<VmbUchar_t>
{
	friend class FrameProcessor;
public:
	VidFrame(Frame &f);
	VidFrame(CVD::Image<VmbUchar_t> &im);
	~VidFrame();
protected:
	static Frame nullframe;
	Frame &frame;
};
*/

#endif
