#include "system.hpp"
#include "main.hpp"

#include <chrono>
#include <ctime>
#include <cvd/image_ref.h>
#include <set>
#include <gtkmm.h>
#include <cstring>
#include <string>
#include <thread>
#include <cmath>
#include <algorithm>
#include "VimbaC/Include/VmbCommonTypes.h"
#include "VimbaCPP/Include/Frame.h"
#include "VimbaCPP/Include/SharedPointerDefines.h"
#include "sdlwindow.hpp"
#include "thread.hpp"
#include "version.hpp"
#include "main.hpp"
#include "autofocus.hpp"
#include "notificationCenter.hpp"
#include "phasecorr_stabiliser.hpp"
#include "sharpness_analyzer.hpp"
#include "sharpness_graph.hpp"
#include "imagingcam.hpp"

bool bSystemLogFlag = 1;		 // 1 = log, 0 = no log
bool bSystemQueueLengthFlag = 0; // 1 = log, 0 = no log
bool bSystemFramesFlag = 0;		 // Used to track how each frame passes through the system

// I don't believe these three code blocks are used anywhere
/*
Frame::Frame() :
	buffer(nullptr),
	bufsize(0),
	width(0),
	height(0),
	xoff(0),
	yoff(0),
	pixf()
{
}

Frame::Frame(const Frame &other) :
	buffer(new VmbUchar_t[other.bufsize]),
	bufsize(other.bufsize),
	width(other.width),
	height(other.height),
	xoff(other.xoff),
	yoff(other.yoff),
	pixf(other.pixf)
{
	std::memcpy(buffer, other.buffer, bufsize);
}

Frame::~Frame()
{
	if (buffer != nullptr)
		delete[] buffer;
}
*/

/*
VidFrame::VidFrame(Frame &f) : CVD::Image<VmbUchar_t>(f.buffer, CVD::ImageRef(f.width, f.height) ),
	frame(f)
{
}

Frame VidFrame::nullframe = Frame();

VidFrame::VidFrame(CVD::Image<VmbUchar_t> &im) : CVD::VideoFrame<VmbUchar_t>(0, im),
	frame(nullframe)
{
}

VidFrame::~VidFrame()
{
}
*/

// Class that runs two threads: one which processes frames, and one which stabilises them
FrameProcessor::FrameProcessor(System &sys) : processed(nullptr),
											  filters(),
											  running(true),
											  _stabNewFrame(false),
											  _stabComplete(true),
											  _frameReleased(true),
											  mutex(),
											  stabMutex(),
											  stabComplete(),
											  frameReleased(),
											  system(sys),
											  offset(TooN::Zeros),
											  currentOff(TooN::Zeros),
											  rasterPos(TooN::Zeros),
											  stabQueue(4),
											  released(),
											  lastCalculatedOffset(TooN::Zeros),
											  distributedOffset(TooN::Zeros),
											  skipFactor(1),
											  framesSinceLastCalculation(0)
{
	processorThread = Glib::Threads::Thread::create(sigc::mem_fun(*this, &FrameProcessor::processFrame));
	stabThread = Glib::Threads::Thread::create(sigc::mem_fun(*this, &FrameProcessor::stabilise));
	if (bSystemFramesFlag)
	{
		logger->info("[FrameProcessor::FrameProcessor] constructor finished");
	}
}

FrameProcessor::~FrameProcessor()
{
	running = false;
	ThreadStopper::stop({processorThread, stabThread});

	if (stabQueue.size() > 0)
	{
		for (VidFrame *vframe = stabQueue.pop(); !stabQueue.empty(); vframe = stabQueue.pop())
			delete vframe;
		stabQueue.clear();
	}

	if (released.size() > 0)
	{
		for (VidFrame *vframe = released.pop(); !released.empty(); vframe = released.pop())
			delete vframe;
		released.clear();
	}
	if (bSystemFramesFlag)
	{
		logger->info("[FrameProcessor::~FrameProcessor] destructor called, processorThread and stabThread stopped");
	}
}

void FrameProcessor::stabilise()
{
	if (bSystemFramesFlag)
	{
		logger->info("[FrameProcessor::stabilise] Thread started");
	}

	ThreadStopper::makeStoppable();

	// Stabilization logic:
	// - Caps analysis at 30fps for performance at high frame rates
	// - Distributes calculated offsets smoothly across frames
	// - Example: At 60fps, calculates every 2nd frame and applies half the offset to each frame

	int frameCount = 0;

	while (running)
	{
		// If the queue is backing up, discard older frames and only keep the most recent one or two
		// so we can keep up at higher framerates.
		static const size_t MAX_ALLOWED_IN_QUEUE = 2;
		while (stabQueue.size() > MAX_ALLOWED_IN_QUEUE)
		{
			VidFrame *oldFrame = stabQueue.pop();
			delete oldFrame;
		}

		VidFrame *vframe = stabQueue.pop();

		// If the frame is null, skip it
		if (vframe == nullptr)
			continue;

		// Improved frame skipping logic - cap analysis at 30fps and distribute offset
		// This smooths stabilization at high frame rates by:
		// 1. Calculating offset at most every 30fps (e.g., every 2nd frame at 60fps)
		// 2. Distributing the calculated offset across all frames in the skip period
		// 3. Applying equal portions to maintain smooth motion
		double currentFPS = system.getFPS();
		bool skipOffsetCalculation = false;

		// Calculate skip factor to cap analysis at 30fps
		skipFactor = std::max(1, static_cast<int>(std::ceil(currentFPS / 30.0)));

		// Only calculate offset every skipFactor frames
		if (currentFPS > 30.0)
		{
			skipOffsetCalculation = (frameCount % skipFactor != 0);
		}

		frameCount++;

		// If the user wants PhaseCorr, do that
		if (system.usePhaseCorr)
		{
			if (!skipOffsetCalculation)
			{
				// Convert VidFrame -> cv::Mat
				cv::Mat fullFrame(
					vframe->size().y,
					vframe->size().x,
					CV_8UC1,
					vframe->data());

				// Extract center ROI BEFORE downscaling to avoid subpixel issues
				int roiWidth = fullFrame.cols / 2;
				int roiHeight = fullFrame.rows / 2;
				int roiX = (fullFrame.cols - roiWidth) / 2;
				int roiY = (fullFrame.rows - roiHeight) / 2;

				// Ensure ROI coordinates are even to avoid subpixel shifts
				roiX = (roiX / 2) * 2;
				roiY = (roiY / 2) * 2;
				roiWidth = (roiWidth / 2) * 2;
				roiHeight = (roiHeight / 2) * 2;

				cv::Rect roiRect(roiX, roiY, roiWidth, roiHeight);
				cv::Mat roiFrame = fullFrame(roiRect);

				// Now downscale the ROI
				cv::Mat halfFrame;
				cv::resize(roiFrame, halfFrame, cv::Size(), 0.5, 0.5);

				cv::Point2f shift(0.0f, 0.0f);
				// If reference not set, init it
				if (system.phaseCorrStabiliserReferenceNotSet)
				{
					system.phaseCorrStabiliser.initReference(halfFrame);
					system.phaseCorrStabiliserReferenceNotSet = false;
				}
				else
				{
					// Compute offset
					shift = system.phaseCorrStabiliser.computeShift(halfFrame);
				}

				stabMutex.lock();
				// Scale shift back up by factor of 2.0 (since we used 0.5 scale)
				TooN::Vector<2> calculatedOffset;
				calculatedOffset[0] = shift.x * -2.0f;
				calculatedOffset[1] = shift.y * -2.0f;

				// Store the full calculated offset and distribute it
				lastCalculatedOffset = calculatedOffset;
				distributedOffset = lastCalculatedOffset / static_cast<double>(skipFactor);
				framesSinceLastCalculation = 0;

				// Apply the distributed portion
				currentOff = distributedOffset;
				offset += currentOff;
				_stabComplete = stabQueue.empty();
				if (_stabComplete)
					stabComplete.broadcast();
				stabMutex.unlock();
			}
			else
			{
				// Apply the distributed portion of the last calculated offset
				stabMutex.lock();
				framesSinceLastCalculation++;

				// Apply the distributed offset to smooth motion
				currentOff = distributedOffset;
				offset += currentOff;

				_stabComplete = stabQueue.empty();
				if (_stabComplete)
					stabComplete.broadcast();
				stabMutex.unlock();
			}
		}
		else
		{
			// Original stabiliser method
			if (system.stabiliser.is_valid())
			{
				if (!skipOffsetCalculation)
				{
					TooN::Vector<2> calculatedOffset = system.stabiliser.stabilise(*vframe, offset);
					stabMutex.lock();

					// Store the full calculated offset and distribute it
					lastCalculatedOffset = calculatedOffset;
					distributedOffset = lastCalculatedOffset / static_cast<double>(skipFactor);
					framesSinceLastCalculation = 0;

					// Apply the distributed portion
					currentOff = distributedOffset;
					offset += currentOff;
					_stabComplete = stabQueue.empty();
					if (_stabComplete)
						stabComplete.broadcast();
					stabMutex.unlock();
				}
				else
				{
					// Apply the distributed portion of the last calculated offset
					stabMutex.lock();
					framesSinceLastCalculation++;

					// Apply the distributed offset to smooth motion
					currentOff = distributedOffset;
					offset += currentOff;

					_stabComplete = stabQueue.empty();
					if (_stabComplete)
						stabComplete.broadcast();
					stabMutex.unlock();
				}
			}
			else
			{
				stabMutex.lock();
				_stabComplete = stabQueue.empty();
				if (_stabComplete)
					stabComplete.broadcast();
				stabMutex.unlock();
			}
		}

		// HERE testing releasing of all frames at the end of processFrames
		// released.push(vframe);
	}
}

void FrameProcessor::processFrame() // What to do with each frame received from FrameObserver
{
	if (bSystemFramesFlag)
	{
		logger->info("[FrameProcessor::processFrame] Thread started");
	}

	ThreadStopper::makeStoppable();
	while (running)
	{
		// Get the frame when it's ready
		if (bSystemFramesFlag)
		{
			logger->info("[FrameProcessor::processFrame] About to take vidFrame from FrameQueue");
		}
		ThreadStopper::lock(mutex);
		vidFrame = system.getFrameQueue().pop(); // Takes frame from top of queue
		_frameReleased = false;
		ThreadStopper::unlock(mutex);
		if (bSystemFramesFlag)
		{
			logger->info("[FrameProcessor::processFrame] vidFrame taken from FrameQueue");
		}

		// If stabilising, put frame in Stabiliser queue
		bool stabilising = system.window.getStabiliseActive().getValue();
		if (stabilising)
		{
			stabMutex.lock();
			_stabComplete = false;
			//_stabNewFrame = true;
			// stabNewFrame.signal();
			stabMutex.unlock();
			stabQueue.push(vidFrame);
		}

		// recording = 1 if incoming frames need to be added to the recorder
		bool recording = system.window.getRecording().getValue() && !system.window.getPausedRecording().getValue();

		if (bSystemFramesFlag)
		{
			logger->info("[FrameProcessor::processFrame] recording = {}", recording);
		}
		if (bSystemFramesFlag)
		{
			logger->info("[FrameProcessor::processFrame] stabilising = {}", stabilising);
		}

		// If recording, sends the frame to the recorder for saving
		if (recording)
		{
			system.getRecorder().putFrame(vidFrame); // putFrame is just frames.push_back(frame); until frames=recording size
		}

		// Draw the frame at the size neccessary to fit the window
		CVD::ImageRef vfDim = vidFrame->size();
		if (bSystemFramesFlag)
		{
			logger->info("[FrameProcessor::processFrame] vidFrame size: {}x{}, about to draw", vfDim.x, vfDim.y);
		}
		// Recreate frame if dimensions are different, but raster.w and raster.h should be correct
		if (vfDim.x != childwin->raster.w || vfDim.y != childwin->raster.h)
		{
			// processed = Cairo::ImageSurface::create(Cairo::FORMAT_RGB24, vfDim.x, vfDim.y);
			SDLWindow::createFrame(childwin, vfDim.x, vfDim.y);
			if (bSystemFramesFlag)
			{
				logger->info("[FrameProcessor::processFrame] SDLWindow::createFrame called");
			}
		}

		// convertToCairo(stabilising);

		// Adds the Vessel map on-top if neccessary
		for (auto pair : filters)
		{
			pair.second->draw(processed);
			if (bSystemFramesFlag)
			{
				logger->info("[FrameProcessor::processFrame] filter drawn");
			}
		}

		// If stabilising, render frame using stabilised offset, otherwise just render normally
		if (stabilising)
		{
			double stabWait = system.window.getStabWaitScaleValue();
			stabMutex.lock();
			if (stabWait > 0 && stabWait < 999999)
			{
				gint64 end_time = g_get_monotonic_time() + HVIGTK_STAB_LIM - stabWait;
				while (!_stabComplete)
					if (!stabComplete.wait_until(stabMutex, end_time))
						break;
			}
			else if (stabWait == 0.0)
				while (!_stabComplete)
					stabComplete.wait(stabMutex);

			rasterPos[0] -= currentOff[0];											// x offset
			rasterPos[1] -= (currentOff[1] + (system.getFPS() > 30.0 ? 0.5 : 1.0)); // y offset
			currentOff = TooN::Zeros;
			stabMutex.unlock();
			SDLWindow::setRaster(childwin, rasterPos[0], rasterPos[1]);

			// Update stabilization offset in shared memory for ROI display
			if (childwin)
			{
				pthread_mutex_lock(&childwin->mutex);
				childwin->stabOffsetX = rasterPos[0];
				childwin->stabOffsetY = rasterPos[1];
				childwin->stabActive = true;
				pthread_mutex_unlock(&childwin->mutex);
			}
		}
		else
		{
			SDLWindow::setRaster(childwin);

			// Clear stabilization offset when not stabilizing
			if (childwin)
			{
				pthread_mutex_lock(&childwin->mutex);
				childwin->stabOffsetX = 0.0;
				childwin->stabOffsetY = 0.0;
				childwin->stabActive = false;
				pthread_mutex_unlock(&childwin->mutex);
			}

			if (bSystemFramesFlag)
			{
				logger->info("[FrameProcessor::processFrame] SDLWindow::setRaster called");
			}
		}
		system.signalNewFrame().emit();
		if (bSystemFramesFlag)
		{
			logger->info("[FrameProcessor::processFrame] system.signalNewFrame called");
		}

		// Now that the current vframe is processed, release it
		if (bSystemFramesFlag)
		{
			logger->info("[FrameProcessor::processFrame] About to release vidFrame, locking mutex");
		}
		ThreadStopper::lock(mutex);
		{
			// When a frame is drawn, it is signalled for release (see code below)
			// window.signalFrameDrawn().connect(sigc::mem_fun(*this, &System::releaseFrame) );

			// Wait until signal that frame can be released (ie. it is drawn)
			while (!_frameReleased)
			{
				ThreadStopper::stopPoint(&frameReleased);
				frameReleased.wait(mutex);
			}
			if (bSystemFramesFlag)
			{
				logger->info("[FrameProcessor::processFrame] _frameReleased is True");
			}

			// If not stabilising, add frame to released queue. If you are stabilising, it should already have been added
			// if (!stabilising)
			// {
			released.push(vidFrame);
			if (bSystemFramesFlag)
			{
				logger->info("[FrameProcessor::processFrame] vidFrame pushed to released queue");
			}
			// }

			// //// PREVIOUS ////
			// // If live view is ON and not recording, delete every frame in the released queue
			// if (system.window.getLiveView().getValue() && !recording)
			// {
			// 	// Deletes every frame in the released queue... same as released.clear()?
			// 	for (VidFrame *vframe = released.pop(); !released.empty(); vframe = released.pop() )
			// 		delete vframe;
			// 	vidFrame = nullptr;
			// }

			// // Otherwise, delete the frame! (if live view is off (ie. on a recording) or live view is on and currently taking a recording), then clear the released queue
			// else
			// {
			// 	released.clear();
			// 	if(bSystemLogFlag) {logger->info("[FrameProcessor::processFrame] released queue cleared, but NOT deleted!!!");}
			// }

			//// NEW ////
			if (recording || !system.window.getLiveView().getValue())
			{
				// If recording or viewing a recording, then don't delete the frame data (which is pointed to by recorder::frames), but clear the released queue of pointers
				released.clear();

				if (bSystemLogFlag)
				{
					logger->info("[FrameProcessor::processFrame] released queue cleared, but data not deleted!!!");
				}
			}
			else
			{
				// Not recording, so can delete the released frames.
				// while(!released.empty()) {
				// 	VidFrame *vframe = released.pop();
				// 	delete vframe;
				// }
				// if(bSystemFramesFlag) logger->info("[FrameProcessor::processFame] released queue cleared");
				//     // Not recording, so can delete the released frames.
				// for (VidFrame *vframe = released.pop(); !released.empty(); vframe = released.pop() )
				// 	delete vframe;
				// vidFrame = nullptr;
				// released.clear();

				// This code fixes a problem where the frame memory would leak (steadily increasing RAM use) even when not recording
				//  using >=5 because often the frame data is still being used to draw the frame when this code is called
				if (released.size() >= 5)
				{
					VidFrame *frame_to_delete = released.pop(); // remove the frame from the queue
					delete frame_to_delete;						// delete the frame
					if (bSystemFramesFlag)
					{
						logger->info("[FrameProcessor::ProcessFrame] Deleted a frame from the released queue.");
					}
				}
			}
		}
		ThreadStopper::unlock(mutex);

		// stabQueue.size() = 0 almost always
		if (bSystemFramesFlag)
		{
			logger->info("[FrameProcessor::processFrame] stabQueue is length: {}", stabQueue.size());
		}

		// released.size() = 0 almost always
		if (bSystemFramesFlag)
		{
			logger->info("[FrameProcessor::processFrame] released queue is length: {}", released.size());
		}

		if (bSystemFramesFlag)
		{
			logger->info("[FrameProcessor::processFrame] frameQueue is length: {}", system.getFrameQueue().size());
		}

		if (system.sharpnessUpdateEnabled)
		{
			auto now = std::chrono::steady_clock::now();
			auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
							   now - system.lastSharpnessUpdate)
							   .count();

			// Update sharpness graph at 4fps (250ms) instead of 1fps
			if (elapsed > 250)
			{
				system.lastSharpnessUpdate = now;

				// Analyze the current frame
				if (vidFrame)
				{
					system.currentSharpnessValues = system.sharpnessAnalyzer.analyzeFrame(
						vidFrame->data(), vidFrame->size().x, vidFrame->size().y);
					system.sigSharpnessUpdated.emit();
				}
			}
		}

		// Imaging camera monitoring will be started only when user double-clicks to set ROI

		// Check for double-click events from SDL window
		if (childwin && vidFrame)
		{
			pthread_mutex_lock(&childwin->mutex);
			if (childwin->lcmd[12].d_bool)
			{										   // Check if we have a new ROI center
				int imageX = childwin->lcmd[10].d_int; // Already converted to image coordinates by SDL child
				int imageY = childwin->lcmd[11].d_int;
				childwin->lcmd[12].d_bool = false; // Clear the flag

				// Calculate ROI size based on zoom level to keep screen pixels constant
				// At 1.0x zoom: 150 screen pixels = 150 image pixels
				// At 2.0x zoom: 150 screen pixels = 75 image pixels
				double zoomFactor = childwin->zoomFactor;
				int roiSizePixels = (int)round(150.0 / zoomFactor);
				roiSizePixels = std::max(20, std::min(roiSizePixels, 500)); // Clamp between 20 and 500 pixels

				// Get frame dimensions for boundary checking
				CVD::ImageRef frameSize = vidFrame->size();

				// Ensure ROI doesn't go outside frame bounds
				int halfRoi = roiSizePixels / 2;
				imageX = std::max(halfRoi, std::min(imageX, frameSize.x - halfRoi));
				imageY = std::max(halfRoi, std::min(imageY, frameSize.y - halfRoi));

				pthread_mutex_unlock(&childwin->mutex);

				// Set the ROI size first
				system.getImagingCam()->setROISize(roiSizePixels, roiSizePixels);

				// Then set the center
				system.updateROICenter(imageX, imageY);

				// Start ROI-based focus search
				if (bSystemLogFlag)
				{
					logger->info("[FrameProcessor::processFrame] Starting imaging camera focus search at ({}, {}) with ROI size {}x{}",
								 imageX, imageY, roiSizePixels, roiSizePixels);
				}
				system.getImagingCam()->startROIFocusSearch();
			}
			else
			{
				pthread_mutex_unlock(&childwin->mutex);
			}
		}
	}
}
void FrameProcessor::releaseFrame()
{
	mutex.lock();
	_frameReleased = true;
	frameReleased.broadcast();
	mutex.unlock();
}

void FrameProcessor::resetRaster()
{
	rasterPos = offset = currentOff = TooN::Zeros;
	lastCalculatedOffset = distributedOffset = TooN::Zeros;
	skipFactor = 1;
	framesSinceLastCalculation = 0;
	SDLWindow::setRaster(childwin);
	if (bSystemLogFlag)
	{
		logger->info("[FrameProcessor::resetRaster] Stabilise offset reset to 0");
	}
}

void FrameProcessor::resetZoom()
{
	// Access the childwin member which is private to this class
	if (childwin)
	{
		SDLWindow::resetZoom(childwin);
	}
}

::Cairo::RefPtr<::Cairo::Surface> FrameProcessor::getFrame()
{
	return processed;
}

// This class is used to receive frames from the camera, but passes frames immediately to FrameProcessor
class FrameObserver : public Glib::Object, virtual public Vimba::IFrameObserver
{
public:
	FrameObserver(Vimba::CameraPtr pCamera, TSQueue<VidFrame *> &frameQueue) : IFrameObserver(pCamera), Object(),
																			   sigReceived(),
																			   sysFrames(frameQueue)
	{
		if (bSystemFramesFlag)
		{
			logger->info("[FrameObserver::FrameObserver] constructor finished");
		}
	}
	~FrameObserver()
	{
		if (sysFrames.size() > 0)
		{
			for (VidFrame *vframe = sysFrames.pop(); !sysFrames.empty(); vframe = sysFrames.pop())
				delete vframe;
			sysFrames.clear();
		}
	}

	void FrameReceived(const Vimba::FramePtr pFrame)
	{
		VmbFrameStatusType eReceiveStatus;
		if (VmbErrorSuccess == pFrame->GetReceiveStatus(eReceiveStatus))
		{
			if (VmbFrameStatusComplete == eReceiveStatus)
			{

				VmbUint32_t sz, width, height;
				pFrame->GetImageSize(sz); // TODO: These should be called once, not every frame
				pFrame->GetWidth(width);
				pFrame->GetHeight(height);

				if (bSystemFramesFlag)
				{
					logger->info("[FrameObserver::FrameReceived] Frame size: {}x{}", width, height);
				}
				IVidFrame *sysFrame = new IVidFrame(CVD::ImageRef(width, height)); // Each of these must get deleted when processed by FrameProcessor

				if (bSystemFramesFlag)
				{
					logger->info("[FrameObserver::FrameReceived] sysFrame created");
				}
				// sysFrame->resize(CVD::ImageRef(width, height) );

				// pFrame->GetOffsetX(sysFrame.xoff);
				// pFrame->GetOffsetY(sysFrame.yoff);

				VmbUchar_t *buf = nullptr;
				pFrame->GetImage(buf);
				memcpy(sysFrame->data(), buf, sz);
				sysFrames.push(sysFrame);
				if (bSystemFramesFlag)
				{
					logger->info("[FrameObserver::FrameReceived] sysFrame pushed to sysFrames queue");
				}

				// Here is where we check the queue size and drop a frame if needed. TODO: just change in frameQueue in System::System
				if (sysFrames.size() >= 15)
				{
					VidFrame *frame_to_delete = sysFrames.pop(); // remove the frame from the queue
					delete frame_to_delete;						 // delete the frame
					if (bSystemFramesFlag)
					{
						logger->info("[FrameObserver::FrameReceived] Popped a frame (should auto-delete) from the sysFrames queue as it was too long.");
					}
				}

				// Quickly recieves 15 frames which are then emptied by FrameProcessor quickly
				if (bSystemFramesFlag)
				{
					logger->info("[FrameObserver::FrameReceived] sysFrames queue is length: {}", sysFrames.size());
				}

				sigReceived.emit();
			}
		}
		m_pCamera->QueueFrame(pFrame);
		if (bSystemFramesFlag)
		{
			logger->info("[FrameObserver::FrameReceived] Frame queued");
		}
	}

	using SignalReceived = sigc::signal<void()>;
	SignalReceived signalReceived()
	{
		return sigReceived;
	}

private:
	SignalReceived sigReceived;
	TSQueue<VidFrame *> &sysFrames;
};

struct System::Private
{
	Vimba::IFrameObserverPtr observer;
	Vimba::FramePtrVector frames;

public:
	Private();
};

System::Private::Private() : observer(),
							 frames(3)
{
}

System::System(int argc, char **argv) : window(),
										vsys(Vimba::VimbaSystem::GetInstance()),
										cam(),
										size(),
										im(),
										priv(new Private()),
										sigNewFrame(hvigtk_threadcontext()),
										frameQueue(100),
										frameProcessor(*this),
										thread(),
										recorder(thread.createObject<Recorder, System &>(*this)),
										sRecorderOperationComplete(),
										stabiliser(),
										madeMap(false),
										AF(),
										sharpnessAnalyzer(),
										sigSharpnessUpdated(),
										currentSharpnessValues(),
										sharpnessUpdateEnabled(true),
										lastSharpnessUpdate(std::chrono::steady_clock::now()),
										imagingCam(std::make_unique<ImagingCam>(*this))
{
	if (bSystemLogFlag)
	{
		logger->info("[System::System] constructor beginning");
	}

	// //Error message handling! DEPRECATED
	// NotificationCenter::instance().registerListener("outOfBoundsError", [this]() {
	// 	on_error();
	// });
	// Registering listeners for device disconnections
	NotificationCenter::instance().registerListener("TiltedCamDisconnected", [&]()
													{
		tiltedCamDisconnected = true;
		handleDisconnection(); });

	NotificationCenter::instance().registerListener("ImagingCamDisconnected", [&]()
													{
		imagingCamDisconnected = true;
		handleDisconnection(); });

	NotificationCenter::instance().registerListener("LensDisconnected", [&]()
													{
		lensDisconnected = true;
		handleDisconnection(); });

	AF.initialize();

	recorder->connectTo(&sRecorderOperationComplete);
	sRecorderOperationComplete.connect(sigc::mem_fun(*this, &System::onRecorderOperationComplete));

	window.signal_map_event().connect([this](GdkEventAny * /*event*/) -> bool
									  {
										  window.move(gtkAppLocationX, gtkAppLocationY); // Replace with your desired coordinates.
										  return false;									 // Continue propagation.
									  });

	window.signalFrameDrawn().connect(sigc::mem_fun(*this, &System::releaseFrame));
	window.signalFeatureUpdated().connect(sigc::mem_fun(*this, &System::onWindowFeatureUpdated));
	// window.signalThresholdChanged().connect(sigc::mem_fun(*this, &System::onWindowThresholdChanged) );
	// window.signalScaleChanged().connect(sigc::mem_fun(*this, &System::onWindowScaleChanged) );
	window.signalBestFocusChanged().connect(sigc::mem_fun(*this, &System::onWindowBestFocusChanged));
	window.signalPauseClicked().connect(sigc::mem_fun(*this, &System::onWindowPauseClicked));
	sigNewFrame.connect(sigc::mem_fun(*this, &System::renderFrame));

	window.signalEnterClicked().connect(sigc::mem_fun(*this, &System::onWindowEnterClicked));

	// CONDITIONS

	window.getLiveView().signalToggled().connect(sigc::mem_fun(*this, &System::whenLiveViewToggled));

	// window.getMakeMapActive().signalToggled().connect(sigc::mem_fun(*this, &System::whenMakeMapToggled) );
	window.getStabiliseActive().signalToggled().connect(sigc::mem_fun(*this, &System::whenStabiliseToggled));
	// window.getShowMapActive().signalToggled().connect(sigc::mem_fun(*this, &System::whenShowMapToggled) );

	window.signalResetClicked().connect(sigc::mem_fun(*this, &System::onResetClicked));
	window.signalRecenterClicked().connect(sigc::mem_fun(*this, &System::onRecenterClicked));
	window.signalGetDepthsClicked().connect(sigc::mem_fun(*this, &System::onGetDepthsClicked));
	window.getGetDepthsActive().signalToggled().connect(sigc::mem_fun(*this, &System::whenGetDepthsToggled));

	window.getHoldFocusActive().signalToggled().connect(sigc::mem_fun(*this, &System::whenHoldFocusToggled));
	window.getViewDepthsActive().signalToggled().connect(sigc::mem_fun(*this, &System::whenViewDepthsToggled));
	window.get3DStabActive().signalToggled().connect(sigc::mem_fun(*this, &System::when3DStabToggled));
	window.get2DStabActive().signalToggled().connect(sigc::mem_fun(*this, &System::when2DStabToggled));

	window.getLoading().signalToggled().connect(sigc::mem_fun(*this, &System::whenLoadingToggled));
	window.getSaving().signalToggled().connect(sigc::mem_fun(*this, &System::whenSavingToggled));
	window.getPlayingBuffer().signalToggled().connect(sigc::mem_fun(*this, &System::whenPlayingToggled));
	window.getSeeking().signalToggled().connect(sigc::mem_fun(*this, &System::whenSeekingToggled));
	window.getRecording().signalToggled().connect(sigc::mem_fun(*this, &System::whenRecordingToggled));

	// Signals that the window has been closed, and other threads must close
	window.signal_delete_event().connect(sigc::mem_fun(*this, &System::onCloseClicked));

	auto now = std::chrono::system_clock::now();
	std::time_t time = std::chrono::system_clock::to_time_t(now);

	char buf[64] = {0};
	std::strftime(buf, sizeof(buf), "%F %R %Z", std::localtime(&time));

	if (vsys.Startup() == VmbErrorSuccess)
	{
		if (bSystemLogFlag)
		{
			logger->info("[System::System] Starting up Vimba: success");
		}
		Vimba::CameraPtrVector cameras;

		if (vsys.GetCameras(cameras) == VmbErrorSuccess)
		{
			if (bSystemLogFlag)
			{
				logger->info("[System::System] Found: {} cameras", cameras.size());
			}
			if (!cameras.empty())
			{
				cam = cameras[0];
				/*
				cam->GetFeatureByName("PixelFormat", pFeature);
				pFeature->SetValue(VmbPixelFormatMono12);
				*/
			}
			else
			{
				if (bSystemLogFlag)
				{
					logger->info("[System::System] Failed to find cameras");
				}
				NotificationCenter::instance().postNotification("ImagingCamDisconnected");
			}
		}
		else
		{
			if (bSystemLogFlag)
			{
				logger->info("[System::System] Failed to find cameras");
			}
		}
	}
	else
	{
		logger->error("[System::System] Failed to start Vimba");
	}
	whenLiveViewToggled(true); // calls startStreaming

	NotificationCenter::instance().registerListener("outOfBoundsError", [&]()
													{ window.showOutOfBoundsWarning(); });

	// Add a listener for when lens moves back in bounds
	NotificationCenter::instance().registerListener("lensInBounds", [&]()
													{ window.hideOutOfBoundsWarning(); });

	// In the System constructor, connect the signal
	window.signalHomePositionChanged().connect(
		sigc::mem_fun(*this, &System::onWindowHomePositionChanged));

	window.signalPGainChanged().connect(sigc::mem_fun(*this, &System::onWindowPGainChanged));

	sigSharpnessUpdated.connect(sigc::mem_fun(*this, &System::updateSharpnessGraph));

	// Set up imaging camera callbacks for GUI integration
	if (getImagingCam())
	{
		getImagingCam()->setHoldFocusCallback([this](bool enabled)
											  {
			// Use a dispatcher to safely update GUI from another thread
			Glib::signal_idle().connect_once([this, enabled]() {
				window.setHoldFocus(enabled);
			}); });

		getImagingCam()->setBestFocusCallback([this](int position)
											  {
			// Use a dispatcher to safely update GUI from another thread
			Glib::signal_idle().connect_once([this, position]() {
				imagingCamUpdatingFocus = true; // Prevent ROI clearing during imaging camera update
				window.setBestFocusScaleValue(static_cast<double>(position));
				imagingCamUpdatingFocus = false; // Reset flag
			}); });

		// Add the new search complete callback
		getImagingCam()->setSearchCompleteCallback([this](bool successful)
												   {
			if (childwin) {
				pthread_mutex_lock(&childwin->mutex);
				childwin->searchComplete = successful;
				pthread_mutex_unlock(&childwin->mutex);
			} });
	}

	// Imaging camera monitoring will be started when user double-clicks to set ROI
}

bool System::startStreaming()
{
	bool FPSsuccess = false;

	std::cout << "Starting streaming" << std::endl;
	if (cam != nullptr)
	{
		std::cout << "Camera not null" << std::endl;
		if (cam->Open(VmbAccessModeFull) == VmbErrorSuccess)
		{
			std::cout << "Camera opened" << std::endl;
			if (bSystemLogFlag)
			{
				logger->info("[System::startStreaming] Successfully opened camera");
			}

			window.updateCameraValues(getFeature<double>("Gain"), getFeature<double>("ExposureTime"), getFeature<double>("Gamma"));
			setFeature("PixelFormat", VmbPixelFormatMono8);
			setFeature("AcquisitionFrameRateEnable", true);
			FPSsuccess = setFeature<double>("AcquisitionFrameRate", window.getFrameRateEntryBox());

			Vimba::FeaturePtr pFeature;
			cam->GetFeatureByName("PayloadSize", pFeature);
			VmbInt64_t nPLS;
			pFeature->GetValue(nPLS);

			priv->observer.reset(new FrameObserver(cam, frameQueue)); // reset means its a smart pointer and will delete itself when it goes out of scope

			for (Vimba::FramePtrVector::iterator iter = priv->frames.begin(); iter != priv->frames.end(); ++iter) // iter is the frame number
			{
				(*iter).reset(new Vimba::Frame(nPLS)); // create a frame of the correct size
				if (bSystemFramesFlag)
				{
					logger->info("[System::startStreaming] Frame {} created", iter - priv->frames.begin());
				}
				(*iter)->RegisterObserver(priv->observer); // register the observer
				if (bSystemFramesFlag)
				{
					logger->info("[System::startStreaming] Frame {} registered", iter - priv->frames.begin());
				}
				cam->AnnounceFrame(*iter); // announce the frame to the camera
				if (bSystemFramesFlag)
				{
					logger->info("[System::startStreaming] Frame {} announced", iter - priv->frames.begin());
				}
			}

			if (cam->StartCapture() == VmbErrorSuccess)
			{
				std::cout << "Started capture" << std::endl;
				for (Vimba::FramePtrVector::iterator iter = priv->frames.begin(); iter != priv->frames.end(); ++iter)
				{
					cam->QueueFrame(*iter);
					if (bSystemFramesFlag)
					{
						logger->info("[System::startStreaming] Frame {} queued", iter - priv->frames.begin());
					}
				}

				if (cam->GetFeatureByName("AcquisitionStart", pFeature) == VmbErrorSuccess && pFeature->RunCommand() == VmbErrorSuccess)
				{
					if (bSystemLogFlag)
					{
						logger->info("[System::startStreaming] Started frame capture");
					}
					if (bSystemFramesFlag)
					{
						logger->info("[System::startStreaming] Started camera aquisition");
					}
				}
				else
				{
					std::cout << "goto fail" << std::endl;
					goto fail;
				}
			}
			else
			{
			fail:;
				std::cout << "At fail" << std::endl;
				NotificationCenter::instance().postNotification("ImagingCamDisconnected");
				if (bSystemLogFlag)
				{
					logger->info("[System::startStreaming] Failed to start frame capture");
				} // TODO might need to mem manage below here
			}
		}
	}
	return FPSsuccess;
}

void System::stopStreaming()
{
	if (cam != nullptr)
	{
		Vimba::FeaturePtr pFeature;

		if (cam->GetFeatureByName("AcquisitionStop", pFeature) == VmbErrorSuccess && pFeature->RunCommand() == VmbErrorSuccess)
		{
			if (bSystemLogFlag)
			{
				logger->info("[System::stopStreaming] Stopped acquisition");
			}
		}

		if (cam->EndCapture() == VmbErrorSuccess)
		{
			if (bSystemLogFlag)
			{
				logger->info("[System::stopStreaming] Stopped capture");
			}
		}

		if (cam->FlushQueue() == VmbErrorSuccess)
		{
			if (bSystemLogFlag)
			{
				logger->info("[System::stopStreaming] Flushed queue");
			}
		}

		for (Vimba::FramePtrVector::iterator iter = priv->frames.begin(); iter != priv->frames.end(); ++iter)
		{
			(*iter)->UnregisterObserver();
		}

		if (cam->RevokeAllFrames() == VmbErrorSuccess)
		{
			if (bSystemLogFlag)
			{
				logger->info("[System::stopStreaming] Revoked frames");
			}
		}

		if (cam->Close() == VmbErrorSuccess)
		{
			if (bSystemLogFlag)
			{
				logger->info("[System::stopStreaming] Closed camera");
			}
		}
	}
}

TSQueue<VidFrame *> &System::getFrameQueue()
{
	return frameQueue;
}

Glib::Dispatcher &System::signalNewFrame()
{
	return sigNewFrame;
}

VidFrame *System::getFrame()
{
	return frameProcessor.vidFrame;
}

void System::renderFrame()
{
	window.renderFrame(frameProcessor.vidFrame);
}

void System::releaseFrame()
{
	// if (window.getMakeMapActive().getValue() && !madeMap)
	// {
	// 	stabiliser.make_map(*getFrame(), DEFAULT_NUM_TRACKERS);
	// 	if(!window.get3DStabActive().getValue() && !window.get2DStabActive().getValue()) { //If either 3D or 2D stab is active, don't show map
	// 		window.setShowingMap(true);
	// 	}
	// 	madeMap = true;
	// }
	frameProcessor.releaseFrame();
}

void System::whenLiveViewToggled(bool viewingLive)
{
	// window.setMakingMap(false);
	if (viewingLive)
	{
		if (bSystemLogFlag)
		{
			logger->info("[System::whenLiveViewToggled] Live view toggled on");
		}
		startStreaming();

		// If there are frames in the recorder, delete them now
		if (recorder->countFrames() > 0)
		{

			recorder->clearFrames();
		}
		window.setHasBuffer(false);
	}
	else
	{
		if (bSystemLogFlag)
		{
			logger->info("[System::whenLiveViewToggled] Live view toggled off");
		}
		stopStreaming();

		// Clear any existing ROI when switching to recording view
		clearROIDisplay();
	}
}

// DEPRECATED FUNCTIONALITY related to stabilizing
//  void System::whenMakeMapToggled(bool makingMap)
//  {
//  	frameProcessor.stabMutex.lock();
//  	while (!frameProcessor._stabComplete)
//  		frameProcessor.stabComplete.wait(frameProcessor.stabMutex);
//  	if (makingMap)
//  	{
//  		if(bSystemLogFlag) {logger->info("[System::whenMakeMapToggled] Making map toggled on");}
//  		frameProcessor.resetRaster();
//  		if (!window.getLiveView().getValue() )
//  		{
//  			VidFrame *vframe = recorder->getFrame(window.getFrameSliderValue() );
//  			if (vframe)
//  			{
//  				stabiliser.make_map(*vframe, DEFAULT_NUM_TRACKERS);
//  				if(!window.get3DStabActive().getValue() && !window.get2DStabActive().getValue()) {  //If either 3D or 2D stab is active, don't show map
//  					window.setShowingMap(true);
//  				}
//  			}
//  			else
//  				window.setMakingMap(false);
//  			madeMap = true;
//  		}
//  		else
//  			madeMap = false;
//  	}
//  	else
//  	{
//  		if(bSystemLogFlag) {logger->info("[System::whenMakeMapToggled] Making map toggled off");}

// 		stabiliser.invalidate(); //TODO: May need to memory manage?
// 	}
// 	frameProcessor.stabMutex.unlock();
// }

void System::whenStabiliseToggled(bool stabilising)
{
	frameProcessor.stabMutex.lock();
	while (!frameProcessor._stabComplete)
		frameProcessor.stabComplete.wait(frameProcessor.stabMutex);

	// Reset the raster so we start from (0,0) offset
	frameProcessor.resetRaster();

	if (stabilising)
	{
		if (bSystemLogFlag)
			logger->info("[System::whenStabiliseToggled] Stabilise toggled on");

		// If user wants the old stabiliser, we do nothing special here
		// If user wants the new phase correlation, ensure it is reset so it starts fresh
		if (usePhaseCorr)
		{
			phaseCorrStabiliser.reset();			   // clear reference frame
			phaseCorrStabiliserReferenceNotSet = true; // Important: Set this to true!
		}
	}
	else
	{
		if (bSystemLogFlag)
			logger->info("[System::whenStabiliseToggled] Stabilise toggled off");

		// Also reset the PhaseCorrStabiliser reference
		phaseCorrStabiliser.reset();
		phaseCorrStabiliserReferenceNotSet = true; // Important: Set this to true!
	}
	frameProcessor.stabMutex.unlock();
}

// void System::whenShowMapToggled(bool showingMap)
// {
// 	if (showingMap)
// 	{
// 		if(bSystemLogFlag) {logger->info("[System::whenShowMapToggled] Show map toggled on");}
// 		stabiliser.predraw();
// 		frameProcessor.filters["map"] = &stabiliser;
// 		SDLWindow::setShowingMap(childwin, true);
// 	}
// 	else
// 	{
// 		if(bSystemLogFlag) {logger->info("[System::whenShowMapToggled] Show map toggled off");}
// 		frameProcessor.filters.erase("map");
// 		SDLWindow::setShowingMap(childwin, false);
// 	}

// 	/*
// 	if (!window.getLiveView().getValue() && !window.getPlayingBuffer().getValue() )
// 	{
// 		VidFrame *vframe = recorder->getFrame(window.getFrameSliderValue() );
// 		if (vframe)
// 		{
// 			frameQueue.clear();
// 			frameQueue.push(vframe);
// 		}
// 	}
// 	*/
// }

void System::onResetClicked()
{
	if (bSystemLogFlag)
	{
		logger->info("[System::onResetClicked] Reset All button clicked - performing comprehensive system reset");
	}

	// 1. Stop all focus modes and clear global variables
	bFindFocus = false;
	bHoldFocus = false;

	// 2. Stop and clear ROI tracking and imaging camera monitoring
	if (getImagingCam())
	{
		getImagingCam()->stop(); // Stop all monitoring threads and focus search
		if (bSystemLogFlag)
		{
			logger->info("[System::onResetClicked] Stopped imaging camera monitoring and focus search");
		}
	}

	// Clear ROI display from SDL window
	clearROIDisplay();

	// 3. Reset stabilization and clear offsets
	frameProcessor.stabMutex.lock();
	while (!frameProcessor._stabComplete)
		frameProcessor.stabComplete.wait(frameProcessor.stabMutex);

	// Reset stabilization offsets
	frameProcessor.resetRaster();

	// Reset phase correlation stabilizer
	if (usePhaseCorr)
	{
		phaseCorrStabiliser.reset();
		phaseCorrStabiliserReferenceNotSet = true;
	}
	frameProcessor.stabMutex.unlock();

	// 4. Clear depth mapping data and reset SDL depth map
	currentDepthMap.isValid = false;
	currentDepthMap.depthImage.clear();
	currentDepthMap.width = 0;
	currentDepthMap.height = 0;

	// Clear SDL depth map data and force texture update
	if (childwin)
	{
		SDLWindow::clearDepthMapData(childwin);
	}

	// 5. Reset zoom and offsets in SDL window
	frameProcessor.resetZoom();

	// Hide out-of-bounds warning
	window.hideOutOfBoundsWarning();

	if (bSystemLogFlag)
	{
		logger->info("[System::onResetClicked] Reset All complete - all systems cleared and reset to defaults");
	}
}

void System::onRecenterClicked()
{
	if (bSystemLogFlag)
	{
		logger->info("[System::onRecenterClicked] Recenter clicked");
	}

	// Reset the zoom using the FrameProcessor method
	frameProcessor.resetZoom();

	// Recenter the rendered video frames
	frameProcessor.resetRaster();
}

void System::whenHoldFocusToggled(bool holdingFocus)
{
	// TODO: Fix this
	if (holdingFocus)
	{
		if (bSystemLogFlag)
		{
			logger->info("[System::whenHoldFocusToggled] Hold focus toggled on");
		}

		// CODE TO BE EXECUTED WHEN "HOLD FOCUS" IS ENABLED GOES HERE
		imgcount = 0;
		bHoldFocus = 1;
		usleep(100000); // sleep for 0.1 second
		window.setBestFocusScaleValue(desiredLocBestFocus);
	}
	else
	{
		if (bSystemLogFlag)
		{
			logger->info("[System::whenHoldFocusToggled] Hold focus toggled off");
		}

		// Stop ROI tracking and clear display
		if (getImagingCam())
		{
			getImagingCam()->stop(); // Stop the monitoring thread
		}

		bHoldFocus = 0;

		// Clear the ROI display from SDL window
		clearROIDisplay();
	}
}

void System::when3DStabToggled(bool active)
{
	if (active)
	{
		if (bSystemLogFlag)
		{
			logger->info("[System::when3DStabToggled] 3D stab toggled on");
		}
		// CODE TO BE EXECUTED WHEN "3D STAB" IS ENABLED GOES HERE
	}
	else
	{
		if (bSystemLogFlag)
		{
			logger->info("[System::when3DStabToggled] 3D stab toggled off");
		}
		// CODE TO BE EXECUTED WHEN "3D STAB" IS DISABLED GOES HERE
	}
}

void System::when2DStabToggled(bool active2)
{
	if (active2)
	{
		if (bSystemLogFlag)
		{
			logger->info("[System::when2DStabToggled] 2D stab toggled on");
		}
		// CODE TO BE EXECUTED WHEN "2D STAB" IS ENABLED GOES HERE
	}
	else
	{
		if (bSystemLogFlag)
		{
			logger->info("[System::when2DStabToggled] 2D stab toggled off");
		}
		// CODE TO BE EXECUTED WHEN "2D STAB" IS DISABLED GOES HERE
	}
}

void System::whenLoadingToggled(bool loading)
{
	if (loading)
		recorder->signalOperationLoad().emit(window.getFileLocation());
}

void System::whenSavingToggled(bool saving)
{
	if (saving)
	{
		window.displayMessageLoadSave("Saving...");
		recorder->signalOperationSave().emit(window.getFileLocation());
	}
}

void System::whenPlayingToggled(bool playing)
{
	if (playing)
	{
		int start = window.getFrameSliderValue();
		int frameRate = window.getFrameRateEntryBox();
		if (!recorder->isBuffering())
		{
			if (bSystemLogFlag)
			{
				logger->info("[System::whenPlayingToggled] Start playing from frame: {} at rate: {}", start, frameRate);
			}

			recorder->signalBuffer().emit(std::make_pair(start, frameRate));
		}
	}
	else
	{
		if (bSystemLogFlag)
		{
			logger->info("[System::whenPlayingToggled] Playing toggled off (probably by 'Pause')");
		}
		// TODO: Need to delete frames in frameQueue? Or manage in recorder?
		recorder->stopBuffering();
		// frameQueue.clear(); //This queue is the frames waiting to be rendered, NOT those in the recorder
	}
}

void System::whenSeekingToggled(bool seeking)
{
	if (seeking)
	{
		if (bSystemLogFlag)
		{
			logger->info("[System::whenSeekingToggled] Seeking toggled on");
		}

		if (window.getPlayingBuffer().getValue())
		{
			recorder->stopBuffering();
		}
		else if (!window.getLiveView().getValue())
		{
			VidFrame *out = recorder->getFrame(window.getFrameSliderValue());
			if (out)
				frameQueue.push(out); // frameQueue is the same as sysQueue in FrameObserver
			window.setSeeking(false);
		}
		else
			window.setSeeking(false);
	}
	else
	{
		if (bSystemLogFlag)
		{
			logger->info("[System::whenSeekingToggled] Seeking toggled off");
		}
		if (window.getPlayingBuffer().getValue())
			whenPlayingToggled(true);
	}
}

void System::whenRecordingToggled(bool recording)
{
	if (recording)
	{
		recorder->clearFrames(); // When a new recording is started, clear the previous frames
								 // frameQueue.clear(); //When a new recording is started, clear the previous frames
	}
}

// This runs when the recorder fills up
void System::onRecorderOperationComplete(RecOpRes res)
{
	OPRESEXPAND(res, op, success);

	switch (op)
	{
	case Recorder::Operation::RECOP_FILLED:
		if (bSystemLogFlag)
		{
			logger->info("[System::onRecorderOperationComplete] Recorder stopped recording");
		}
		if (success)
			window.displayMessageFPS("Recording complete. Remember to save!");
		else
			window.displayMessageFPS("Recording failed");
		window.setLiveView(!success);
		window.setHasBuffer(success);
		window.setRecording(false);
		window.setTrackingFPS(false);
		if (window.get3DStabActive().getValue())
		{
			window.set3DStab(false);
		}
		if (window.getHoldFocusActive().getValue())
		{
			window.setHoldFocus(false);
		}
		window.setFindFocus(false);
		break;
	case Recorder::Operation::RECOP_LOAD:
		if (window.getLoading().getValue())
		{
			if (success)
			{
				window.displayMessageLoadSave("Loading complete");
				window.displayMessageFPS("");
			}
			else
				window.displayMessageLoadSave("Loading failed");
			window.setLoading(false);
		}
		window.setHasBuffer(success);
		window.setLiveView(!success);
		break;

	case Recorder::Operation::RECOP_SAVE:
		window.setSaving(false);
		if (success)
			window.displayMessageLoadSave("Saving complete");
		else
			window.displayMessageLoadSave("Saving failed");
		break;

	case Recorder::Operation::RECOP_BUFFER:
		if (bSystemLogFlag)
		{
			logger->info("[System::onRecorderOperationComplete] Recorder stopped buffering");
		}
		window.setPlayingBuffer(false);
		break;

	case Recorder::Operation::RECOP_ADDFRAME:
		if (window.getLoading().getValue())
			window.displayMessageLoadSave(std::string("Loaded frame: ") + std::to_string(recorder->countFrames()));
		else if (window.getRecording().getValue())
			window.displayMessageFPS(std::string("Recorded frame: ") + std::to_string(recorder->countFrames()) + std::string(" of ") + std::to_string(static_cast<int>(window.getRecordingSizeScaleValue())));
		break;
	case Recorder::Operation::RECOP_EMPTIED:
		window.setHasBuffer(false);
		break;
	}
}

void System::onWindowFeatureUpdated(std::string fname, double val)
{
	setFeature(fname, val);
}

void System::onWindowThresholdChanged(double val)
{
	if (bSystemLogFlag)
	{
		logger->info("[System::onWindowThresholdChanged] Threshold changed to: {} recalculating map", val);
	}
	frameProcessor.stabMutex.lock();
	while (!frameProcessor._stabComplete)
		frameProcessor.stabComplete.wait(frameProcessor.stabMutex);

	stabiliser.adjust_thresh(val);
	if (stabiliser.is_valid())
	{
		stabiliser.predraw();
		/*
		if (!window.getLiveView().getValue() && !window.getPlayingBuffer().getValue() )
		{
			VidFrame *out = recorder->getFrame(window.getFrameSliderValue() );
			if (out)
			{
				frameQueue.clear();
				frameQueue.push(out);
			}
		}
		*/
	}

	frameProcessor.stabMutex.unlock();
}

void System::onWindowScaleChanged(double val)
{
	if (bSystemLogFlag)
	{
		logger->info("[System::onWindowScaleChanged] Scale changed to: {} recalculating map", val);
	}

	frameProcessor.stabMutex.lock();
	while (!frameProcessor._stabComplete)
		frameProcessor.stabComplete.wait(frameProcessor.stabMutex);

	stabiliser.adjust_scale(val);
	if (stabiliser.is_valid())
	{
		stabiliser.predraw();
		/*
		if (!window.getLiveView().getValue() && !window.getPlayingBuffer().getValue() )
		{
			VidFrame *out = recorder->getFrame(window.getFrameSliderValue() );
			if (out)
			{
				frameQueue.clear();
				frameQueue.push(out);
			}
		}
		*/
	}
	frameProcessor.stabMutex.unlock();
}

void System::onWindowBestFocusChanged(double val)
{
	// CODE TO EXECUTE WHEN BEST FOCUS SCALE CHANGED
	if (bSystemLogFlag)
	{
		logger->info("[System::onWindowBestFocusChanged] Best focus location changed to: {}, updating desiredLocBestFocus", val);
	}

	int val2 = round(val);
	// TODO: This should be in a mutex, and done much better. but it's not critical
	desiredLocBestFocus = val2;

	// Check if this is a manual user change (not from imaging camera callback)
	// If ROI search was complete and user manually changes focus, clear the ROI tracking
	if (childwin && !imagingCamUpdatingFocus)
	{
		pthread_mutex_lock(&childwin->mutex);
		bool wasSearchComplete = childwin->searchComplete;
		bool hasActiveROI = childwin->showROI;
		pthread_mutex_unlock(&childwin->mutex);

		// If there was a completed ROI search and user manually changed focus, clear it
		if (wasSearchComplete && hasActiveROI)
		{
			if (bSystemLogFlag)
			{
				logger->info("[System::onWindowBestFocusChanged] User manually changed focus after ROI search - clearing ROI tracking");
			}

			// Clear the ROI display and stop monitoring
			clearROIDisplay();
		}
	}
}

void System::onWindowPauseClicked()
{
}

void System::onWindowEnterClicked()
{
	double newFrameRate = window.getFrameRateEntryBox();
	if (0.0 < newFrameRate && newFrameRate < 144.0)
	{
		if (window.getLiveView().getValue()) // if Live, must restart camera
		{
			stopStreaming();
			usleep(500000); // sleep for 0.5s
			std::cout << "about to start streaming" << std::endl;
			if (!startStreaming())
			{
				std::cout << "failed to set FPS" << std::endl;
				window.displayMessageError("Failed to Set FPS");
			}
			else
			{
				window.displayMessageError("");
			}
		}
		if (recorder->isBuffering()) // if Buffering, must restart buffering at new frame rate
		{
			recorder->stopBuffering();
			usleep(100000); // sleep for 0.1s
			recorder->signalBuffer().emit(std::make_pair(window.getFrameSliderValue(), newFrameRate));
			window.setPlayingBuffer(true);
		}
		// If neither buffering nor viewing live (i.e. paused recording), do nothing as new frame rate  will be picked up when 'play' clicked
		// TODO: Deal with frameRate FAILED to set
	}
	else
	{
		window.displayMessageFPS("FPS Out-of-range");
	}

	if (bSystemLogFlag)
	{
		logger->info("[System::onEnterClicked] Enter key pressed");
	}
}

double System::getFPS()
{
	return window.getFrameRateEntryBox();
}

bool System::onCloseClicked(const GdkEventAny *event)
{
	// CODE TO EXECUTE WHEN CLOSE BUTTON CLICKED
	if (bSystemLogFlag)
	{
		logger->info("[System::onCloseClicked] Close button clicked");
	}
	// Close the streaming camera
	stopStreaming();
	// vsys.Shutdown(); //TODO: Maybe move to ~System()?
	// delete priv; //TODO: Maybe move to ~System()?
	return false;
}

System::~System()
{
	// m_conn.disconnect();

	if (frameQueue.size() > 0)
	{
		for (VidFrame *vframe = frameQueue.pop(); !frameQueue.empty(); vframe = frameQueue.pop())
			delete vframe;
		frameQueue.clear();
	}

	vsys.Shutdown();
	delete priv;
	priv = nullptr; // Set to nullptr after deleting to avoid dangling pointer
	if (bSystemLogFlag)
	{
		logger->info("[System::~System] destructor ran");
	}

	// Stop imaging camera before other cleanup
	if (getImagingCam())
	{
		getImagingCam()->stop();
		imagingCam.reset();
	}
}

MainWindow &System::getWindow()
{
	return window;
}

Recorder &System::getRecorder()
{
	return *recorder;
}

void System::handleDisconnection()
{
	std::cout << "Handling disconnection, tiltedCamDisconnected: " << tiltedCamDisconnected << ", imagingCamDisconnected: " << imagingCamDisconnected << ", lensDisconnected: " << lensDisconnected << std::endl;
	if (tiltedCamDisconnected && imagingCamDisconnected && lensDisconnected)
	{
		std::cout << "All devices are disconnected!" << std::endl;
		window.displayMessageError("Imaging Camera, Tilted Camera, and Lens are Disconnected");
	}
	else if (tiltedCamDisconnected && imagingCamDisconnected)
	{
		std::cout << "Tilted Camera and Imaging Camera are Disconnected!" << std::endl;
		window.displayMessageError("Imaging Camera and Tilted Camera are Disconnected");
	}
	else if (tiltedCamDisconnected && lensDisconnected)
	{
		std::cout << "TiltedCam and Lens are disconnected!" << std::endl;
		window.displayMessageError("Tilted Camera and Lens are Disconnected");
	}
	else if (imagingCamDisconnected && lensDisconnected)
	{
		std::cout << "ImagingCam and Lens are disconnected!" << std::endl;
		window.displayMessageError("Imaging Camera and Lens are Disconnected");
	}
	else if (imagingCamDisconnected)
	{
		std::cout << "ImagingCam is disconnected!" << std::endl;
		window.displayMessageError("Imaging Camera is Disconnected");
	}
	else if (lensDisconnected)
	{
		std::cout << "Lens is disconnected!" << std::endl;
		window.displayMessageError("Lens is Disconnected");
	}
	else if (tiltedCamDisconnected)
	{
		std::cout << "TiltedCam is disconnected!" << std::endl;
		window.displayMessageError("Tilted Camera is Disconnected");
	}
}

void System::onWindowHomePositionChanged(double val)
{
	if (bSystemLogFlag)
	{
		logger->info("[System::onWindowHomePositionChanged] Lens home position changed to: {}mm", val);
	}
	// Update the lens return position
	AF.getLens().setReturnPosition(val);
}

void System::onWindowPGainChanged(double val)
{
	if (bSystemLogFlag)
	{
		logger->info("[System::onWindowPGainChanged] P gain changed to: {}", val);
	}
	// Update the autofocus P gain
	AF.setPGain(val);
}

void System::updateSharpnessGraph()
{
	window.updateSharpnessGraph(currentSharpnessValues);
}

void System::onGetDepthsClicked()
{
	if (bSystemLogFlag)
	{
		logger->info("[System::onGetDepthsClicked] Get Depths toggle clicked");
	}

	// Check toggle state to determine action
	bool gettingDepths = window.getGetDepthsActive().getValue();

	if (gettingDepths)
	{
		// Toggle ON - Start depth mapping
		if (bSystemLogFlag)
		{
			logger->info("[System::onGetDepthsClicked] Starting depth mapping");
		}

		// Only run if viewing live feed
		if (!window.getLiveView().getValue())
		{
			window.displayMessageError("Depth mapping only available in live view mode");
			window.getGetDepthsActive().setValue(false); // Turn off toggle if invalid
			return;
		}

		// Step 1: Clear stabilization and reset offsets
		if (bSystemLogFlag)
		{
			logger->info("[System::onGetDepthsClicked] Clearing stabilization and resetting offsets");
		}

		// Turn off stabilization to reset offsets
		window.setStabiliseActive(false);

		// Reset the stabilization offsets in frame processor
		frameProcessor.resetRaster();

		// Small delay to ensure stabilization is properly reset
		std::this_thread::sleep_for(std::chrono::milliseconds(50));

		// Turn stabilization back on with fresh start
		window.setStabiliseActive(true);

		// Step 2: Clear any existing depth map data
		if (bSystemLogFlag)
		{
			logger->info("[System::onGetDepthsClicked] Clearing existing depth map data");
		}

		// Clear system depth map data
		currentDepthMap.isValid = false;
		currentDepthMap.depthImage.clear();
		currentDepthMap.width = 0;
		currentDepthMap.height = 0;

		// Clear SDL depth map data
		if (childwin)
		{
			SDLWindow::clearDepthMapData(childwin);
		}

		// Turn off 'View Depths' toggle
		window.setViewDepths(false);

		// Step 3: Enable Hold Focus
		if (bSystemLogFlag)
		{
			logger->info("[System::onGetDepthsClicked] Enabling Hold Focus");
		}

		// Enable Hold Focus mode
		bHoldFocus = true;
		window.setHoldFocus(true);

		// Start the depth mapping process
		if (getImagingCam())
		{
			getImagingCam()->startDepthMapping();
		}
	}
	else
	{
		// Toggle OFF - Clear depth mapping
		if (bSystemLogFlag)
		{
			logger->info("[System::onGetDepthsClicked] Clearing depth mapping");
		}

		// Clear system depth map data
		currentDepthMap.isValid = false;
		currentDepthMap.depthImage.clear();
		currentDepthMap.width = 0;
		currentDepthMap.height = 0;

		// Clear SDL depth map data and force texture update
		if (childwin)
		{
			SDLWindow::clearDepthMapData(childwin);
		}

		if (bSystemLogFlag)
		{
			logger->info("[System::onGetDepthsClicked] Depth mapping cleared");
		}
	}
}

void System::whenGetDepthsToggled(bool gettingDepths)
{
	// This method is called when the toggle state changes
	// The actual logic is handled in onGetDepthsClicked()
	if (bSystemLogFlag)
	{
		logger->info("[System::whenGetDepthsToggled] Get Depths toggle state changed to: {}", gettingDepths);
	}
}

void System::whenViewDepthsToggled(bool viewingDepths)
{
	if (bSystemLogFlag)
	{
		logger->info("[System::whenViewDepthsToggled] View Depths toggled: {}", viewingDepths);
	}

	// Update SDL window to show/hide depth map overlay
	if (childwin)
	{
		pthread_mutex_lock(&childwin->mutex);
		childwin->showDepthMap = viewingDepths;

		pthread_mutex_unlock(&childwin->mutex);
	}
}

void System::updateROICenter(int x, int y)
{
	// Only update ROI if we're in live view mode
	if (!window.getLiveView().getValue())
	{
		if (bSystemLogFlag)
		{
			logger->info("[System::updateROICenter] ROI update ignored - not in live view mode");
		}
		return;
	}

	// Start ROI tracking if imagingCam exists
	if (getImagingCam())
	{
		getImagingCam()->setROICenter(x, y);
		getImagingCam()->start(); // Start the monitoring thread
	}

	// Update SDL window display parameters with current ROI info from imaging camera
	if (childwin)
	{
		pthread_mutex_lock(&childwin->mutex);
		childwin->roiCenterX = x;
		childwin->roiCenterY = y;
		childwin->showROI = true;
		childwin->searchComplete = false; // Reset search complete state

		// Get the current ROI size from imaging camera to ensure synchronization
		if (getImagingCam())
		{
			int centerX, centerY, width, height;
			getImagingCam()->getCurrentROI(centerX, centerY, width, height);
			childwin->roiWidth = width;
			childwin->roiHeight = height;
		}

		pthread_mutex_unlock(&childwin->mutex);
	}
}

void System::clearROIDisplay()
{
	// Stop ROI tracking first to ensure threads are stopped
	if (getImagingCam())
	{
		getImagingCam()->stop(); // Stop the monitoring thread and any active focus search

		if (bSystemLogFlag)
		{
			logger->info("[System::clearROIDisplay] Stopped imaging camera monitoring and focus search threads");
		}
	}

	// Clear the SDL window ROI display
	if (childwin)
	{
		pthread_mutex_lock(&childwin->mutex);
		childwin->showROI = false;
		childwin->roiCenterX = -1;
		childwin->roiCenterY = -1;
		childwin->searchComplete = false; // Reset search complete state
		pthread_mutex_unlock(&childwin->mutex);

		if (bSystemLogFlag)
		{
			logger->info("[System::clearROIDisplay] ROI display cleared from SDL window");
		}
	}
}

void System::getCurrentROI(int &centerX, int &centerY, int &width, int &height) const
{
	if (imagingCam)
	{
		imagingCam->getCurrentROI(centerX, centerY, width, height);
	}
	else
	{
		centerX = centerY = -1;
		width = height = 150;
	}

	// Also update SDL window with current ROI info
	if (childwin && centerX >= 0 && centerY >= 0)
	{
		pthread_mutex_lock(&childwin->mutex);
		childwin->roiCenterX = centerX;
		childwin->roiCenterY = centerY;
		childwin->roiWidth = width;
		childwin->roiHeight = height;
		childwin->showROI = true;
		childwin->searchComplete = false;
		pthread_mutex_unlock(&childwin->mutex);
	}
}

void System::setSDLWindow(SDLWindow::SDLWin *win)
{
	if (win)
	{
		win->system = this;
		win->roiWidth = 150; // Default ROI size
		win->roiHeight = 150;
		win->showROI = false;
		win->roiCenterX = -1;
		win->roiCenterY = -1;
		win->showDepthMap = false;
		win->hasDepthMap = false;
		win->depthMapReady = false;
		win->depthMapWidth = 0;
		win->depthMapHeight = 0;
		for (int y = 0; y < SDLWindow::SDLWin::MAX_DEPTH_HEIGHT; y++)
		{
			for (int x = 0; x < SDLWindow::SDLWin::MAX_DEPTH_WIDTH; x++)
			{
				win->focusPositions[y][x] = -1.0f; // Negative indicates no data
			}
		}

		if (bSystemLogFlag)
		{
			logger->info("[System::setSDLWindow] SDL window connected to system");
		}
	}
}

bool System::getStabilizationOffset(double &offsetX, double &offsetY)
{
	bool stabilising = window.getStabiliseActive().getValue();
	if (stabilising)
	{
		// Use try_lock to avoid deadlock if stabMutex is already held by current thread
		if (frameProcessor.stabMutex.trylock())
		{
			offsetX = frameProcessor.rasterPos[0];
			offsetY = frameProcessor.rasterPos[1];
			frameProcessor.stabMutex.unlock();
			return true;
		}
		else
		{
			// If we can't get the lock, use the last known values from SDL shared memory
			// This avoids deadlock when called from within stabilization thread
			if (childwin)
			{
				if (pthread_mutex_trylock(&childwin->mutex) == 0)
				{
					offsetX = childwin->stabOffsetX;
					offsetY = childwin->stabOffsetY;
					pthread_mutex_unlock(&childwin->mutex);
					return true;
				}
			}
			// Fallback to no offset if all locks fail
			offsetX = 0.0;
			offsetY = 0.0;
			return false;
		}
	}
	else
	{
		offsetX = 0.0;
		offsetY = 0.0;
		return false;
	}
}