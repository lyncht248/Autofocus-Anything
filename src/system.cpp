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
											  released()
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

	// This counter will be used to skip the phase-correlation calculation
	// when the user-defined FPS is above 40.
	static int frameCount = 0;

	while (running)
	{
		// If the queue is backing up, discard older frames and only keep the most recent one or two
		// so we can keep up at higher framerates.
		static const size_t MAX_ALLOWED_IN_QUEUE = 2;
		while (stabQueue.size() > MAX_ALLOWED_IN_QUEUE)
		{
			VidFrame *oldFrame = stabQueue.pop();
			delete oldFrame; // or recycle if you have a different memory scheme
		}

		VidFrame *vframe = stabQueue.pop();

		// If the frame is null, skip it
		if (vframe == nullptr)
			continue;

		// Check if we should skip this frame's offset calculation
		bool skipOffsetCalculation = false;
		if (system.getFPS() > 40.0)
		{
			skipOffsetCalculation = ((frameCount % 2) != 0); // Skip every other frame
			frameCount++;
		}

		// If the user wants PhaseCorr, do that
		if (system.usePhaseCorr)
		{
			if (!skipOffsetCalculation)
			{
				// Convert VidFrame -> cv::Mat
				// vframe->data() is pointer to image data
				// vframe->size().x is width, vframe->size().y is height
				cv::Mat fullFrame(
					vframe->size().y,
					vframe->size().x,
					CV_8UC1,	   // 8-bit single channel
					vframe->data() // pointer to raw pixels
				);

				// Convert single-channel grayscale to 3-channel BGR (phaseCorrStabiliser uses COLOR_BGR2GRAY)
				// If your camera is truly monochrome, you can adapt this to keep single channel as well.
				cv::Mat colorFrame;
				cv::cvtColor(fullFrame, colorFrame, cv::COLOR_GRAY2BGR);

				// Downscale by factor 0.5 before calling phase correlation
				cv::Mat halfFrame;
				cv::resize(colorFrame, halfFrame, cv::Size(), 0.26, 0.26); // for some reason 0.25 causes problems

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
				// Use shift to update currentOff. Multiply by factor 2 if needed
				// (Since we used 0.26 scale, shift is 1/0.26 = 3.8461538461538462)
				currentOff[0] = shift.x * -3.8461538461538462f;
				currentOff[1] = shift.y * -3.8461538461538462f;

				offset += currentOff;
				_stabComplete = stabQueue.empty();
				if (_stabComplete)
					stabComplete.broadcast();
				stabMutex.unlock();
			}
			else
			{
				// If skipping, still set _stabComplete when queue is empty
				stabMutex.lock();
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
					TooN::Vector<2> off = system.stabiliser.stabilise(*vframe, offset);
					stabMutex.lock();
					currentOff = off;
					offset += currentOff;
					_stabComplete = stabQueue.empty();
					if (_stabComplete)
						stabComplete.broadcast();
					stabMutex.unlock();
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

			rasterPos[0] -= currentOff[0];
			rasterPos[1] -= currentOff[1];
			currentOff = TooN::Zeros;
			stabMutex.unlock();
			SDLWindow::setRaster(childwin, rasterPos[0], rasterPos[1]);
		}
		else
		{
			SDLWindow::setRaster(childwin);
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
	SDLWindow::setRaster(childwin);
	if (bSystemLogFlag)
	{
		logger->info("[FrameProcessor::resetRaster] Stabilise offset reset to 0");
	}
}

void FrameProcessor::resetZoom()
{
	// Access the childwin member which is private to this class
	if (childwin) {
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
										recorder(thread.createObject<Recorder, System &>(*this)), // Creates a recorder object in a separate thread, passing it a thread pointer
										sRecorderOperationComplete(),
										stabiliser(),
										madeMap(false),
										AF() // Creates an autofocus object
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

	AF.initialize(); // Initialises the autofocus object. This was the constructor, but need to be able to catch errors

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

	window.getHoldFocusActive().signalToggled().connect(sigc::mem_fun(*this, &System::whenHoldFocusToggled));
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
		logger->info("[System::onResetClicked] Lens reset button clicked");
	}
	// Hide the out of bounds warning when reset is clicked
	window.hideOutOfBoundsWarning();
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
		// CODE TO BE EXECUTED WHEN "HOLD FOCUS" IS DISABLED GOES HERE
		bHoldFocus = 0;
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
		// make recorder toggle insensitive

		// window.getShowMapActive().setValue(!success);
		// window.getStabiliseActive().setValue(!success);
		// window.getMakeMapActive().setValue(!success);
		// window.getHoldFocusActive().setValue(!success);
		// window.get2DStabActive().setValue(!success);
		// window.get3DStabActive().setValue(!success);
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