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

bool bSystemLogFlag = 1; // 1 = log, 0 = no log
bool bSystemQueueLengthFlag = 1; // 1 = log, 0 = no log

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
FrameProcessor::FrameProcessor(System &sys) :
	processed(nullptr),
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
	processorThread = Glib::Threads::Thread::create(sigc::mem_fun(*this, &FrameProcessor::processFrame) );
	stabThread = Glib::Threads::Thread::create(sigc::mem_fun(*this, &FrameProcessor::stabilise) );
	if(bSystemLogFlag) {logger->info("[FrameProcessor::FrameProcessor] constructor finished");}
}

FrameProcessor::~FrameProcessor()
{
	running = false;
	ThreadStopper::stop({processorThread, stabThread});

	if(bSystemLogFlag) {logger->info("[FrameProcessor::~FrameProcessor] destructor called, processorThread and stabThread stopped");}
}

void FrameProcessor::stabilise()
{
	if(bSystemLogFlag) {logger->info("[FrameProcessor::stabilise] Thread started");}

	ThreadStopper::makeStoppable();
	while (running)
	{
		VidFrame *vframe = stabQueue.pop();
		//stabMutex.lock();
		//_stabNewFrame = false;
		//stabMutex.unlock();

		if (system.stabiliser.is_valid() )
		{
			TooN::Vector<2> off = system.stabiliser.stabilise(*vframe, offset);
			//TODO vframe seems to point to the same image regardless of whether its a loaded recording or a live view...
			// but when its a loaded recording, offset and off become NULL. Why? 
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

		released.push(vframe);
	}
}

void FrameProcessor::processFrame() //What to do with each frame received from FrameObserver
{
	if(bSystemLogFlag) {logger->info("[FrameProcessor::processFrame] Thread started");}

	ThreadStopper::makeStoppable();
	while (running)
	{
		// Get the frame when it's ready
		ThreadStopper::lock(mutex);
		vidFrame = system.getFrameQueue().pop(); // Takes frame from top of queue
		_frameReleased = false;
		ThreadStopper::unlock(mutex);

		// If stabilising, put frame in Stabiliser queue
		bool stabilising = system.window.getStabiliseActive().getValue();
		if (stabilising)
		{
			stabMutex.lock();
			_stabComplete = false;
			//_stabNewFrame = true;
			//stabNewFrame.signal();
			stabMutex.unlock();
			stabQueue.push(vidFrame);
		}

		// recording = 1 if incoming frames need to be added to the recorder
		bool recording = system.window.getRecording().getValue() && !system.window.getPausedRecording().getValue();

		// If recording, sends the frame to the recorder for saving
		if (recording)
		{
			system.getRecorder().putFrame(vidFrame);
		}

		// Draw the frame at the size neccessary to fit the window
		CVD::ImageRef vfDim = vidFrame->size();
		if (vfDim.x != childwin->raster.w || vfDim.y != childwin->raster.h)
		{
			//processed = Cairo::ImageSurface::create(Cairo::FORMAT_RGB24, vfDim.x, vfDim.y);
			SDLWindow::createFrame(childwin, vfDim.x, vfDim.y);
		}
		
		//convertToCairo(stabilising);

		// deprecated code (I believe) TODO remove
		for (auto pair : filters)
		{
			pair.second->draw(processed);
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
					if (!stabComplete.wait_until(stabMutex, end_time) )
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
			SDLWindow::setRaster(childwin);

		system.signalNewFrame().emit();

		// Now that the current vframe is processed, release it
		ThreadStopper::lock(mutex);
		{
			// Signal to release frame from thread and then wait for frame to be released
			while (!_frameReleased)
			{
				ThreadStopper::stopPoint(&frameReleased);
				frameReleased.wait(mutex);
			}

			// If not stabilising, add frame to released queue. If you are stabilising, it should already have been added
			if (!stabilising)
			{
				released.push(vidFrame);
			}

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
			if (recording)
			{
				// If recording, then don't delete the frame, but clear the released queue. These frames can be deleted when the recording is saved.
				released.clear();
				if(bSystemLogFlag) {logger->info("[FrameProcessor::processFrame] released queue cleared, but NOT deleted!!!");}
			}
			else 
			{
				// Not recording, so can delete the released frames.
				for (VidFrame *vframe = released.pop(); !released.empty(); vframe = released.pop() ) 
					delete vframe;
				vidFrame = nullptr;
				released.clear();
			}
		}
		ThreadStopper::unlock(mutex);

		// stabQueue.size() = 0 almost always
		if(bSystemQueueLengthFlag) {logger->info("[FrameProcessor::processFrame] stabQueue is length: {}", stabQueue.size());}

		// released.size() = 0 almost always
		if(bSystemQueueLengthFlag) {logger->info("[FrameProcessor::processFrame] released queue is length: {}", released.size());}

		if(bSystemQueueLengthFlag) {logger->info("[FrameProcessor::processFrame] frameQueue is length: {}", system.getFrameQueue().size());}
	}
}

void FrameProcessor::releaseFrame() //TODO: Should here be a delete frame command?
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
	if(bSystemLogFlag) {logger->info("[FrameProcessor::resetRaster] Stabilise offset reset to 0");}
}

::Cairo::RefPtr< ::Cairo::Surface> FrameProcessor::getFrame()
{
	return processed;
}

// This class is used to receive frames from the camera, but passes frames immediately to FrameProcessor
class FrameObserver : public Glib::Object, virtual public Vimba::IFrameObserver
{
public :
	FrameObserver(Vimba::CameraPtr pCamera, TSQueue<VidFrame *> &frameQueue) :  IFrameObserver ( pCamera ), Object(),
		sigReceived(),
		sysFrames(frameQueue)
	{
	}
	// ~FrameObserver()
	// {
	// 	for (VidFrame *vframe = sysFrames.pop(); !sysFrames.empty(); vframe = sysFrames.pop())
	// 		delete vframe;
	// 	sysFrames.clear();
	// }
	
	void FrameReceived ( const Vimba::FramePtr pFrame )
	{
		VmbFrameStatusType eReceiveStatus;
		if( VmbErrorSuccess == pFrame -> GetReceiveStatus ( eReceiveStatus ) )
		{
			if (VmbFrameStatusComplete == eReceiveStatus)
			{

				VmbUint32_t sz, width, height; 
				pFrame->GetImageSize(sz); //TODO: These should be called once, not every frame
				pFrame->GetWidth(width);
				pFrame->GetHeight(height);

				IVidFrame *sysFrame = new IVidFrame(CVD::ImageRef(width, height) ); //Each of these must get deleted when processed by FrameProcessor

				//sysFrame->resize(CVD::ImageRef(width, height) );

				//pFrame->GetOffsetX(sysFrame.xoff);
				//pFrame->GetOffsetY(sysFrame.yoff);

				VmbUchar_t *buf = nullptr;
				pFrame->GetImage(buf);
				memcpy(sysFrame->data(), buf, sz);
				sysFrames.push(sysFrame);

				// Here is where we check the queue size and drop a frame if needed. TODO: just change in frameQueue in System::System
				if (sysFrames.size() >= 15)
				{
					VidFrame* frame_to_delete = sysFrames.pop(); // remove the frame from the queue

					if(bSystemQueueLengthFlag) {logger->info("[FrameObserver::FrameReceived] Popped a frame (should auto-delete) from the sysFrames queue as it was too long.");}
				}
				
				// Quickly recieves 15 frames which are then emptied by FrameProcessor quickly
				if(bSystemQueueLengthFlag) {logger->info("[FrameObserver::FrameReceived] sysFrames queue is length: {}", sysFrames.size());}

				sigReceived.emit();
			}
		}
		m_pCamera -> QueueFrame ( pFrame );
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

System::Private::Private() :
	observer(),
	frames(3)
{
}

System::System(int argc, char **argv) :
	window(), 
	vsys(Vimba::VimbaSystem::GetInstance() ), 
	cam(),
	size(),
	im(),
	priv(new Private() ),
	sigNewFrame(hvigtk_threadcontext() ),
	frameQueue(100),
	frameProcessor(*this),
	thread(),
	recorder(thread.createObject<Recorder, System&>(*this) ), //Creates a recorder object in a separate thread, passing it a thread pointer
	sRecorderOperationComplete(),
	stabiliser(),
	madeMap(false),
	AF() //Creates an autofocus object
{
	if(bSystemLogFlag) {logger->info("[System::System] constructor beginning");}

	//Error message handling!
	NotificationCenter::instance().registerListener("error", [this]() {
		on_error();
	});

	recorder->connectTo(&sRecorderOperationComplete);
	sRecorderOperationComplete.connect(sigc::mem_fun(*this, &System::onRecorderOperationComplete) );

    window.signal_map_event().connect([this](GdkEventAny* /*event*/) -> bool {
        window.move(gtkAppLocationX, gtkAppLocationY);  // Replace with your desired coordinates.
        return false;  // Continue propagation.
    });

	window.signalFrameDrawn().connect(sigc::mem_fun(*this, &System::releaseFrame) );
	window.signalFeatureUpdated().connect(sigc::mem_fun(*this, &System::onWindowFeatureUpdated) );
	window.signalThresholdChanged().connect(sigc::mem_fun(*this, &System::onWindowThresholdChanged) );
	window.signalScaleChanged().connect(sigc::mem_fun(*this, &System::onWindowScaleChanged) );
	window.signalBestFocusChanged().connect(sigc::mem_fun(*this, &System::onWindowBestFocusChanged) );
	window.signalPauseClicked().connect(sigc::mem_fun(*this, &System::onWindowPauseClicked) );
	sigNewFrame.connect(sigc::mem_fun(*this, &System::renderFrame) );

	//CONDITIONS
	
	window.getLiveView().signalToggled().connect(sigc::mem_fun(*this, &System::whenLiveViewToggled) );

	window.getMakeMapActive().signalToggled().connect(sigc::mem_fun(*this, &System::whenMakeMapToggled) );
	window.getStabiliseActive().signalToggled().connect(sigc::mem_fun(*this, &System::whenStabiliseToggled) );
	window.getShowMapActive().signalToggled().connect(sigc::mem_fun(*this, &System::whenShowMapToggled) );

	window.signalFindFocusClicked().connect(sigc::mem_fun(*this, &System::onFindFocusClicked));
	window.signalResetClicked().connect(sigc::mem_fun(*this, &System::onResetClicked));

	window.getHoldFocusActive().signalToggled().connect(sigc::mem_fun(*this, &System::whenHoldFocusToggled) );
	window.get3DStabActive().signalToggled().connect(sigc::mem_fun(*this, &System::when3DStabToggled) );
	window.get2DStabActive().signalToggled().connect(sigc::mem_fun(*this, &System::when2DStabToggled) );

	window.getLoading().signalToggled().connect(sigc::mem_fun(*this, &System::whenLoadingToggled) );
	window.getSaving().signalToggled().connect(sigc::mem_fun(*this, &System::whenSavingToggled) );
	window.getPlayingBuffer().signalToggled().connect(sigc::mem_fun(*this, &System::whenPlayingToggled) );
	window.getSeeking().signalToggled().connect(sigc::mem_fun(*this, &System::whenSeekingToggled) );
	window.getRecording().signalToggled().connect(sigc::mem_fun(*this, &System::whenRecordingToggled) );

	//Signals that the window has been closed, and other threads must close
	window.signal_delete_event().connect(sigc::mem_fun(*this, &System::onCloseClicked));

	auto now = std::chrono::system_clock::now();
	std::time_t time = std::chrono::system_clock::to_time_t(now);
	
 	char buf[64] = { 0 };
    std::strftime(buf, sizeof(buf), "%F %R %Z", std::localtime(&time) );

	if (vsys.Startup() == VmbErrorSuccess)
	{
		if(bSystemLogFlag) {logger->info("[System::System] Starting up Vimba: success");}
		Vimba::CameraPtrVector cameras;

		if (vsys.GetCameras(cameras) == VmbErrorSuccess)
		{
			if(bSystemLogFlag) {logger->info("[System::System] Found: {} cameras", cameras.size());}
			if (!cameras.empty() )
			{
				cam = cameras[0];
				/*
				cam->GetFeatureByName("PixelFormat", pFeature);
				pFeature->SetValue(VmbPixelFormatMono12);
				*/
			}
		}
		else
		{
			if(bSystemLogFlag) {logger->info("[System::System] Failed to find cameras");}
		}
	}
	else
	{
		logger->error("[System::System] Failed to start Vimba");
	}
	whenLiveViewToggled(true); // calls startStreaming
}

void System::startStreaming()
{
	if (cam != nullptr)
	{
		if (cam->Open(VmbAccessModeFull) == VmbErrorSuccess)
		{
			if(bSystemLogFlag) {logger->info("[System::startStreaming] Successfully opened camera");}
			
			setFeature("PixelFormat", VmbPixelFormatMono8);
			window.updateCameraValues(getFeature<double>("Gain"), getFeature<double>("ExposureTime"), getFeature<double>("Gamma") );
			setFeature("AcquisitionFrameRateEnable", true);
			setFeature<double>("AcquisitionFrameRate", window.getFrameRateScaleValue() );

			Vimba::FeaturePtr pFeature;
			cam->GetFeatureByName("PayloadSize", pFeature);
			VmbInt64_t nPLS;
			pFeature->GetValue(nPLS);
			
			priv->observer.reset(new FrameObserver(cam, frameQueue) ); //reset means its a smart pointer and will delete itself when it goes out of scope

			for (Vimba::FramePtrVector::iterator iter = priv->frames.begin(); iter != priv->frames.end(); ++iter) //iter is the frame number
			{
				(*iter).reset( new Vimba::Frame ( nPLS ) ); //create a frame of the correct size
				(*iter)->RegisterObserver(priv->observer); //register the observer
				cam->AnnounceFrame(*iter); //announce the frame to the camera
			}

			if (cam->StartCapture() == VmbErrorSuccess)
			{
				for (Vimba::FramePtrVector::iterator iter = priv->frames.begin(); iter != priv->frames.end(); ++iter)
				{
					cam->QueueFrame(*iter);
				}
				
				if (cam->GetFeatureByName("AcquisitionStart", pFeature) == VmbErrorSuccess && pFeature->RunCommand() == VmbErrorSuccess)
				{
					if(bSystemLogFlag) {logger->info("[System::startStreaming] Started frame capture");}
				}
				else
				{
					goto fail;
				}
			}
			else
			{
			fail:;
				if(bSystemLogFlag) {logger->info("[System::startStreaming] Failed to start frame capture");} //TODO might need to mem manage below here
			}
		}
	}
}

void System::stopStreaming()
{
	if (cam != nullptr)
	{
		Vimba::FeaturePtr pFeature;

		if (cam->GetFeatureByName ("AcquisitionStop", pFeature ) == VmbErrorSuccess && pFeature->RunCommand() == VmbErrorSuccess)
		{
			if(bSystemLogFlag) {logger->info("[System::stopStreaming] Stopped acquisition");}
		}

		if (cam->EndCapture() == VmbErrorSuccess)
		{
			if(bSystemLogFlag) {logger->info("[System::stopStreaming] Stopped capture");}
		}

		if (cam->FlushQueue() == VmbErrorSuccess)
		{
			if(bSystemLogFlag) {logger->info("[System::stopStreaming] Flushed queue");}
		}

		for (Vimba::FramePtrVector::iterator iter = priv->frames.begin(); iter != priv->frames.end(); ++iter)
		{
			(*iter)->UnregisterObserver();
		}
		
		if (cam->RevokeAllFrames() == VmbErrorSuccess)
		{
			if(bSystemLogFlag) {logger->info("[System::stopStreaming] Revoked frames");}
		}

		if (cam->Close() == VmbErrorSuccess)
		{
			if(bSystemLogFlag) {logger->info("[System::stopStreaming] Closed camera");}
		}
	}
}

TSQueue<VidFrame*>& System::getFrameQueue()
{
	return frameQueue;
}

Glib::Dispatcher& System::signalNewFrame()
{
	return sigNewFrame;
}

VidFrame* System::getFrame()
{
	return frameProcessor.vidFrame;
}

void System::renderFrame()
{
	window.renderFrame(frameProcessor.vidFrame);
}

void System::releaseFrame()
{
	if (window.getMakeMapActive().getValue() && !madeMap)
	{
		stabiliser.make_map(*getFrame(), DEFAULT_NUM_TRACKERS);
		if(!window.get3DStabActive().getValue() && !window.get2DStabActive().getValue()) { //If either 3D or 2D stab is active, don't show map
			window.setShowingMap(true);
		}
		madeMap = true;
	}
	frameProcessor.releaseFrame();
}

void System::whenLiveViewToggled(bool viewingLive)
{
	window.setMakingMap(false);
	if (viewingLive)
	{
		if(bSystemLogFlag) {logger->info("[System::whenLiveViewToggled] Live view toggled on");}
		startStreaming();
	}
	else
	{
		if(bSystemLogFlag) {logger->info("[System::whenLiveViewToggled] Live view toggled off");}
		stopStreaming();
	}
}

void System::whenMakeMapToggled(bool makingMap)
{
	frameProcessor.stabMutex.lock();
	while (!frameProcessor._stabComplete)
		frameProcessor.stabComplete.wait(frameProcessor.stabMutex);
	if (makingMap)
	{
		if(bSystemLogFlag) {logger->info("[System::whenMakeMapToggled] Making map toggled on");}
		frameProcessor.resetRaster();
		if (!window.getLiveView().getValue() )
		{
			VidFrame *vframe = recorder->getFrame(window.getFrameSliderValue() );
			if (vframe)
			{
				stabiliser.make_map(*vframe, DEFAULT_NUM_TRACKERS);
				if(!window.get3DStabActive().getValue() && !window.get2DStabActive().getValue()) {  //If either 3D or 2D stab is active, don't show map
					window.setShowingMap(true);	
				}
			}
			else
				window.setMakingMap(false);
			madeMap = true;
		}
		else
			madeMap = false;
	}
	else 
	{
		if(bSystemLogFlag) {logger->info("[System::whenMakeMapToggled] Making map toggled off");}
	
		stabiliser.invalidate(); //TODO: May need to memory manage?
	}
	frameProcessor.stabMutex.unlock();
}

void System::whenStabiliseToggled(bool stabilising)
{
	frameProcessor.stabMutex.lock();
	while (!frameProcessor._stabComplete)
		frameProcessor.stabComplete.wait(frameProcessor.stabMutex);

	frameProcessor.resetRaster();
	if (stabilising)
	{
		if(bSystemLogFlag) {logger->info("[System::whenStabiliseToggled] Stabilise toggled on");}
		//CODE TO BE EXECUTED WHEN "STABILISER" IS ENABLED GOES HERE
	}
	else
	{
		if(bSystemLogFlag) {logger->info("[System::whenStabiliseToggled] Stabilise toggled off");}
		//CODE TO BE EXECUTED WHEN "STABILISER" IS DISABLED GOES HERE
	}
	frameProcessor.stabMutex.unlock();
}

void System::whenShowMapToggled(bool showingMap)
{
	if (showingMap)
	{
		if(bSystemLogFlag) {logger->info("[System::whenShowMapToggled] Show map toggled on");}
		stabiliser.predraw();
		frameProcessor.filters["map"] = &stabiliser;
		SDLWindow::setShowingMap(childwin, true);
	}
	else
	{
		if(bSystemLogFlag) {logger->info("[System::whenShowMapToggled] Show map toggled off");}
		frameProcessor.filters.erase("map");
		SDLWindow::setShowingMap(childwin, false);
	}

	/*
	if (!window.getLiveView().getValue() && !window.getPlayingBuffer().getValue() )
	{
		VidFrame *vframe = recorder->getFrame(window.getFrameSliderValue() );
		if (vframe)
		{
			frameQueue.clear();
			frameQueue.push(vframe);
		}
	}
	*/
}

void System::onFindFocusClicked() {
	//TODO: see if this fires or not
	if(bSystemLogFlag) {logger->info("[System::onFindFocusClicked] Find focus button clicked!");}
	imgcount = 0;
	bFindFocus = 1;

	usleep(1000000);
	bFindFocus = 0;
}


void System::onResetClicked() {
	if(bSystemLogFlag) {logger->info("[System::onResetClicked] Lens reset button clicked");}
}

void System::whenHoldFocusToggled(bool holdingFocus)
{
	//TODO: Fix this
	if (holdingFocus)
	{
		if(bSystemLogFlag) {logger->info("[System::whenHoldFocusToggled] Hold focus toggled on");}

		//CODE TO BE EXECUTED WHEN "HOLD FOCUS" IS ENABLED GOES HERE
		imgcount = 0;
		bHoldFocus = 1;
		usleep(100000); //sleep for 0.1 second
        window.setBestFocusScaleValue(desiredLocBestFocus);

	}
	else
	{
		if(bSystemLogFlag) {logger->info("[System::whenHoldFocusToggled] Hold focus toggled off");}
		//CODE TO BE EXECUTED WHEN "HOLD FOCUS" IS DISABLED GOES HERE
		bHoldFocus = 0;
	}
}

void System::when3DStabToggled(bool active)
{
	if (active)
	{
		if(bSystemLogFlag) {logger->info("[System::when3DStabToggled] 3D stab toggled on");}
		//CODE TO BE EXECUTED WHEN "3D STAB" IS ENABLED GOES HERE
	}
	else
	{
		if(bSystemLogFlag) {logger->info("[System::when3DStabToggled] 3D stab toggled off");}
		//CODE TO BE EXECUTED WHEN "3D STAB" IS DISABLED GOES HERE
	}
}

void System::when2DStabToggled(bool active2)
{
	if (active2)
	{
		if(bSystemLogFlag) {logger->info("[System::when2DStabToggled] 2D stab toggled on");}
		//CODE TO BE EXECUTED WHEN "2D STAB" IS ENABLED GOES HERE
	}
	else
	{
		if(bSystemLogFlag) {logger->info("[System::when2DStabToggled] 2D stab toggled off");}
		//CODE TO BE EXECUTED WHEN "2D STAB" IS DISABLED GOES HERE
	}
}

void System::whenLoadingToggled(bool loading)
{
	if (loading)
		recorder->signalOperationLoad().emit(window.getFileLocation() );
}

void System::whenSavingToggled(bool saving)
{
	if (saving)
	{
		window.displayMessage("Saving...");
		recorder->signalOperationSave().emit(window.getFileLocation() );
	}
}

void System::whenPlayingToggled(bool playing)
{
	if (playing)
	{
		int start = window.getFrameSliderValue();
		int frameRate = window.getFrameRateScaleValue();
		if (!recorder->isBuffering() )
		{
			if(bSystemLogFlag) {logger->info("[System::whenPlayingToggled] Start playing from frame: {} at rate: {}", start, frameRate);}
		
			recorder->signalBuffer().emit(std::make_pair(start, frameRate) );
		}
	}
	else
	{
		if(bSystemLogFlag) {logger->info("[System::whenPlayingToggled] Stop playing toggled off");}
		//TODO: Need to delete frames in frameQueue? Or manage in recorder?
		recorder->stopBuffering();
		frameQueue.clear();
	}
}

void System::whenSeekingToggled(bool seeking)
{
	if (seeking)
	{
		if(bSystemLogFlag) {logger->info("[System::whenSeekingToggled] Seeking toggled on");}

		if (window.getPlayingBuffer().getValue() )
		{
			recorder->stopBuffering();
		}
		else if (!window.getLiveView().getValue() )
		{
			VidFrame *out = recorder->getFrame(window.getFrameSliderValue() );
			if (out)
				frameQueue.push(out); //frameQueue is the same as sysQueue in FrameObserver
			window.setSeeking(false);
		}
		else 
			window.setSeeking(false);
	}
	else
	{
		if(bSystemLogFlag) {logger->info("[System::whenSeekingToggled] Seeking toggled off");}
		if (window.getPlayingBuffer().getValue() )
			whenPlayingToggled(true);
	}
}

void System::whenRecordingToggled(bool recording)
{
	if (recording)
	{
		recorder->clearFrames(); //When a new recording is started, clear the previous frames
	}
}

void System::onRecorderOperationComplete(RecOpRes res)
{
	OPRESEXPAND(res, op, success);

	switch (op)
	{
		case Recorder::Operation::RECOP_FILLED:
			if(bSystemLogFlag) {logger->info("[System::onRecorderOperationComplete] Recorder stopped recording");}
			if (success)
				window.displayMessage("Recording complete");
			else
				window.displayMessage("Recording failed");
			window.setRecording(false);
			//The rest is the same as if loaded
		case Recorder::Operation::RECOP_LOAD:
			if (window.getLoading().getValue() )
			{
				if (success)
					window.displayMessage("Loading complete");
				else
					window.displayMessage("Loading failed");
				window.setLoading(false);
			}
			window.setHasBuffer(success);
			window.setLiveView(!success);
			break;

		case Recorder::Operation::RECOP_SAVE:
			window.setSaving(false);
			if (success)
				window.displayMessage("Saving complete");
			else
				window.displayMessage("Saving failed");
			break;

		case Recorder::Operation::RECOP_BUFFER:
			if(bSystemLogFlag) {logger->info("[System::onRecorderOperationComplete] Recorder stopped buffering");}
			window.setPlayingBuffer(false);
			break;

		case Recorder::Operation::RECOP_ADDFRAME:
			if (window.getLoading().getValue() )
				window.displayMessage(std::string("Loaded frame: ") + std::to_string(recorder->countFrames() ) );
			else if (window.getRecording().getValue() )
				window.displayMessage(std::string("Recorded frame: ") + std::to_string(recorder->countFrames() ) );
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
	if(bSystemLogFlag) {logger->info("[System::onWindowThresholdChanged] Threshold changed to: {} recalculating map", val);}
	frameProcessor.stabMutex.lock();
	while (!frameProcessor._stabComplete)
		frameProcessor.stabComplete.wait(frameProcessor.stabMutex);

	stabiliser.adjust_thresh(val);
	if (stabiliser.is_valid() )
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
	if(bSystemLogFlag) {logger->info("[System::onWindowScaleChanged] Scale changed to: {} recalculating map", val);}

	frameProcessor.stabMutex.lock();
	while (!frameProcessor._stabComplete)
		frameProcessor.stabComplete.wait(frameProcessor.stabMutex);

	stabiliser.adjust_scale(val);
	if (stabiliser.is_valid() )
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
	//CODE TO EXECUTE WHEN BEST FOCUS SCALE CHANGED
	if(bSystemLogFlag) {logger->info("[System::onWindowBestFocusChanged] Best focus location changed to: {}, updating desiredLocBestFocus", val);}

	int val2 = round(val);
	//TODO: This should be in a mutex, and done much better. but it's not critical
	desiredLocBestFocus = val2;

}

void System::onWindowPauseClicked()
{
}

bool System::onCloseClicked(const GdkEventAny* event)
{
	//CODE TO EXECUTE WHEN CLOSE BUTTON CLICKED
	if(bSystemLogFlag) {logger->info("[System::onCloseClicked] Close button clicked");}
	//Close the streaming camera
	stopStreaming();
	// vsys.Shutdown(); //TODO: Maybe move to ~System()?
	// delete priv; //TODO: Maybe move to ~System()?
	return false;
}

System::~System()
{
    //m_conn.disconnect();
	vsys.Shutdown();
    delete priv;
    priv = nullptr; // Set to nullptr after deleting to avoid dangling pointer
	if(bSystemLogFlag) {logger->info("[System::~System] destructor ran");}
}

MainWindow& System::getWindow()
{
	return window;
}

Recorder& System::getRecorder()
{
	return *recorder;
}

void System::on_error()
{
	Gtk::MessageDialog dialog("Error: Air-Lens has hit the edge of the slide", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK, true);
	dialog.set_secondary_text("Please 'Reset' the lens and restart");
	dialog.set_position(Gtk::WIN_POS_CENTER);
	dialog.set_keep_above(true);
	dialog.run();

}