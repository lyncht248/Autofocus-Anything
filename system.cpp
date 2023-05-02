#include "system.hpp"

#include <chrono>
#include <ctime>
#include <cvd/image_ref.h>
#include <set>
#include <gtkmm.h>
#include <cstring>
#include <string>

#include "VimbaC/Include/VmbCommonTypes.h"
#include "VimbaCPP/Include/Frame.h"
#include "VimbaCPP/Include/SharedPointerDefines.h"
#include "sdlwindow.hpp"
#include "thread.hpp"
#include "version.hpp"
#include "main.hpp"
#include "autofocus.hpp"



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
}

FrameProcessor::~FrameProcessor()
{
	running = false;
	ThreadStopper::stop({processorThread, stabThread});
}

void FrameProcessor::stabilise()
{
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

void FrameProcessor::processFrame()
{
	ThreadStopper::makeStoppable();
	while (running)
	{
		ThreadStopper::lock(mutex);
		vidFrame = system.getFrameQueue().pop();
		_frameReleased = false;
		ThreadStopper::unlock(mutex);

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

		bool recording = system.window.getRecording().getValue() && !system.window.getPausedRecording().getValue();
		if (recording)
		{
			system.getRecorder().putFrame(vidFrame);
		}

		CVD::ImageRef vfDim = vidFrame->size();
		if (vfDim.x != childwin->raster.w || vfDim.y != childwin->raster.h)
		{
			//processed = Cairo::ImageSurface::create(Cairo::FORMAT_RGB24, vfDim.x, vfDim.y);
			SDLWindow::createFrame(childwin, vfDim.x, vfDim.y);
		}
		
		//convertToCairo(stabilising);

		for (auto pair : filters)
		{
			pair.second->draw(processed);
		}

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

		ThreadStopper::lock(mutex);
		{
			while (!_frameReleased)
			{
				ThreadStopper::stopPoint(&frameReleased);
				frameReleased.wait(mutex);
			}
			if (!stabilising)
				released.push(vidFrame);
			if (system.window.getLiveView().getValue() && !recording)
			{
				for (VidFrame *vframe = released.pop(); !released.empty(); vframe = released.pop() )
					delete vframe;
				vidFrame = nullptr;
			}
			else
				released.clear();
		}
		ThreadStopper::unlock(mutex);
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
}

::Cairo::RefPtr< ::Cairo::Surface> FrameProcessor::getFrame()
{
	return processed;
}

class FrameObserver : public Glib::Object, virtual public Vimba::IFrameObserver
{
public :
	FrameObserver(Vimba::CameraPtr pCamera, TSQueue<VidFrame *> &frameQueue) :  IFrameObserver ( pCamera ), Object(),
		sigReceived(),
		sysFrames(frameQueue)
	{
	}
	void FrameReceived ( const Vimba::FramePtr pFrame )
	{
		VmbFrameStatusType eReceiveStatus;
		if( VmbErrorSuccess == pFrame -> GetReceiveStatus ( eReceiveStatus ) )
		{
			if (VmbFrameStatusComplete == eReceiveStatus)
			{

				VmbUint32_t sz, width, height;
				pFrame->GetImageSize(sz);
				pFrame->GetWidth(width);
				pFrame->GetHeight(height);

				IVidFrame *sysFrame = new IVidFrame(CVD::ImageRef(width, height) );
				//sysFrame->resize(CVD::ImageRef(width, height) );

				//pFrame->GetOffsetX(sysFrame.xoff);
				//pFrame->GetOffsetY(sysFrame.yoff);

				VmbUchar_t *buf = nullptr;
				pFrame->GetImage(buf);
				memcpy(sysFrame->data(), buf, sz);
				sysFrames.push(sysFrame);

				if (sysFrames.size() == 50)
				{
					hvigtk_logfile << "Frame queue filling up - playback too slow?" << std::endl;
					hvigtk_logfile.flush();
				}

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
	recorder(thread.createObject<Recorder, System&>(*this) ),
	sRecorderOperationComplete(),
	stabiliser(),
	madeMap(false)
{
	recorder->connectTo(&sRecorderOperationComplete);
	sRecorderOperationComplete.connect(sigc::mem_fun(*this, &System::onRecorderOperationComplete) );

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

	window.getFindFocusActive().signalToggled().connect(sigc::mem_fun(*this, &System::whenFindFocusToggled) );
	window.getHoldFocusActive().signalToggled().connect(sigc::mem_fun(*this, &System::whenHoldFocusToggled) );
	window.get3DStabActive().signalToggled().connect(sigc::mem_fun(*this, &System::when3DStabToggled) );

	window.getLoading().signalToggled().connect(sigc::mem_fun(*this, &System::whenLoadingToggled) );
	window.getSaving().signalToggled().connect(sigc::mem_fun(*this, &System::whenSavingToggled) );
	window.getPlayingBuffer().signalToggled().connect(sigc::mem_fun(*this, &System::whenPlayingToggled) );
	window.getSeeking().signalToggled().connect(sigc::mem_fun(*this, &System::whenSeekingToggled) );
	window.getRecording().signalToggled().connect(sigc::mem_fun(*this, &System::whenRecordingToggled) );

	hvigtk_logfile << "HVI-GTK " << HVIGTK_VERSION_STR << std::endl;
	hvigtk_logfile.flush();

	auto now = std::chrono::system_clock::now();
	std::time_t time = std::chrono::system_clock::to_time_t(now);
	
 	char buf[64] = { 0 };
    std::strftime(buf, sizeof(buf), "%F %R %Z", std::localtime(&time) );
	hvigtk_logfile << "Run time: " << buf << std::endl;
	hvigtk_logfile.flush();

	if (vsys.Startup() == VmbErrorSuccess)
	{
		hvigtk_logfile << "Starting up Vimba: success" << std::endl;
		Vimba::CameraPtrVector cameras;

		if (vsys.GetCameras(cameras) == VmbErrorSuccess)
		{
			hvigtk_logfile << "Found: " << cameras.size() << " cameras" << std::endl;
			hvigtk_logfile.flush();
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
			hvigtk_logfile << "Camera search failed" << std::endl;
			hvigtk_logfile.flush();
		}
	}
	else
	{
		hvigtk_logfile << "Failed to start Vimba" << std::endl;
		hvigtk_logfile.flush();
	}
	whenLiveViewToggled(true);
}

void System::startStreaming()
{
	if (cam != nullptr)
	{
		if (cam->Open(VmbAccessModeFull) == VmbErrorSuccess)
		{
			hvigtk_logfile << "Successfully opened camera" << std::endl;
			hvigtk_logfile.flush();
			
			setFeature("PixelFormat", VmbPixelFormatMono8);
			window.updateCameraValues(getFeature<double>("Gain"), getFeature<double>("ExposureTime"), getFeature<double>("Gamma") );
			setFeature("AcquisitionFrameRateEnable", true);
			setFeature<double>("AcquisitionFrameRate", window.getFrameRateScaleValue() );

			Vimba::FeaturePtr pFeature;
			cam->GetFeatureByName("PayloadSize", pFeature);
			VmbInt64_t nPLS;
			pFeature->GetValue(nPLS);
			
			priv->observer.reset(new FrameObserver(cam, frameQueue) );

			for (Vimba::FramePtrVector::iterator iter = priv->frames.begin(); iter != priv->frames.end(); ++iter)
			{
				(*iter).reset( new Vimba::Frame ( nPLS ) );
				(*iter)->RegisterObserver(priv->observer);
				cam->AnnounceFrame(*iter);
			}

			if (cam->StartCapture() == VmbErrorSuccess)
			{
				for (Vimba::FramePtrVector::iterator iter = priv->frames.begin(); iter != priv->frames.end(); ++iter)
				{
					cam->QueueFrame(*iter);
				}
				
				if (cam->GetFeatureByName("AcquisitionStart", pFeature) == VmbErrorSuccess && pFeature->RunCommand() == VmbErrorSuccess)
				{
					hvigtk_logfile << "Started frame capture" << std::endl;
					hvigtk_logfile.flush();
				}
				else
				{
					goto fail;
				}
			}
			else
			{
			fail:;
				hvigtk_logfile << "Failed to start frame capture" << std::endl;
				hvigtk_logfile.flush();
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
			hvigtk_logfile << "Stopped acquisition" << std::endl;
			hvigtk_logfile.flush();
		}

		if (cam->EndCapture() == VmbErrorSuccess)
		{
			hvigtk_logfile << "Stopped capture" << std::endl;
			hvigtk_logfile.flush();
		}

		if (cam->FlushQueue() == VmbErrorSuccess)
		{
			hvigtk_logfile << "Flushed queue" << std::endl;
			hvigtk_logfile.flush();
		}

		for (Vimba::FramePtrVector::iterator iter = priv->frames.begin(); iter != priv->frames.end(); ++iter)
		{
			(*iter)->UnregisterObserver();
		}
		
		if (cam->RevokeAllFrames() == VmbErrorSuccess)
		{
			hvigtk_logfile << "Revoked frames" << std::endl;
			hvigtk_logfile.flush();
		}

		if (cam->Close() == VmbErrorSuccess)
		{
			hvigtk_logfile << "Closed camera" << std::endl;
			hvigtk_logfile.flush();
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
		window.setShowingMap(true);
		madeMap = true;
	}
	frameProcessor.releaseFrame();
}

void System::whenLiveViewToggled(bool viewingLive)
{
	window.setMakingMap(false);
	if (viewingLive)
	{
		startStreaming();
	}
	else
	{
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
		frameProcessor.resetRaster();
		if (!window.getLiveView().getValue() )
		{
			VidFrame *vframe = recorder->getFrame(window.getFrameSliderValue() );
			if (vframe)
			{
				stabiliser.make_map(*vframe, DEFAULT_NUM_TRACKERS);
				window.setShowingMap(true);
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
		stabiliser.invalidate();
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
		//CODE TO BE EXECUTED WHEN "STABILISER" IS ENABLED GOES HERE
	}
	else
	{
		//CODE TO BE EXECUTED WHEN "STABILISER" IS DISABLED GOES HERE
	}
	frameProcessor.stabMutex.unlock();
}

void System::whenShowMapToggled(bool showingMap)
{
	if (showingMap)
	{
		stabiliser.predraw();
		frameProcessor.filters["map"] = &stabiliser;
		SDLWindow::setShowingMap(childwin, true);
	}
	else
	{
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

void System::whenFindFocusToggled(bool findingFocus)
{
	if (findingFocus)
	{
		//CODE TO BE EXECUTED WHEN "FIND FOCUS" IS ENABLED GOES HERE

		hvigtk_logfile << "Finding focus" << std::endl;
		imgcount = 0;
		bFindFocus = 1;
		usleep(500000); //half a second
		
		// untoggle the button
		window.getFindFocus().setValue(false);

	}
	else
	{
		bFindFocus = 0;
		//CODE TO BE EXECUTED WHEN "FIND FOCUS" IS DISABLED GOES HERE
	}
}

void System::whenHoldFocusToggled(bool holdingFocus)
{
	if (holdingFocus)
	{
		//CODE TO BE EXECUTED WHEN "HOLD FOCUS" IS ENABLED GOES HERE
		imgcount = 0;
		bHoldFocus = 1;

	}
	else
	{
		//CODE TO BE EXECUTED WHEN "HOLD FOCUS" IS DISABLED GOES HERE
		bHoldFocus = 0;
	}
}

void System::when3DStabToggled(bool active)
{
	if (active)
	{
		//CODE TO BE EXECUTED WHEN "3D STAB" IS ENABLED GOES HERE
	}
	else
	{
		//CODE TO BE EXECUTED WHEN "3D STAB" IS DISABLED GOES HERE
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
			hvigtk_logfile << "Start playing from frame: " << start << " at rate: " << frameRate << std::endl;
			hvigtk_logfile.flush();
			recorder->signalBuffer().emit(std::make_pair(start, frameRate) );
		}
	}
	else
	{
		recorder->stopBuffering();
		frameQueue.clear();
	}
}

void System::whenSeekingToggled(bool seeking)
{
	if (seeking)
	{
		if (window.getPlayingBuffer().getValue() )
		{
			recorder->stopBuffering();
		}
		else if (!window.getLiveView().getValue() )
		{
			VidFrame *out = recorder->getFrame(window.getFrameSliderValue() );
			if (out)
				frameQueue.push(out);
			window.setSeeking(false);
		}
		else 
			window.setSeeking(false);
	}
	else
	{
		if (window.getPlayingBuffer().getValue() )
			whenPlayingToggled(true);
	}
}

void System::whenRecordingToggled(bool recording)
{
	if (recording)
	{
		recorder->clearFrames();
	}
}

void System::onRecorderOperationComplete(RecOpRes res)
{
	OPRESEXPAND(res, op, success);

	switch (op)
	{
		case Recorder::Operation::RECOP_FILLED:
			hvigtk_logfile << "Recorder stopped recording" << std::endl;
			hvigtk_logfile.flush();
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
			hvigtk_logfile << "Recorder stopped buffering" << std::endl;
			hvigtk_logfile.flush();
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

	int val2 = round(val);
	center = val2;

}

void System::onWindowPauseClicked()
{
}

System::~System()
{
	vsys.Shutdown();
	delete priv;
}

MainWindow& System::getWindow()
{
	return window;
}

Recorder& System::getRecorder()
{
	return *recorder;
}
