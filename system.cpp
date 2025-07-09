#include "system.hpp"

#include <chrono>
#include <ctime>
#include <cvd/image_ref.h>
#include <set>
#include <gtkmm.h>
#include <cstring>
#include <string>

#include "VimbaC/Include/VmbCommonTypes.h"
#include "VimbaCPP/Include/SharedPointerDefines.h"
#include "glibmm/threads.h"
#include "sigc++/functors/mem_fun.h"
#include "version.hpp"

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
	mutex(),
	frameReleased(),
	system(sys)
{
	Glib::Threads::Thread *thread = Glib::Threads::Thread::create(sigc::mem_fun(*this, &FrameProcessor::processFrame) );
}

FrameProcessor::~FrameProcessor()
{
	running = false;
}

void FrameProcessor::processFrame()
{
	while (running)
	{
		//Gets most recent image ready for processing from TS Queue
		processed = system.getFrameQueue().pop();
		hvigtk_logfile << "just ran pop() and got processed" << std::endl;
    	hvigtk_logfile.flush();

		//Passes image to putFrame() if getValue in cond.hpp is true
		if (system.getWindow().getRecording().getValue() )
		{
			system.getRecorder().putFrame(new VidFrame(*processed) );
			hvigtk_logfile << "I think this is pop() for recordings" << std::endl;
    		hvigtk_logfile.flush();

		}

		// Filters frame if neccessary
		for (std::function<void(VidFrame *)> &filter : filters)
		{
			filter(processed);
		}

		// Signals that new frame has been processed
		system.signalNewFrame().emit();

		// Deletes processed frame
		mutex.lock();
		{
			frameReleased.wait(mutex);
			delete processed;
			processed = nullptr;
		}
		mutex.unlock();
	}
}

void FrameProcessor::releaseFrame()
{
	mutex.lock();
	frameReleased.broadcast();
	mutex.unlock();
}

VidFrame* FrameProcessor::getFrame()
{
	hvigtk_logfile << "getting a frame" << std::endl;
   	hvigtk_logfile.flush();

	return processed;
}

// Gets attributes of the recieved frame and prepares the GUI window
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
				VidFrame *sysFrame = new VidFrame();

				VmbUint32_t sz, width, height;
				pFrame->GetImageSize(sz);
				pFrame->GetWidth(width);
				pFrame->GetHeight(height);

				sysFrame->resize(CVD::ImageRef(width, height) );

				//pFrame->GetOffsetX(sysFrame.xoff);
				//pFrame->GetOffsetY(sysFrame.yoff);

				VmbUchar_t *buf = sysFrame->data();
				pFrame->GetImage(buf);
				sigReceived.emit();
				sysFrames.push(sysFrame);
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
	FrameObserver *observer;

	public:
		Private();
};

System::Private::Private() :
	observer(nullptr)
{
}

System::System(int argc, char **argv) :
	Edge_Threshold(0.4),
	Edge_Scale(2.00),
	window(),
	vsys(Vimba::VimbaSystem::GetInstance() ),
	cam(),
	size(),
	im(), //HERE
	priv(),
	sigNewFrame(hvigtk_threadcontext() ),
	frameQueue(),
	frameProcessor(*this),
	thread(),
	recorder(thread.createObject<Recorder, System&>(*this) ),
	sRecorderOperationComplete()
{
	recorder->connectTo(&sRecorderOperationComplete);
	sRecorderOperationComplete.connect(sigc::mem_fun(*this, &System::onRecorderOperationComplete) );

	window.signalFrameDrawn().connect(sigc::mem_fun(*this, &System::releaseFrame) );
	window.signalFeatureUpdated().connect(sigc::mem_fun(*this, &System::onWindowFeatureUpdated) );
	window.signalThresholdChanged().connect(sigc::mem_fun(*this, &System::onWindowThresholdChanged) );
	window.signalScaleChanged().connect(sigc::mem_fun(*this, &System::onWindowScaleChanged) );
	window.signalBestFocusChanged().connect(sigc::mem_fun(*this, &System::onWindowBestFocusChanged) );

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
				setFeature("PixelFormat", VmbPixelFormatMono12);
				window.updateCameraValues(getFeature<double>("Gain"), getFeature<double>("ExposureTime"), getFeature<double>("Gamma") );
				setFeature("AcquisitionFrameRateEnable", true);
				setFeature<double>("AcquisitionFrameRate", 30.0);
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

			Vimba::FeaturePtr pFeature;
			cam->GetFeatureByName("PayloadSize", pFeature);
			VmbInt64_t nPLS;
			pFeature->GetValue(nPLS);
			
			if (priv->observer != nullptr)
				delete priv->observer;
			priv->observer = new FrameObserver(cam, frameQueue);
			Vimba::IFrameObserverPtr pObserver(priv->observer);

			Vimba::FramePtrVector frames(3);
			for (Vimba::FramePtrVector::iterator iter = frames.begin(); iter != frames.end(); ++iter)
			{
				(*iter).reset( new Vimba::Frame ( nPLS ) );
				(*iter)->RegisterObserver(pObserver);
				cam->AnnounceFrame(*iter);
			}

			if (cam->StartCapture() == VmbErrorSuccess)
			{
				for (Vimba::FramePtrVector::iterator iter = frames.begin(); iter != frames.end(); ++iter)
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

void System::renderFrame()
{
	if (window.getLiveView().getValue() )
		window.renderFrame(frameProcessor.getFrame() );
	else
		window.renderFrame(recorder->getFrame() );
}

void System::releaseFrame()
{
	if (window.getLiveView().getValue() )
	{
		frameProcessor.releaseFrame();
	}
	else
	{
		recorder->releaseFrame();
	}
}

void System::whenLiveViewToggled(bool viewingLive)
{
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
	if (makingMap)
	{
		im = *recorder->getFrame();
		//CODE TO BE EXECUTED WHEN "MAKE MAP" IS ENABLED GOES HERE

		my_stabiliser.make_map(im,300);
  		// stabilise_button->set(); This should go in mainwindow.cpp
  		// show_map_button->set(); This should go in mainwindow.cpp
  		offset=Zeros;
  		glRasterPos2i(0,0); // Used to position the openGL rendering location... May need to change to match central image. 
  		// fltk::focus(mygl);
	}
	else
	{
		//CODE TO BE EXECUTED WHEN "MAKE MAP" IS DISABLED GOES HERE
	}
}

void System::whenStabiliseToggled(bool stabilising)
{
	if (stabilising)
	{
		//CODE TO BE EXECUTED WHEN "STABILISER" IS ENABLED GOES HERE
	}
	else
	{
		//CODE TO BE EXECUTED WHEN "STABILISER" IS DISABLED GOES HERE
	}
}

void System::whenShowMapToggled(bool showingMap)
{
	if (showingMap)
	{
		hvigtk_logfile << "show map toggled" << std::endl;
		hvigtk_logfile.flush();

		//CODE TO BE EXECUTED WHEN "SHOW MAP" IS ENABLED GOES HERE
		glColor3f(0,1,0);
    	glPointSize(1);
    	my_stabiliser.draw();
		
		hvigtk_logfile << "about to draw pixels" << std::endl;
		hvigtk_logfile.flush();

		glDrawPixels(im);
		hvigtk_logfile << "pixels drawn" << std::endl;
		hvigtk_logfile.flush();

	}
	else
	{
		//CODE TO BE EXECUTED WHEN "SHOW MAP" IS DISABLED GOES HERE
	}
}

void System::whenFindFocusToggled(bool findingFocus)
{
	if (findingFocus)
	{
		//CODE TO BE EXECUTED WHEN "FIND FOCUS" IS ENABLED GOES HERE
	}
	else
	{
		//CODE TO BE EXECUTED WHEN "FIND FOCUS" IS DISABLED GOES HERE
	}
}

void System::whenHoldFocusToggled(bool holdingFocus)
{
	if (holdingFocus)
	{
		//CODE TO BE EXECUTED WHEN "HOLD FOCUS" IS ENABLED GOES HERE
	}
	else
	{
		//CODE TO BE EXECUTED WHEN "HOLD FOCUS" IS DISABLED GOES HERE
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
		if (!recorder->isBuffering() )
		{
			hvigtk_logfile << "Start playing from frame: " << start << std::endl;
			hvigtk_logfile.flush();
			recorder->signalBuffer().emit(start);
		}
	}
	else
	{
		recorder->stopBuffering();
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
			window.renderFrame(recorder->getFrame(window.getFrameSliderValue() ) );
			window.setSeeking(false);
		}
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
	Edge_Threshold = val;
}

void System::onWindowScaleChanged(double val)
{
	Edge_Scale = val;
}

void System::onWindowBestFocusChanged(double val)
{
	//CODE TO EXECUTE WHEN BEST FOCUS SCALE CHANGED
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
