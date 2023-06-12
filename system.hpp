#ifndef HVIGTK_SYSTEM_H
#define HVIGTK_SYSTEM_H


#include <fstream>
#include <iostream>
#include <queue>

#include "cairomm/surface.h"
#include "vidframe.hpp"
#include "mainwindow.hpp"
#include "autofocus.hpp"
#include "cond.hpp"
#include "thread.hpp"
#include "recorder.hpp"
#include "tsqueue.hpp"
#include "logfile.hpp"
#include "stabiliser.hpp"
#include "framefilter.hpp"

namespace Vimba = AVT::VmbAPI;

class System;

class FrameProcessor 
{
	friend class System;
public:
	FrameProcessor(System &sys);
	~FrameProcessor();
	
	void releaseFrame();
	void resetRaster();
	::Cairo::RefPtr< ::Cairo::Surface> getFrame();

protected: //Only available to derived and friend classes
	void stabilise();
	void processFrame();

	::Cairo::RefPtr< ::Cairo::ImageSurface> processed;
	VidFrame *vidFrame;
	std::unordered_map<std::string, FrameFilter*> filters;
	bool running;
	bool _stabNewFrame;
	bool _stabComplete;
	bool _frameReleased;

	Glib::Threads::Mutex mutex, stabMutex;
	Glib::Threads::Cond frameReleased, stabNewFrame, stabComplete;

	System &system;

	TooN::Vector<2> offset, currentOff, rasterPos;
	TSQueue<VidFrame *> stabQueue, released;

	Glib::Threads::Thread *processorThread, *stabThread;
};

/*
This class is responsible for all the processing behind the GUI, including frame streaming (see FrameProcessor), closing/opening the GUI, 
and operating other neccessary threads.
*/
class System
{
	friend class FrameProcessor; //Allows FrameProcessor to access private methods
public:

	// Constructor function for system class
	System(int argc, char **argv);
	~System();

	MainWindow& getWindow();
	Recorder& getRecorder();

	void startStreaming();
	void stopStreaming();
	
	template<typename T>
	void setFeature(const std::string &fname, T val)
	{
		if (cam != nullptr)
		{
			Vimba::FeaturePtr pFeature;
			if (cam->GetFeatureByName(fname.c_str(), pFeature ) == VmbErrorSuccess && pFeature->SetValue(val) == VmbErrorSuccess)
			{
				hvigtk_logfile << "Set " << fname << " to: " << val << std::endl;
				hvigtk_logfile.flush();
			}
			else
			{
				hvigtk_logfile << "FAILED to set " << fname << " to: " << val << std::endl;
				hvigtk_logfile.flush();
			}
		}
	}

	template<typename T>
	T getFeature(const std::string &fname)
	{
		if (cam != nullptr)
		{
			Vimba::FeaturePtr pFeature;
			if (cam->GetFeatureByName(fname.c_str(), pFeature ) == VmbErrorSuccess )
			{
				T out;
				pFeature->GetValue(out);
				return out;
			}
			else
			{
				hvigtk_logfile << "FAILED to get " << fname << std::endl;
				hvigtk_logfile.flush();
			}
		}

		return T();
	}

	TSQueue<VidFrame *>& getFrameQueue();
	Glib::Dispatcher& signalNewFrame();

	VidFrame* getFrame();


private:
	void renderFrame();
	void releaseFrame();

	void whenLiveViewToggled(bool viewingLive);

	void whenMakeMapToggled(bool makingMap);
	void whenStabiliseToggled(bool stabilising);
	void whenShowMapToggled(bool showingMap);

	void whenHoldFocusToggled(bool holdingFocus);
	void when3DStabToggled(bool active);
	void when2DStabToggled(bool active2);

	void whenLoadingToggled(bool loading);
	void whenSavingToggled(bool saving);
	void whenPlayingToggled(bool playing);
	void whenSeekingToggled(bool seeking);
	void whenRecordingToggled(bool recording);

	void onRecorderOperationComplete(RecOpRes res);
	void onWindowFeatureUpdated(std::string fname, double val);

	void onWindowThresholdChanged(double val);
	void onWindowScaleChanged(double val);
	void onWindowBestFocusChanged(double val);
	void onWindowPauseClicked();
	void onFindFocusClicked();
	void onResetClicked();

	bool onCloseClicked(const GdkEventAny* event);

	void on_error();

	struct Private;
	MainWindow window; //object from mainwindow.cpp class
	autofocus AF; //object from autofocus.cpp class
	Vimba::VimbaSystem &vsys; //For interacting with Vimba camera
	Vimba::CameraPtr cam; //For interacting with Vimba camera

	CVD::ImageRef size; 
	CVD::Image<unsigned int> im;

	struct Private *priv;

	Glib::Dispatcher sigNewFrame;

	TSQueue<VidFrame *> frameQueue;

	
	FrameProcessor frameProcessor;
	
	ObjectThread thread;

	//Recorder *recorder;
	std::unique_ptr<Recorder> recorder; //TODO: This causes a CRITICAL error, but fixes some memory management, so I'm leaving it for now
	VDispatcher<RecOpRes> sRecorderOperationComplete;

	Stabiliser stabiliser; //For running x-y stabilization
	bool madeMap;
};

#endif
