#ifndef HVIGTK_SYSTEM_H
#define HVIGTK_SYSTEM_H


#include <fstream>
#include <iostream>
#include <queue>

#include "vidframe.hpp"
#include "mainwindow.hpp"
#include "cond.hpp"
#include "thread.hpp"
#include "recorder.hpp"
#include "tsqueue.hpp"
#include "logfile.hpp"
#include "stabiliser.hpp"

#include <TooN/TooN.h>


namespace Vimba = AVT::VmbAPI;
using namespace TooN;

class System;

class FrameProcessor 
{
	friend class System;
public:

	// Starts a GLib::Thread and passes relevant variables (ie. mutex) to process frames using processFrame();
	FrameProcessor(System &sys);

	// Destructor that stops frame processing 
	~FrameProcessor();
	
	// Releases frame using Glib::Threads::Cond frameReleased
	void releaseFrame();

	//public function to get the most recent processed frame.
	VidFrame *getFrame();

protected: 
	// Processes frames according to a tsqueue
	void processFrame();

	//Pointer to the most recent proessed frame
	VidFrame *processed;
	std::vector<std::function<void(VidFrame*)> > filters;
	bool running;

	Glib::Threads::Mutex mutex;
	Glib::Threads::Cond frameReleased;

	System &system;
};

class System
{
public:
	//Constructor, sets up GUI by calling functions using MainWindow window object.
	// All GUI changes are handled by mainwindow.cpp from here. 
	System(int argc, char **argv);

	//Destructor, closes app
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

private:
	// Either pulls frame from recorder or frame processor (live cam) and sends to window
	void renderFrame();
	void releaseFrame();

	void whenLiveViewToggled(bool viewingLive);

	//Past logic for buttons below... 

	void whenMakeMapToggled(bool makingMap);
	void whenStabiliseToggled(bool stabilising);
	void whenShowMapToggled(bool showingMap);

	void whenFindFocusToggled(bool findingFocus);
	void whenHoldFocusToggled(bool holdingFocus);
	void when3DStabToggled(bool active);

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

	double Edge_Threshold, Edge_Scale;


	struct Private;
	MainWindow window;
	Vimba::VimbaSystem &vsys;
	Vimba::CameraPtr cam;

	CVD::ImageRef size;
	
	CVD::Image<unsigned char> im;
	//CVD::Image<unsigned int> im;


    Stabiliser my_stabiliser;
    Vector<2> offset;

	struct Private *priv;

	Glib::Dispatcher sigNewFrame;

	TSQueue<VidFrame *> frameQueue;
	
	FrameProcessor frameProcessor;
	
	ObjectThread thread;

	Recorder *recorder;
	VDispatcher<RecOpRes> sRecorderOperationComplete;
};

#endif
