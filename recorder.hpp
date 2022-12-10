#ifndef HVIGTK_RECORDER_H
#define HVIGTK_RECORDER_H

#include <tuple>
#include <vector>
#include <gtkmm.h>

#include "thread.hpp"
#include "vidframe.hpp"

class System;

class Recorder
{
public:
	enum Operation
	{
		RECOP_SAVE,
		RECOP_LOAD,
		RECOP_BUFFER,
		RECOP_FILLED,
		RECOP_EMPTIED,
		RECOP_ADDFRAME
	};
	Recorder(System &sys);
	~Recorder();
	
	// Returns frame n of recording, if there is a recording
	VidFrame* getFrame(int n);

	// Puts frame into position n of a recording
	int putFrame(VidFrame *frame);

	// Saves or loads all the frames in a given location into 'frames' vector
	void saveFrames(std::string location);
	void loadFrames(std::string location);

	// True when busy saving/loading
	bool isBuffering() const;

	void stopBuffering();

	int countFrames();

	// Returns current frame
	VidFrame* getFrame();
	void releaseFrame();

	void clearFrames();
	
	void connectTo(VDispatcher<std::tuple<Operation, bool> > *sOperationComplete);
	//Glib::Dispatcher* signalOperationComplete();
	VDispatcher<std::string>& signalOperationLoad();
	VDispatcher<std::string>& signalOperationSave();
	VDispatcher<int>& signalBuffer();
private:
	void emitOperationComplete(Operation op, bool success);
	void bufferFrames(int start);
	System &system;
	bool buffering;
	std::vector<VidFrame*> frames; // This is the vector of video frames
	VDispatcher<std::tuple<Operation, bool> > *sigOperationComplete;
	VDispatcher<std::string> sigOperationSave, sigOperationLoad;
	VDispatcher<int> sigBuffer;

	Glib::Threads::Mutex mutex;
	Glib::Threads::Cond frameReleased;

	VidFrame *current;
};

using RecOpRes = std::tuple<Recorder::Operation, bool>;

#define OPRESEXPAND(res, op, success) Recorder::Operation op; bool success; std::tie(op, success) = (res)

#endif
