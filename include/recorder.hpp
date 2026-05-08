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
	
	std::shared_ptr<IVidFrame> getFrame(int n);
	int putFrame(std::shared_ptr<IVidFrame> frame);

	void saveFrames(const std::string &location);
	void loadFrames(const std::string &location);

	bool isBuffering() const; 

	void stopBuffering();

	int countFrames();

	std::shared_ptr<IVidFrame> getFrame();
	// void releaseFrame();

	void clearFrames();

	void resetCurrent();

	void setBufferFrameRate();

	void connectTo(VDispatcher<std::tuple<Operation, bool> > *sOperationComplete);
	//Glib::Dispatcher* signalOperationComplete();
	VDispatcher<std::string>& signalOperationLoad();
	VDispatcher<std::string>& signalOperationSave();
	VDispatcher<std::pair<int, int> >& signalBuffer();

private:
	void emitOperationComplete(Operation op, bool success);
	void bufferFrames(std::pair<int, int> data); // 
	System &system;
	bool buffering;
	std::vector<std::shared_ptr<IVidFrame> > frames;
	VDispatcher<std::tuple<Operation, bool> > *sigOperationComplete;
	VDispatcher<std::string> sigOperationSave, sigOperationLoad;
	VDispatcher<std::pair<int, int> > sigBuffer;

	Glib::Threads::Mutex mutex;
	Glib::Threads::Cond frameReleased;

	std::shared_ptr<IVidFrame> current;
	int bufSleep;
};

using RecOpRes = std::tuple<Recorder::Operation, bool>;

#define OPRESEXPAND(res, op, success) Recorder::Operation op; bool success; std::tie(op, success) = (res)

#endif
