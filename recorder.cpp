#include <chrono>
#include <thread>
#include <cvd/image_io.h>
#include <sys/stat.h>
#include "recorder.hpp"
#include "logfile.hpp"
#include "system.hpp"
#include "main.hpp"

#define FNUM_SIZE 21

Recorder::Recorder(System &sys) :
	system(sys),
	buffering(false),
	frames(),
	sigOperationComplete(nullptr),
	sigOperationSave(),
	sigOperationLoad(),
	sigBuffer(),
	mutex(),
	frameReleased(),
	current(nullptr)
{
	sigOperationLoad.connect(sigc::mem_fun(*this, &Recorder::loadFrames) );
	sigOperationSave.connect(sigc::mem_fun(*this, &Recorder::saveFrames) );
	sigBuffer.connect(sigc::mem_fun(*this, &Recorder::bufferFrames) );
}

Recorder::~Recorder()
{
	for (VidFrame *vf : frames)
	{
		delete vf;
	}
}

VidFrame* Recorder::getFrame(int n)
{
	if (n < frames.size() )
	{
		return frames[n];
	}
	else
		return nullptr;
}

int Recorder::putFrame(VidFrame *frame)
{
	frames.push_back(frame);
	emitOperationComplete(Operation::RECOP_ADDFRAME, true);
	if (frames.size() == (int) system.getWindow().getRecordingSizeScaleValue() )
		emitOperationComplete(Operation::RECOP_FILLED, true);
	return frames.size();
}

void Recorder::saveFrames(const std::string &location)
{
	char fnum[FNUM_SIZE];
	mkdir(location.c_str(), 0777);
	for (unsigned long i = 0; i < frames.size(); i++)
	{
		std::snprintf(fnum, FNUM_SIZE, "/hvi-video-%.5lu.pgm", i);
		std::ofstream ofs(location + fnum);
		if (ofs.is_open() )
		{
			hvigtk_logfile << "Saving: " << location << fnum << std::endl;
			hvigtk_logfile.flush();
			CVD::img_save(*(frames[i]), ofs, CVD::ImageType::PNM);
		}
		else
		{
			emitOperationComplete(Operation::RECOP_SAVE, false);
			return;
		}
	}
	emitOperationComplete(Operation::RECOP_SAVE, true);
}

void Recorder::loadFrames(const std::string &location)
{
	for (VidFrame *vf : frames)
	{
		delete vf;
	}
	frames.clear();
	int i = 0;
	char fnum[FNUM_SIZE];
	while (true)
	{
		std::snprintf(fnum, FNUM_SIZE, "/hvi-video-%.5d.pgm", i);
		std::ifstream ifs(location + fnum);
		if (ifs.is_open() )
		{
			IVidFrame *frame = new IVidFrame();
			CVD::img_load(*frame, ifs);
			auto sz = frame->size();
			hvigtk_logfile << "Loading: " << location << fnum << " (" << sz.x << ", " << sz.y << ")" << std::endl;
			hvigtk_logfile.flush();
			frames.push_back(frame);
			i++;
			emitOperationComplete(Operation::RECOP_ADDFRAME, true);
		}
		else
		{
			emitOperationComplete(Operation::RECOP_LOAD, frames.size() > 0);
			return;
		}
	}
}

int Recorder::countFrames()
{
	return frames.size();
}

VidFrame* Recorder::getFrame()
{
	return current;
}

void Recorder::releaseFrame()
{
	mutex.lock();
	frameReleased.broadcast();
	mutex.unlock();
}

void Recorder::clearFrames()
{
	for (VidFrame *frame : frames)
		delete frame;
	frames.clear();
	emitOperationComplete(Operation::RECOP_EMPTIED, true);
}

void Recorder::emitOperationComplete(Operation op, bool success)
{
	if (sigOperationComplete != nullptr)
		sigOperationComplete->emit(std::make_tuple(op, success) );
}

void Recorder::bufferFrames(std::pair<int, int> data)
{
	int start = data.first;
	int bufSleep = round(1000.0 / data.second);
	hvigtk_logfile << "Recorder buffering started from frame: " << start << std::endl;
	hvigtk_logfile.flush();
	buffering = true;
	for (unsigned long i = start; buffering && i < frames.size(); i++)
	{
		//mutex.lock();
		{
			current = frames[i];
			system.getFrameQueue().push(frames[i]);
			std::this_thread::sleep_for(std::chrono::milliseconds(bufSleep) );
		}
		//mutex.unlock();
	}
	system.getFrameQueue().waitForEmpty();
	buffering = false;
	emitOperationComplete(Operation::RECOP_BUFFER, true);
}

bool Recorder::isBuffering() const
{
	return buffering;
}

void Recorder::stopBuffering()
{
	buffering = false;
}

void Recorder::connectTo(VDispatcher<RecOpRes> *sOperationComplete)
{
	sigOperationComplete = sOperationComplete;
}
/*
Glib::Dispatcher* Recorder::signalOperationComplete()
{
	return sigOperationComplete;
}
*/

VDispatcher<std::pair<int, int> >& Recorder::signalBuffer()
{
	return sigBuffer;
}

VDispatcher<std::string>& Recorder::signalOperationSave()
{
	return sigOperationSave;
}

VDispatcher<std::string>& Recorder::signalOperationLoad()
{
	return sigOperationLoad;
}
