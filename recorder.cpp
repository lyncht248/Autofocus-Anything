#include <chrono>
#include <thread>
#include <cvd/image_io.h>
#include <sys/stat.h>
#include "recorder.hpp"
#include "logfile.hpp"
#include "system.hpp"
#include "main.hpp"

#define FNUM_SIZE 26

bool bRecorderLogFlag = 0; // 1 = log, 0 = don't log

Recorder::Recorder(System &sys) :
	system(sys),
	buffering(false),
	frames(),
	sigOperationComplete(nullptr),
	sigOperationSave(),
	sigOperationLoad(),
	sigBuffer(),
	mutex(), // Holds frame data
	frameReleased(),
	current(nullptr)
{
	sigOperationLoad.connect(sigc::mem_fun(*this, &Recorder::loadFrames) );
	sigOperationSave.connect(sigc::mem_fun(*this, &Recorder::saveFrames) );
	sigBuffer.connect(sigc::mem_fun(*this, &Recorder::bufferFrames) );

	if(bRecorderLogFlag) logger->info("[Recorder::Recorder()] constructor called");
}

Recorder::~Recorder()
{
	for (VidFrame *vf : frames)
	{
		delete vf;
	}
	frames.clear(); //Added this to try to fix a memory leak
	if(bRecorderLogFlag) logger->info("[Recorder::~Recorder()] destructor called");
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

	//If the frames queue size is equal to the recording size scale value, emit the RECOP_FILLED signal
	if (frames.size() == (int) system.getWindow().getRecordingSizeScaleValue() )
		emitOperationComplete(Operation::RECOP_FILLED, true);

	//Should always be the length of the recorded frames
	if(bRecorderLogFlag) logger->info("[Recorder::putFrame()] frames queue is size: {}", frames.size());
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
			CVD::img_save(*(frames[i]), ofs, CVD::ImageType::PNM);
		}
		else
		{
			emitOperationComplete(Operation::RECOP_SAVE, false);
			return;
		}
	}
	if(bRecorderLogFlag) logger->info("[Recorder::saveFrames()] frames saved to: {}", location);

    // Delete the frames. TODO is this correct?? 
    for (unsigned long i = 0; i < frames.size();)
    {
        delete frames[i];
        frames.erase(frames.begin() + i);
    }

	emitOperationComplete(Operation::RECOP_SAVE, true);
}

void Recorder::loadFrames(const std::string &location)
{
	//Clears existing frames in recorder
	for (VidFrame *vf : frames)
	{
		delete vf;
	}
	frames.clear();

	//Loads new frames from location
	int i = 0;
	char fnum[FNUM_SIZE];
	while (true)
	{
		std::snprintf(fnum, FNUM_SIZE, "/hvi-video-%.5d.pgm", i);
		std::ifstream ifs(location + fnum);
		if (ifs.is_open() )
		{
			// IVidFrame *frame = new IVidFrame();
			// CVD::img_load(*frame, ifs);
			// auto sz = frame->size();
			// frames.push_back(frame);
			// i++;
			//emitOperationComplete(Operation::RECOP_ADDFRAME, true);

			IVidFrame *tempFrame = new IVidFrame();
			CVD::img_load(*tempFrame, ifs);

			VidFrame *frame = new VidFrame(*tempFrame); // Use the copy constructor

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
	if(bRecorderLogFlag) logger->info("[Recorder::loadFrames()] frames loaded from: {}", location);
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

void Recorder::clearFrames() //Called ONLY when a new recording is started... call more often?
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

// Loads frames into FrameQueue (see FrameObserver) at data.second frames per second, starting at data.first frame
void Recorder::bufferFrames(std::pair<int, int> data)
{
	int start = data.first;
	int bufSleep = round(1000.0 / data.second);
	if(bRecorderLogFlag) logger->info("[Recorder::bufferFrames()] buffering started from frame: {}", start);
	buffering = true;
	//Plays all frames from start to end
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
	system.getFrameQueue().waitForEmpty(); //Stops here until all frames have been played
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
