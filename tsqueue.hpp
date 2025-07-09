// Temporary storage queue, used to hold images that are ready for processing

#ifndef HVIGTK_TSQUEUE_H
#define HVIGTK_TSQUEUE_H

#include <gtkmm.h>
#include <queue>

template <class T>
class TSQueue
{
public:
	TSQueue() :
		queue(),
		mutex(),
		hasItems()
	{
	}

	// Adds element to queue
	void push(T val)
	{
		mutex.lock();
		queue.push(val);
		hasItems.broadcast();
		mutex.unlock();
	}

	// Gets first element from TS queue, deletes that element from the queue
	T pop()
	{
		mutex.lock();
		if (queue.empty() )
			hasItems.wait(mutex);
		T out = queue.front();
		queue.pop();
		mutex.unlock();
		return out;
	}

	void clear()
	{
		mutex.lock();
		queue.clear();
		mutex.unlock();
	}

private:
	std::queue<T> queue;
	Glib::Threads::Mutex mutex;
	Glib::Threads::Cond hasItems;
};

#endif
