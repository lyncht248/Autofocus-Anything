#ifndef HVIGTK_TSQUEUE_H
#define HVIGTK_TSQUEUE_H

#include <gtkmm.h>
#include <queue>
#include "thread.hpp"

template <class T>
class TSQueue
{
public:
	TSQueue(unsigned long limit) :
		queue(),
		mutex(),
		hasItems(),
		isEmpty(),
		limit(limit)
	{
	}

	TSQueue() : 
		TSQueue(~(0UL) )
	{
	}

	void push(T val)
	{
		mutex.lock();
		queue.push(val);
		if (queue.size() == 1)
			hasItems.broadcast();
		else if (queue.size() > limit)
			queue.pop();
		mutex.unlock();
	}

	T pop()
	{
		ThreadStopper::lock(mutex);
		while (queue.empty() )
		{
			ThreadStopper::stopPoint(&hasItems);
			hasItems.wait(mutex);
		}
		T out = queue.front();
		queue.pop();
		if (queue.empty() )
			isEmpty.broadcast();
		ThreadStopper::unlock(mutex);
		return out;
	}

	void clear()
	{
		mutex.lock();
		while (!queue.empty() )
			queue.pop();
		isEmpty.broadcast();
		mutex.unlock();
	}

	bool empty()
	{
		return queue.empty();
	}

	void waitForEmpty()
	{
		ThreadStopper::lock(mutex);
		while (!queue.empty() )
		{
			ThreadStopper::stopPoint(&isEmpty);
			isEmpty.wait(mutex);
		}
		ThreadStopper::unlock(mutex);
	}

	unsigned long size() const
	{
		return queue.size();
	}

private:
	std::queue<T> queue;
	Glib::Threads::Mutex mutex;
	Glib::Threads::Cond hasItems;
	Glib::Threads::Cond isEmpty;
	unsigned long limit;
};

#endif
