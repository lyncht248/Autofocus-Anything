/**
 * @file tsqueue.hpp
 * @brief Thread-safe queue implementation with optional size limit.
 */

#ifndef HVIGTK_TSQUEUE_H
#define HVIGTK_TSQUEUE_H

#include <gtkmm.h>
#include <queue>
#include "thread.hpp"

/**
 * @class TSQueue
 * @brief A thread-safe queue with optional size limit.
 *
 * This class provides a thread-safe queue implementation using Glib::Threads::Mutex
 * and Glib::Threads::Cond for synchronization. It supports operations like push,
 * pop, clear, and waiting for the queue to become empty.
 *
 * @tparam T The type of elements stored in the queue.
 */
template <class T>
class TSQueue
{
public:
	/**
	 * @brief Constructor with a size limit.
	 * @param limit The maximum number of elements the queue can hold.
	 */
	TSQueue(unsigned long limit) :
		queue(),
		mutex(),
		hasItems(),
		isEmpty(),
		limit(limit)
		{
			
		}
		queue(),
		mutex(),
		hasItems(),
		isEmpty(),
		limit(limit)
	{
	}

	/**
	 * @brief Default constructor with no size limit.
	 */
	TSQueue() : 
		TSQueue(~(0UL) )
	{
	}

	/**
	 * @brief Pushes an element into the queue.
	 *
	 * If the queue exceeds the size limit, the oldest element is removed.
	 * @param val The element to push into the queue.
	 */
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

	/**
	 * @brief Pops an element from the queue.
	 *
	 * Waits if the queue is empty until an element is available.
	 * @return The front element of the queue.
	 */
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

	/**
	 * @brief Clears all elements from the queue.
	 */
	void clear()
	{
		mutex.lock();
		while (!queue.empty() )
			queue.pop();
		isEmpty.broadcast();
		mutex.unlock();
	}

	/**
	 * @brief Checks if the queue is empty.
	 * @return True if the queue is empty, false otherwise.
	 */
	bool empty()
	{
		return queue.empty();
	}

	/**
	 * @brief Waits until the queue becomes empty.
	 */
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

	/**
	 * @brief Gets the current size of the queue.
	 * @return The number of elements in the queue.
	 */
	unsigned long size() const
	{
		return queue.size();
	}

private:
    std::queue<T> queue; /**< The underlying queue storing elements. */
    Glib::Threads::Mutex mutex; /**< Mutex for synchronizing access to the queue. */
    Glib::Threads::Cond hasItems; /**< Condition variable for signaling when items are added. */
    Glib::Threads::Cond isEmpty; /**< Condition variable for signaling when the queue becomes empty. */
    unsigned long limit; /**< The maximum number of elements the queue can hold. */
};

#endif
