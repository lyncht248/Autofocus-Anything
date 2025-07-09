#ifndef HVIGTK_THREAD_H
#define HVIGTK_THREAD_H

#include "glibmm/main.h"
#include "glibmm/threads.h"
#include "sigc++/connection.h"
#include <gtkmm.h>
#include <initializer_list>
#include <queue>

Glib::RefPtr<Glib::MainContext> hvigtk_threadcontext();

/*
 * Interface to allow one thread to nicely request another thread to finish at predefined 'stop points'
 */
namespace ThreadStopper
{
	/*
	 * Make the current thread responsive to stop requests. This should be called at the start of the thread routine.
	 */
	extern void makeStoppable();

	/*
	 * Lock a mutex such that it is automatically unlocked when the thread is stopped. Note that the mutex will only be unlocked on a 'stop request' - it will not be unlocked if the thread exits normally or by means other than ThreadStopper::stop.
	 */
	extern void lock(Glib::Threads::Mutex &mutex);

	/*
	 * Unlock the mutex and tell ThreadStopper not to automatically unlock it
	 */
	extern void unlock(Glib::Threads::Mutex &mutex);

	/*
	 * Set a stop point at which the current thread will respond to a stop request. A 'condition' variable may optionally be provided which will make the current thread responsive to stop requests while waiting on the condition.
	 */
	extern void stopPoint(Glib::Threads::Cond *condition=nullptr);

	/*
	 * Send a stop request to the specified threads. This function will call 'join()' on each of the threads.
	 */
	extern void stop(std::initializer_list<Glib::Threads::Thread*> ts);

	/*
	 * Wait for a thread to finish and clean up any data ThreadStopper may have for it.
	 */
	extern void join(Glib::Threads::Thread *thread);
};

template <typename T>
class VDispatcher
{
	friend class Connection;

	class Connection
	{
		friend class VDispatcher;
	public:
		Connection(VDispatcher<T> &dispatcher, const sigc::connection &connection) :
			dispatcher(dispatcher),
			connection(connection)
		{
		}

		Connection(const Connection &connection) :
			Connection(connection.dispatcher, connection.connection)
		{
		}

		void disconnect()
		{
			dispatcher.mutex.lock();
			dispatcher.nConnections--;
			if (dispatcher.nConnections == 0)
			{
				dispatcher.queue.clear();
				dispatcher.n = 0;
			}
			else
			{
				dispatcher.n--;
				if (dispatcher.n == 0)
				{
					dispatcher.queue.pop();
					dispatcher.n = dispatcher.nConnections;
				}
			}
			dispatcher.mutex.unlock();
			connection.disconnect();
		}

		VDispatcher<T> &dispatcher;
		sigc::connection connection;
	};

public:
	VDispatcher() :
		nConnections(0),
		n(0),
		queue(),
		mutex(),
		dispatch(hvigtk_threadcontext() )
	{
	}

	void emit(const T &v)
	{
		mutex.lock();
		queue.push(v);
		mutex.unlock();
		dispatch.emit();
	}

	Connection connect(const sigc::slot<void(const T&)>& slot)
	{
		mutex.lock();
		nConnections++;
		n++;
		mutex.unlock();
		Connection out(*this, dispatch.connect([this,slot]()
		{
			mutex.lock();
			T v = queue.front();
			n--;
			if (n == 0)
			{
				queue.pop();
				n = nConnections;
			}
			mutex.unlock();
			slot(v);
		}) );
		return out;
	}

private:
	int nConnections;
	int n;
	std::queue<T> queue;
	Glib::Threads::Mutex mutex;
	Glib::Dispatcher dispatch;
};

class ObjectThread
{
public:
	ObjectThread();
	~ObjectThread();

	template <class T, typename... As> 
	T* createObject(As... as)
	{
		T *out;
		loop->get_context()->invoke([this, &out, &as...]() 
		{
			mutex.lock();
			out = new T(as...);
			_objCreated = true;
			objCreated.signal();
			mutex.unlock();
			return false;
		});
		mutex.lock();
		_objCreated = false;
		while (!_objCreated)
			objCreated.wait(mutex);
		mutex.unlock();
		return out;
	}

protected:
	void slot();
	Glib::Threads::Mutex mutex;
	Glib::Threads::Cond objCreated;
	Glib::Threads::Cond started;
	bool _started, _objCreated;

	Glib::RefPtr<Glib::MainLoop> loop;
	Glib::Threads::Thread *thread;
};

#endif
