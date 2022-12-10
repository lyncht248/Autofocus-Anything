#ifndef HVIGTK_THREAD_H
#define HVIGTK_THREAD_H

#include "glibmm/main.h"
#include <gtkmm.h>
#include <queue>

Glib::RefPtr<Glib::MainContext> hvigtk_threadcontext();

template <typename T>
class VDispatcher
{
public:
	VDispatcher() :
		queue(),
		mutex(),
		dispatch(hvigtk_threadcontext() )
	{
	}

	void emit(T v)
	{
		mutex.lock();
		queue.push(v);
		dispatch.emit();
		mutex.unlock();
	}

	sigc::connection connect(const sigc::slot<void(T)>& slot)
	{
		return dispatch.connect([this,slot]()
		{
			mutex.lock();
			T v = queue.front();
			queue.pop();
			mutex.unlock();
			slot(v);
		});
	}

private:
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
			objCreated.signal();
			mutex.unlock();
			return false;
		});
		mutex.lock();
		objCreated.wait(mutex);
		mutex.unlock();
		return out;
	}

protected:
	void slot();
	Glib::Threads::Mutex mutex;
	Glib::Threads::Cond objCreated;

	Glib::RefPtr<Glib::MainLoop> loop;
	Glib::Threads::Thread *thread;
};

#endif
