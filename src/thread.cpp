#include "thread.hpp"
#include "glibmm/main.h"
#include "glibmm/threads.h"
#include <unordered_map>
#include <unordered_set>

namespace ThreadStopper
{
	struct ThreadData
	{
		std::unordered_set<Glib::Threads::Mutex *> mutexes;
		Glib::Threads::Cond *cond;
		bool stopped;
	};
	static std::unordered_map<Glib::Threads::Thread*, ThreadData> threads;
	static Glib::Threads::Mutex mutex;

	void makeStoppable()
	{
		if (!threads.count(Glib::Threads::Thread::self() ) )
		{
			mutex.lock();
			ThreadData &data = threads[Glib::Threads::Thread::self()];
			data.stopped = false;
			mutex.unlock();
		}
	}

	void lock(Glib::Threads::Mutex &mutex)
	{
		mutex.lock();
		Glib::Threads::Thread *self = Glib::Threads::Thread::self();
		if (threads.count(self) )
			threads[self].mutexes.insert(&mutex);
	}

	void unlock(Glib::Threads::Mutex &mutex)
	{
		Glib::Threads::Thread *self = Glib::Threads::Thread::self();
		if (threads.count(self) )
			threads[self].mutexes.erase(&mutex);
		mutex.unlock();
	}

	void stopPoint(Glib::Threads::Cond *condition)
	{
		Glib::Threads::Thread *self = Glib::Threads::Thread::self();
		mutex.lock();
		if (threads.count(self) )
		{
			if (threads[self].stopped)
			{
				mutex.unlock();
				for (Glib::Threads::Mutex *m : threads[self].mutexes)
					m->unlock();
				throw Glib::Threads::Thread::Exit();
			}

			threads[self].cond = condition;
		}
		mutex.unlock();
	}

	void stop(std::initializer_list<Glib::Threads::Thread*> ts)
	{
		for (Glib::Threads::Thread *thread : ts)
		{
			mutex.lock();
			if (threads.count(thread) )
			{
				threads[thread].stopped = true;
				if (threads[thread].cond)
				{
					threads[thread].cond->broadcast();
				}
			}
			mutex.unlock();
			join(thread);
		}
	}

	void join(Glib::Threads::Thread *thread)
	{
		thread->join();
		threads.erase(thread);
	}
}

Glib::RefPtr<Glib::MainContext> hvigtk_threadcontext()
{
	return Glib::MainContext::get_thread_default();
}

ObjectThread::ObjectThread() :
	mutex(),
	objCreated(),
	started(),
	_started(false),
	_objCreated(false),
	loop(),
	thread(Glib::Threads::Thread::create(sigc::mem_fun(*this, &ObjectThread::slot) ) )
{
	mutex.lock();
	while (!_started)
		started.wait(mutex);
	mutex.unlock();
}

ObjectThread::~ObjectThread()
{
	loop->quit();
}

void ObjectThread::slot()
{
	mutex.lock();
	Glib::RefPtr<Glib::MainContext> cxt = Glib::MainContext::create();
	loop = Glib::MainLoop::create(cxt);
	cxt->push_thread_default();
	_started = true;
	started.signal();
	mutex.unlock();
	loop->run();
}
