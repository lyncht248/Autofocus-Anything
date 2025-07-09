#include "thread.hpp"
#include "glibmm/main.h"
#include "glibmm/threads.h"

Glib::RefPtr<Glib::MainContext> hvigtk_threadcontext()
{
	return Glib::MainContext::get_thread_default();
}

ObjectThread::ObjectThread() :
	mutex(),
	objCreated(),
	loop(),
	thread(Glib::Threads::Thread::create(sigc::mem_fun(*this, &ObjectThread::slot) ) )
{
	mutex.lock();
	objCreated.wait(mutex);
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
	objCreated.signal();
	mutex.unlock();
	loop->run();
}
