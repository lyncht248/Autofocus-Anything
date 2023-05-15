#include "sdlwindow.hpp"
#include "sdlwinchild.hpp"

#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <iostream>
using namespace SDLWindow;

//only meaningful in child process
SDLWin *SDLWindow::sdlwin = nullptr;

SDLWin* SDLWindow::sdlwin_open()
{
	//Fork a process and use shared memory because the SDL renderer doesn't like running on any thread other than main, and this process already has GTK on the main thread.
	// This spawns sdlwinchild, which runs on a process from child_main()
	SDLWin *out = (struct SDLWin *) mmap(NULL, sizeof(struct SDLWin), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	if (out == MAP_FAILED)
	{
		return NULL;
	}
	memset(out, 0, sizeof(struct SDLWin) );

	pthread_mutexattr_t mattr;
	pthread_mutexattr_init(&mattr);
	pthread_mutexattr_setpshared(&mattr, 1);

	pthread_condattr_t cattr;
	pthread_condattr_init(&cattr);
	pthread_condattr_setpshared(&cattr, 1);

	if (pthread_mutex_init(&out->mutex, &mattr) )
	{
		munmap(out, sizeof(struct SDLWin) );
		return NULL;
	}
	if (pthread_cond_init(&out->hasCommand, &cattr) )
	{
		pthread_mutex_destroy(&out->mutex);
		munmap(out, sizeof(struct SDLWin) );
		return NULL;
	}
	if (pthread_cond_init(&out->hasResponse, &cattr) )
	{
		pthread_cond_destroy(&out->hasCommand);
		pthread_mutex_destroy(&out->mutex);
		munmap(out, sizeof(struct SDLWin) );
		return NULL;
	}

	out->command = CMD_NONE;
	out->response = RE_NONE;

	//Does the fork here. PID is process ID
	pid_t cpid = fork();
	if (cpid > 0)
	{
		out->pid = cpid;
		return out;
	}
	else if (cpid == 0)
	{
		sdlwin = out;
		exit(child_main() );
	}
	else
	{
		pthread_cond_destroy(&out->hasCommand);
		pthread_mutex_destroy(&out->mutex);
		munmap(out, sizeof(struct SDLWin) );
		return NULL;
	}
}

void SDLWindow::sdlwin_close(SDLWin* win)
{
	kill(win->pid, SIGINT);
	waitpid(win->pid, NULL, 0);
	pthread_cond_destroy(&win->hasResponse);
	pthread_cond_destroy(&win->hasCommand);
	pthread_mutex_destroy(&win->mutex);
	munmap(win, sizeof(struct SDLWin) );
	std::cout << "SDLWin closed" << std::endl;
}

Response SDLWindow::createFrame(SDLWin *win, int width, int height)
{
	lockcmd(win);
	win->raster.w = width;
	win->raster.h = height;
	
	win->command = CMD_CREATE_FRAME;

	Response out = waitForResponse(win);
	unlockcmd(win);

	return out;
}

Response SDLWindow::renderFrameG8(SDLWin *win, const void *buf, size_t n, int xoff, int yoff)
{
	lockcmd(win);
	if (buf)
		memcpy(win->bcmd, buf, n);
	win->raster.x += xoff;
	win->raster.y += yoff;

	win->command = CMD_RENDER_G8;

	Response out = waitForResponse(win);
	unlockcmd(win);

	return out;
}

void SDLWindow::setRaster(SDLWin *win, int x, int y)
{
	win->raster.x = x;
	win->raster.y = y;
}

unsigned long long SDLWindow::upload(SDLWin *win, const void *buf, size_t n, bool lock, int i, unsigned long long off)
{
	if (lock)
		pthread_mutex_lock(&win->mutex);
	if (i >= 0)
		memcpy(win->buser, ( (uint8_t *) buf) + off, n);
	else
		memcpy(win->bcmd, ( (uint8_t *) buf) + off, n);
	if (lock)
		pthread_mutex_unlock(&win->mutex);
	return off + n;
}

Response SDLWindow::updateMap(SDLWin *win, const void *buf, int width, int height, int pitch)
{
	lockcmd(win);
	upload(win, buf, height*pitch, false, -1);
	win->lcmd[0].d_int = width;
	win->lcmd[1].d_int = height;
	win->lcmd[2].d_int = pitch;
	win->command = CMD_UPDATE_MAP;
	Response out = waitForResponse(win);
	unlockcmd(win);
	return out;
}

void SDLWindow::setShowingMap(SDLWin *win, bool value)
{
	pthread_mutex_lock(&win->mutex);
	win->luser[0].d_bool = value;
	pthread_mutex_unlock(&win->mutex);
}

void SDLWindow::raise(SDLWin *win)
{
	lockcmd(win);
	win->command = CMD_RAISE;
	unlockcmd(win);
}

void SDLWindow::unraise(SDLWin *win)
{
	lockcmd(win);
	win->command = CMD_UNRAISE;
	unlockcmd(win);
}

void SDLWindow::hide(SDLWin *win)
{
	lockcmd(win);
	win->command = CMD_HIDE;
	unlockcmd(win);
}

void SDLWindow::move(SDLWin *win, int x, int y)
{
	lockcmd(win);
	win->lcmd[0].d_int = x;
	win->lcmd[1].d_int = y;
	win->command = CMD_MOVE;
	unlockcmd(win);
}
