#ifndef SDLWINDOW_H
#define SDLWINDOW_H

#include <SDL.h>
#include <pthread.h>
#include "sdlwincommon.hpp"

namespace SDLWindow
{
	static inline void lockcmd(SDLWin *sdlwin)
	{
		pthread_mutex_lock(&sdlwin->mutex);
		while (sdlwin->command)
			pthread_cond_wait(&sdlwin->hasResponse, &sdlwin->mutex);
	}

	static inline void unlockcmd(SDLWin *sdlwin)
	{
		pthread_mutex_unlock(&sdlwin->mutex);
	}

	static inline Response waitForResponse(SDLWin *sdlwin)
	{
		sdlwin->response = RE_NONE;
		while (!sdlwin->response)
			pthread_cond_wait(&sdlwin->hasResponse, &sdlwin->mutex);
		return sdlwin->response;
	}

	extern SDLWin* sdlwin_open();
	extern void sdlwin_close(SDLWin* win);

	extern Response createFrame(SDLWin *win, int width, int height);
	extern Response renderFrameG8(SDLWin *win, const void *buf = nullptr, size_t n=0, int xoff=0, int yoff=0);
	extern void setRaster(SDLWin *win, int x=0, int y=0);

	extern unsigned long long upload(SDLWin *win, const void *buf, size_t n, bool lock = true, int i = 0, unsigned long long off=0);
	extern Response updateMap(SDLWin *win, const void *buf, int width, int height, int pitch);
	extern void setShowingMap(SDLWin *win, bool value);
	extern void resetZoom(SDLWin *win);

	extern void raise(SDLWin *win);
	extern void unraise(SDLWin *win);
	extern void hide(SDLWin *win);
	extern void move(SDLWin *win, int x, int y);

	extern void quit(SDLWin *win);
}

#endif
