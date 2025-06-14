#ifndef SDLWINCOMMON_H
#define SDLWINCOMMON_H

#include <stdint.h>
#include <pthread.h>

#define UPLSZ 0x80000  //512 kB
#define CMD_BUF_SZ 0x10000 //64 kB

namespace SDLWindow
{
	enum Command
	{
		CMD_NONE,
		CMD_QUIT,
		CMD_CREATE_FRAME,
		CMD_RENDER_FRAME_G8,
		CMD_UPDATE_MAP,
		CMD_RAISE,
		CMD_UNRAISE,
		CMD_HIDE,
		CMD_MOVE,
		CMD_RESET_ZOOM
	};

	enum Response
	{
		RE_NONE,
		RE_OK,
		RE_FAIL,
	};

	union Value
	{
		double d_double;
		int d_int;
		bool d_bool;
	};

	struct Raster
	{
		int x, y, w, h;
	};

	struct SDLWin
	{
		pid_t pid;
		Command command;
		Response response;

		pthread_mutex_t mutex;
		pthread_cond_t hasCommand, hasResponse;

		uint8_t buser[UPLSZ];
		uint8_t bcmd[CMD_BUF_SZ];

		Value luser[100];
		Value lcmd[100];

		struct Raster raster;

		double zoomFactor;
		double zoomOffsetX;
		double zoomOffsetY;
	};
}

#endif 