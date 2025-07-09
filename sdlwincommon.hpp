#ifndef SDLWINCOMMON_H
#define SDLWINCOMMON_H

#include <SDL.h>
#include <pthread.h>

#define MDATASZ 1024
#define BDATASZ 10485760

namespace SDLWindow
{
	typedef union
	{
		bool d_bool;
		char d_char;
		unsigned char d_uchar;
		int d_int;
		unsigned int d_uint;
		long d_long;
		unsigned long d_ulong;
		float d_float;
		double d_double;
	} ldata_t;
	typedef unsigned char mdata_t[MDATASZ];
	typedef unsigned char bdata_t[BDATASZ];

	enum Command
	{
		CMD_NONE = 0,
		CMD_DRAW_LINES,
		CMD_CREATE_FRAME,
		CMD_RENDER_G8,
		CMD_UPDATE_MAP,
		CMD_RENDER_MAP,
		CMD_RAISE,
		CMD_UNRAISE,
		CMD_HIDE,
		CMD_MOVE,
		CMD_QUIT
	};

	enum Response
	{
		RE_NONE = 0,
		RE_EFRAME,
		RE_ERROR,
		RE_OK
	};

	struct SDLWin
	{
		int pid;
		pthread_mutex_t mutex;
		pthread_cond_t hasCommand, hasResponse;
		Command command;
		Response response;
		SDL_Rect raster;
		ldata_t lcmd[8], lre[8], luser[16];
		mdata_t mcmd[3], mre[3], muser[4];
		bdata_t bcmd, buser;
	};
}

#endif
