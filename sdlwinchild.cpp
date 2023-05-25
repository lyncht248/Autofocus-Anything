#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <iostream>

#include "SDL_render.h"
#include "SDL_video.h"
#include "sdlwinchild.hpp"
#include "main.hpp"

#define BORDER_THRES 9

enum
{
	BORDER_TOP,
	BORDER_BOTTOM,
	BORDER_LEFT,
	BORDER_RIGHT
};

SDL_Rect borders[4] = { 
	{0,0,1,BORDER_THRES}, 
	{0,1,1,BORDER_THRES}, 
	{0,0,BORDER_THRES,1},
	{1,0,BORDER_THRES,1}
};

using namespace SDLWindow;

void convert_g8_to_rgb888(uint32_t *dst, const uint8_t *src, ssize_t n)
{
	for (; n >= 0; n--)
	{
		uint32_t val = src[n];
		val |= (val << 8) | (val << 16) | (val << 24);
		dst[n] = val;
	}
}

bool running = true;

void interrupt(int signo, siginfo_t *info, void *context)
{
	running = false;
}

static inline void redraw(SDL_Renderer *renderer, SDL_Texture *frame, SDL_Texture *map, int w, int h)
{
	if (frame)
	{
		//scale and buffer frame
		double xscale = ( (double) w - BORDER_THRES*2) / sdlwin->raster.w;
		double yscale = ( (double) h - BORDER_THRES*2) / sdlwin->raster.h;

		SDL_Rect dstrect;
		double centerx;
		{
			double arscale = xscale > yscale ? yscale : xscale;
			double swidth = arscale * sdlwin->raster.w;
			centerx = xscale > yscale ? ( ( (w-BORDER_THRES*2) - swidth) / 2.0) : 0.0;
			dstrect = (SDL_Rect) { 
				(int) (arscale * sdlwin->raster.x + centerx) + BORDER_THRES, 
				(int) (arscale * sdlwin->raster.y) + BORDER_THRES, 
				(int) swidth, 
				(int) (arscale * sdlwin->raster.h)};
		}

		SDL_RenderCopy(renderer, frame, NULL, &dstrect);
		bool showingMap = sdlwin->luser[0].d_bool;
		if (showingMap && map)
		{
			dstrect.x = centerx + BORDER_THRES;
			dstrect.y = BORDER_THRES;
			SDL_RenderCopy(renderer, map, NULL, &dstrect);
		}
	}

	//draw borders
	SDL_SetRenderDrawColor(renderer, 0x15, 0x18, 0x1A, 0xFF);
	borders[BORDER_TOP].w = w;
	borders[BORDER_BOTTOM].w = w;
	borders[BORDER_BOTTOM].y = h - BORDER_THRES;
	borders[BORDER_LEFT].h = h;
	borders[BORDER_RIGHT].h = h;
	borders[BORDER_RIGHT].x = w - BORDER_THRES;
	SDL_RenderFillRects(renderer, borders, 4);

	/*
	thickLineColor(renderer, 0, 0, w, 0, BORDER_THRES, 0xFF15181A);
	thickLineColor(renderer, 0, h, w, h, BORDER_THRES, 0xFF15181A);
	thickLineColor(renderer, 0, 0, 0, h, BORDER_THRES, 0xFF15181A);
	thickLineColor(renderer, w, 0, w, h, BORDER_THRES, 0xFF15181A);
	*/
}

static SDL_HitTestResult hit_test(SDL_Window *window, const SDL_Point *pt, void *data)
{
	int w, h;
	SDL_GetWindowSize(window, &w, &h);
	if (pt->x < BORDER_THRES)
	{
		if (pt->y < BORDER_THRES)
			return SDL_HITTEST_RESIZE_TOPLEFT;
		else if (h - pt->y < BORDER_THRES)
			return SDL_HITTEST_RESIZE_BOTTOMLEFT;
		else
			return SDL_HITTEST_RESIZE_LEFT;
	}
	else if (w - pt->x < BORDER_THRES)
	{
		if (pt->y < BORDER_THRES)
			return SDL_HITTEST_RESIZE_TOPRIGHT;
		else if (h - pt->y < BORDER_THRES)
			return SDL_HITTEST_RESIZE_BOTTOMRIGHT;
		else
			return SDL_HITTEST_RESIZE_RIGHT;
	}
	else if (pt->y < BORDER_THRES)
		return SDL_HITTEST_RESIZE_TOP;
	else if (h - pt->y < BORDER_THRES)
		return SDL_HITTEST_RESIZE_BOTTOM;

	return SDL_HITTEST_DRAGGABLE;
}

int SDLWindow::child_main()
{
	struct sigaction act = {0};
	act.sa_flags = SA_SIGINFO;
	act.sa_sigaction = &interrupt;
	sigaction(SIGINT, &act, NULL);

	if (SDL_Init(SDL_INIT_VIDEO) )
		return 1;

	int scalexyz = 2;
	// SDL_Window *window = SDL_CreateWindow("sdlwindow", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 800 * scalexyz, 600 * scalexyz, 
	// 		SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_BORDERLESS | SDL_WINDOW_UTILITY | SDL_WINDOW_SKIP_TASKBAR | SDL_WINDOW_ALLOW_HIGHDPI);
	// SDL_Window *window = SDL_CreateWindow("sdlwindow", 0, 0, 900 * scalexyz, 500 * scalexyz, 
	// 	SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_BORDERLESS  | SDL_WINDOW_SKIP_TASKBAR | SDL_WINDOW_ALLOW_HIGHDPI);
	SDL_Window *window = SDL_CreateWindow("sdlwindow", gtkAppLocationX, gtkAppLocationY+205, 918 * scalexyz, 442 * scalexyz, SDL_WINDOW_RESIZABLE | SDL_WINDOW_BORDERLESS | SDL_WINDOW_ALLOW_HIGHDPI);

	if (!window)
		return 2;
	
	//SDL_SetWindowPosition(window, 3000, 3000); 

	SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "best");

	SDL_SetWindowHitTest(window, &hit_test, NULL);
	
	SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
	if (!renderer)
	{
		SDL_DestroyWindow(window);
		return 3;
	}

	SDL_Texture *frame = NULL, *map = NULL;

	sdlwin->raster = (SDL_Rect) {0,0,0,0};

	SDL_Event e;
	while(running)
	{
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0xFF);
		SDL_RenderClear(renderer);
		if (sdlwin->command)
		{
			pthread_mutex_lock(&sdlwin->mutex);
			switch(sdlwin->command)
			{
				case CMD_DRAW_LINES:
				{
					SDL_RenderDrawLines(renderer, (SDL_Point *) sdlwin->bcmd, sdlwin->lcmd[0].d_int);
					break;
				}
				case CMD_CREATE_FRAME:
				{
					if (frame)
						SDL_DestroyTexture(frame);
 
					frame = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB888, SDL_TEXTUREACCESS_STREAMING, sdlwin->raster.w, sdlwin->raster.h);
					if (frame)
						sdlwin->response = RE_OK;
					else
						sdlwin->response = RE_EFRAME;
					break;
				}
				case CMD_RENDER_G8:
				{
					if (frame)
					{
						void *pixels;
						int pitch;
						if (!SDL_LockTexture(frame, NULL, &pixels, &pitch) )
						{
							convert_g8_to_rgb888( (uint32_t *) pixels, sdlwin->bcmd, sdlwin->raster.w * sdlwin->raster.h);
							SDL_UnlockTexture(frame);
							sdlwin->response = RE_OK;
						} else sdlwin->response = RE_EFRAME;
					} else sdlwin->response = RE_EFRAME;
					break;
				}
				case CMD_UPDATE_MAP:
				{
					if (map)
						SDL_DestroyTexture(map);
					int w = sdlwin->lcmd[0].d_int;
					int h = sdlwin->lcmd[1].d_int;
					int p = sdlwin->lcmd[2].d_int;
					map = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, w, h);
					if (map)
					{
						SDL_SetTextureBlendMode(map, SDL_BLENDMODE_BLEND);
						void *pixels;
						int pitch;
						if (!SDL_LockTexture(map, NULL, &pixels, &pitch) )
						{
							memcpy(pixels, sdlwin->bcmd, h * pitch);
							SDL_UnlockTexture(map);
							sdlwin->response = RE_OK;
						} else sdlwin->response = RE_EFRAME;
					}
					else
						sdlwin->response = RE_EFRAME;
					break;
				}
				case CMD_RAISE:
				{
					SDL_ShowWindow(window);
					SDL_SetWindowAlwaysOnTop(window, SDL_TRUE);
					SDL_Delay(10);
					break;
				}
				case CMD_UNRAISE:
				{
					SDL_SetWindowAlwaysOnTop(window, SDL_FALSE);
					SDL_Delay(10);
					break;
				}
				case CMD_HIDE:
				{
					SDL_HideWindow(window);
					SDL_Delay(10);
					break;
				}
				case CMD_MOVE:
				{
					int x = sdlwin->lcmd[0].d_int;
					int y = sdlwin->lcmd[1].d_int;
					//Causes window to always appear in centerof screen... test removing for now
					//SDL_SetWindowPosition(window, x, y);
					break;
				}
				case CMD_QUIT:
				{
					sdlwin->command = CMD_NONE;
					pthread_cond_broadcast(&sdlwin->hasResponse);
					pthread_mutex_unlock(&sdlwin->mutex);
					goto quit;
				}
				default:
					break;
			}
			sdlwin->command = CMD_NONE;
			pthread_cond_broadcast(&sdlwin->hasResponse);
			pthread_mutex_unlock(&sdlwin->mutex);
		}

		while( SDL_PollEvent( &e ) != 0 )
		{
		}
		
		if (SDL_GetWindowFlags(window) & SDL_WINDOW_SHOWN)
		{
			int w, h;
			SDL_GetRendererOutputSize(renderer, &w, &h);
			redraw(renderer, frame, map, w, h);
			SDL_RenderPresent(renderer);
			SDL_Delay(1);
		}
		else
			SDL_Delay(100);
	}
	quit:;
	if (frame)
		SDL_DestroyTexture(frame);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
	
	return 0;
}
