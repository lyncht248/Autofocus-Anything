#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <iostream>
#include "logfile.hpp"
#include "SDL_render.h"
#include "SDL_video.h"
#include "sdlwinchild.hpp"
#include "main.hpp"
#include <SDL_ttf.h>
#include <SDL.h>

#define BORDER_THRES 9

enum
{
	BORDER_TOP,
	BORDER_BOTTOM,
	BORDER_LEFT,
	BORDER_RIGHT
};

SDL_Rect borders[4] = {
	{0, 0, 1, BORDER_THRES},
	{0, 1, 1, BORDER_THRES},
	{0, 0, BORDER_THRES, 1},
	{1, 0, BORDER_THRES, 1}};

using namespace SDLWindow;

TTF_Font *font = nullptr;

// Track mouse dragging for image panning and window dragging
bool isDraggingImage = false;
bool isDraggingWindow = false;
int dragStartX = 0;
int dragStartY = 0;

SDL_Cursor *handCursor = nullptr;
SDL_Cursor *defaultCursor = nullptr;

static bool debug_print = true;
static int counter = 0;

// Add near the top with other static variables
static Uint32 lastClickTime = 0;
static int lastClickX = 0, lastClickY = 0;

// Forward declarations
static void handleDoubleClick(int x, int y);
static void drawROIOverlay(SDL_Renderer *renderer);
static void handleDoubleClickWithRenderer(int x, int y, SDL_Renderer *renderer);

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

// Add this function after the existing functions
static void handleDoubleClick(int x, int y)
{
	if (!sdlwin)
		return;

	// We need to get the renderer to do proper coordinate conversion
	// For now, store screen coordinates and let the main process handle conversion
	pthread_mutex_lock(&sdlwin->mutex);
	sdlwin->lcmd[10].d_int = x; // Store screen coordinates temporarily
	sdlwin->lcmd[11].d_int = y;
	sdlwin->lcmd[12].d_bool = true; // Flag that we have a new ROI center
	pthread_mutex_unlock(&sdlwin->mutex);

	printf("[SDL Child] Double-click detected at screen (%d, %d)\n", x, y);
}

// Add this function to handle double-click with proper coordinate conversion
static void handleDoubleClickWithRenderer(int x, int y, SDL_Renderer *renderer)
{
	if (!sdlwin || !renderer)
		return;

	// Get current window dimensions from renderer
	int windowWidth, windowHeight;
	SDL_GetRendererOutputSize(renderer, &windowWidth, &windowHeight);

	// Use the same coordinate transformation logic as in the mouse wheel handler
	double xscale = ((double)windowWidth - BORDER_THRES * 2) / sdlwin->raster.w;
	double yscale = ((double)windowHeight - BORDER_THRES * 2) / sdlwin->raster.h;
	double arscale = xscale > yscale ? yscale : xscale;

	// Apply zoom factor to the scaling
	double zoomedScale = arscale * sdlwin->zoomFactor;
	double swidth = zoomedScale * sdlwin->raster.w;
	double sheight = zoomedScale * sdlwin->raster.h;

	// Calculate center position (same as in redraw function)
	double centerX = (windowWidth - swidth) / 2.0;
	double centerY = (windowHeight - sheight) / 2.0;

	// Make sure we don't go beyond borders
	if (centerX < BORDER_THRES)
		centerX = BORDER_THRES;
	if (centerY < BORDER_THRES)
		centerY = BORDER_THRES;

	// Account for raster position and zoom offsets (same as in redraw function)
	double adjustedCenterX = centerX + zoomedScale * (sdlwin->raster.x + sdlwin->zoomOffsetX);
	double adjustedCenterY = centerY + zoomedScale * (sdlwin->raster.y + sdlwin->zoomOffsetY);

	// Convert screen coordinates to image coordinates
	// This reverses the transformation done in redraw function
	double imageX = (x - adjustedCenterX) / zoomedScale;
	double imageY = (y - adjustedCenterY) / zoomedScale;

	// Clamp to image bounds
	imageX = std::max(0.0, std::min(imageX, (double)(sdlwin->raster.w - 1)));
	imageY = std::max(0.0, std::min(imageY, (double)(sdlwin->raster.h - 1)));

	// Store the converted image coordinates in shared memory for the main process
	pthread_mutex_lock(&sdlwin->mutex);
	sdlwin->lcmd[10].d_int = (int)imageX; // Store converted image coordinates
	sdlwin->lcmd[11].d_int = (int)imageY;
	sdlwin->lcmd[12].d_bool = true; // Flag that we have a new ROI center
	pthread_mutex_unlock(&sdlwin->mutex);

	printf("[SDL Child] Double-click at screen (%d, %d) -> image (%d, %d)\n",
		   x, y, (int)imageX, (int)imageY);
}

// Add this function to draw the ROI overlay
static void drawROIOverlay(SDL_Renderer *renderer)
{
	if (!sdlwin || !sdlwin->showROI)
	{
		return;
	}

	pthread_mutex_lock(&sdlwin->mutex);
	int imageCenterX = sdlwin->roiCenterX; // These are in image coordinates
	int imageCenterY = sdlwin->roiCenterY;
	int imageWidth = sdlwin->roiWidth;
	int imageHeight = sdlwin->roiHeight;
	bool showROI = sdlwin->showROI;
	bool searchSuccessful = sdlwin->searchComplete;
	pthread_mutex_unlock(&sdlwin->mutex);

	if (!showROI || imageCenterX < 0 || imageCenterY < 0)
	{
		return;
	}

	// Convert image coordinates to screen coordinates using the same logic as redraw function
	int windowWidth, windowHeight;
	SDL_GetRendererOutputSize(renderer, &windowWidth, &windowHeight);

	// Calculate scaling factors (same as in redraw function)
	double xscale = ((double)windowWidth - BORDER_THRES * 2) / sdlwin->raster.w;
	double yscale = ((double)windowHeight - BORDER_THRES * 2) / sdlwin->raster.h;
	double arscale = xscale > yscale ? yscale : xscale;

	// Apply zoom factor to the scaling
	double zoomedScale = arscale * sdlwin->zoomFactor;
	double swidth = zoomedScale * sdlwin->raster.w;
	double sheight = zoomedScale * sdlwin->raster.h;

	// Calculate center position (same as in redraw function)
	double centerX = (windowWidth - swidth) / 2.0;
	double centerY = (windowHeight - sheight) / 2.0;

	// Make sure we don't go beyond borders
	if (centerX < BORDER_THRES)
		centerX = BORDER_THRES;
	if (centerY < BORDER_THRES)
		centerY = BORDER_THRES;

	// Account for raster position and zoom offsets (same as in redraw function)
	double adjustedCenterX = centerX + zoomedScale * (sdlwin->raster.x + sdlwin->zoomOffsetX);
	double adjustedCenterY = centerY + zoomedScale * (sdlwin->raster.y + sdlwin->zoomOffsetY);

	// Convert image coordinates to screen coordinates
	int screenCenterX = (int)(adjustedCenterX + imageCenterX * zoomedScale);
	int screenCenterY = (int)(adjustedCenterY + imageCenterY * zoomedScale);
	int screenWidth = (int)(imageWidth * zoomedScale);
	int screenHeight = (int)(imageHeight * zoomedScale);

	// Calculate ROI rectangle in screen coordinates
	int roiX = screenCenterX - screenWidth / 2;
	int roiY = screenCenterY - screenHeight / 2;

	// Choose color based on search success: red for successful, yellow for normal
	if (searchSuccessful)
	{
		SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // Red color - optimal focus found
	}
	else
	{
		SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255); // Yellow color - normal ROI
	}

	SDL_Rect roiRect = {roiX, roiY, screenWidth, screenHeight};
	SDL_RenderDrawRect(renderer, &roiRect);

	// Draw thicker border by drawing multiple rectangles
	for (int i = 1; i < 3; i++)
	{
		SDL_Rect thickRect = {roiX - i, roiY - i, screenWidth + 2 * i, screenHeight + 2 * i};
		SDL_RenderDrawRect(renderer, &thickRect);
	}

	// Draw center crosshair
	SDL_RenderDrawLine(renderer, screenCenterX - 5, screenCenterY, screenCenterX + 5, screenCenterY);
	SDL_RenderDrawLine(renderer, screenCenterX, screenCenterY - 5, screenCenterX, screenCenterY + 5);
}

static inline void redraw(SDL_Renderer *renderer, SDL_Texture *frame, SDL_Texture *map, int w, int h)
{
	if (frame)
	{
		// scale and buffer frame
		double xscale = ((double)w - BORDER_THRES * 2) / sdlwin->raster.w;
		double yscale = ((double)h - BORDER_THRES * 2) / sdlwin->raster.h;

		SDL_Rect dstrect;
		double centerx;
		double centery;
		{
			double arscale = xscale > yscale ? yscale : xscale;

			// Apply zoom factor to the scaling
			double zoomedScale = arscale * sdlwin->zoomFactor;
			double swidth = zoomedScale * sdlwin->raster.w;
			double sheight = zoomedScale * sdlwin->raster.h;

			// Calculate center position to keep zoom centered
			centerx = (w - swidth) / 2.0;
			centery = (h - sheight) / 2.0;

			// Make sure we don't go beyond borders
			if (centerx < BORDER_THRES)
				centerx = BORDER_THRES;
			if (centery < BORDER_THRES)
				centery = BORDER_THRES;

			dstrect = (SDL_Rect){
				(int)(centerx + zoomedScale * (sdlwin->raster.x + sdlwin->zoomOffsetX)),
				(int)(centery + zoomedScale * (sdlwin->raster.y + sdlwin->zoomOffsetY)),
				(int)swidth,
				(int)sheight};
		}

		SDL_RenderCopy(renderer, frame, NULL, &dstrect);
		bool showingMap = sdlwin->luser[0].d_bool;
		if (showingMap && map)
		{
			// Apply the same zoom to the map if it's showing
			dstrect.x = centerx + BORDER_THRES;
			dstrect.y = BORDER_THRES;
			SDL_RenderCopy(renderer, map, NULL, &dstrect);
		}

		// After rendering the frame in redraw function, add zoom indicator
		if (sdlwin->zoomFactor > 1.0)
		{
			// Requires SDL_ttf to be initialized
			if (font) // Make sure you have a TTF_Font* font initialized
			{
				char zoomText[32];
				sprintf(zoomText, "Zoom: %.1fx", sdlwin->zoomFactor);
				SDL_Color textColor = {255, 255, 255, 255};
				SDL_Surface *textSurface = TTF_RenderText_Solid(font, zoomText, textColor);
				SDL_Texture *textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);

				SDL_Rect textRect = {10, 10, textSurface->w, textSurface->h};
				SDL_RenderCopy(renderer, textTexture, NULL, &textRect);

				SDL_FreeSurface(textSurface);
				SDL_DestroyTexture(textTexture);
			}
		}
	}

	// draw borders
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

	// Add ROI overlay at the end, just before returning
	drawROIOverlay(renderer);
}

static SDL_HitTestResult hit_test(SDL_Window *window, const SDL_Point *pt, void *data)
{
	int w, h;
	SDL_GetWindowSize(window, &w, &h);

	// Check for border resize areas first
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

	// For the main area, return NORMAL to allow our mouse events to be processed
	// We'll handle window dragging manually when not zoomed in
	return SDL_HITTEST_NORMAL;
}

int SDLWindow::child_main()
{
	struct sigaction act = {0};
	act.sa_flags = SA_SIGINFO;
	act.sa_sigaction = &interrupt;
	sigaction(SIGINT, &act, NULL);

	if (SDL_Init(SDL_INIT_VIDEO))
		return 1;

	if (TTF_Init() < 0)
	{
		return 1;
	}

	// Load font - replace path with your actual font file location
	font = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16);
	if (!font)
	{
		printf("TTF_OpenFont error: %s\n", TTF_GetError());
		// Consider whether to return an error or continue without text rendering
	}

	int scalexyz = 2;
	// SDL_Window *window = SDL_CreateWindow("sdlwindow", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 800 * scalexyz, 600 * scalexyz,
	// 		SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_BORDERLESS | SDL_WINDOW_UTILITY | SDL_WINDOW_SKIP_TASKBAR | SDL_WINDOW_ALLOW_HIGHDPI);
	// SDL_Window *window = SDL_CreateWindow("sdlwindow", 0, 0, 900 * scalexyz, 500 * scalexyz,
	// 	SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_BORDERLESS  | SDL_WINDOW_SKIP_TASKBAR | SDL_WINDOW_ALLOW_HIGHDPI);
	SDL_Window *window = SDL_CreateWindow("sdlwindow", gtkAppLocationX, gtkAppLocationY + 205, 913 * scalexyz, 442 * scalexyz, SDL_WINDOW_RESIZABLE | SDL_WINDOW_BORDERLESS | SDL_WINDOW_ALLOW_HIGHDPI);

	if (!window)
		return 2;

	// SDL_SetWindowPosition(window, 3000, 3000);

	SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "best");

	SDL_SetWindowHitTest(window, &hit_test, NULL);

	SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
	if (!renderer)
	{
		SDL_DestroyWindow(window);
		return 3;
	}

	SDL_Texture *frame = NULL, *map = NULL;

	sdlwin->raster = (SDL_Rect){0, 0, 0, 0};

	// Initialize cursors
	defaultCursor = SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_ARROW);
	handCursor = SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_HAND);

	SDL_Event e;
	while (running)
	{
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0xFF);
		SDL_RenderClear(renderer);
		if (sdlwin->command)
		{
			pthread_mutex_lock(&sdlwin->mutex);
			switch (sdlwin->command)
			{
			case CMD_DRAW_LINES:
			{
				SDL_RenderDrawLines(renderer, (SDL_Point *)sdlwin->bcmd, sdlwin->lcmd[0].d_int);
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
					if (!SDL_LockTexture(frame, NULL, &pixels, &pitch))
					{
						convert_g8_to_rgb888((uint32_t *)pixels, sdlwin->bcmd, sdlwin->raster.w * sdlwin->raster.h);
						SDL_UnlockTexture(frame);
						sdlwin->response = RE_OK;
					}
					else
						sdlwin->response = RE_EFRAME;
				}
				else
					sdlwin->response = RE_EFRAME;
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
					if (!SDL_LockTexture(map, NULL, &pixels, &pitch))
					{
						memcpy(pixels, sdlwin->bcmd, h * pitch);
						SDL_UnlockTexture(map);
						sdlwin->response = RE_OK;
					}
					else
						sdlwin->response = RE_EFRAME;
				}
				else
					sdlwin->response = RE_EFRAME;
				break;
			}
			case CMD_RAISE:
			{
				SDL_ShowWindow(window);
				// SDL_SetWindowAlwaysOnTop(window, SDL_TRUE);
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
				// Causes window to always appear in centerof screen... test removing for now
				// SDL_SetWindowPosition(window, x, y);
				break;
			}
			case CMD_QUIT:
			{
				sdlwin->command = CMD_NONE;
				pthread_cond_broadcast(&sdlwin->hasResponse);
				pthread_mutex_unlock(&sdlwin->mutex);
				goto quit;
			}
			case CMD_RESET_ZOOM:
			{
				sdlwin->zoomFactor = 1.0;  // Reset zoom
				sdlwin->zoomOffsetX = 0.0; // Reset zoom offsets
				sdlwin->zoomOffsetY = 0.0;
				sdlwin->response = RE_OK;
				break;
			}
			default:
				break;
			}
			sdlwin->command = CMD_NONE;
			pthread_cond_broadcast(&sdlwin->hasResponse);
			pthread_mutex_unlock(&sdlwin->mutex);
		}

		while (SDL_PollEvent(&e) != 0)
		{
			if (e.type == SDL_MOUSEWHEEL)
			{
				// Get mouse position
				int mouseX, mouseY;
				SDL_GetMouseState(&mouseX, &mouseY);

				// Get current window dimensions
				int windowWidth, windowHeight;
				SDL_GetRendererOutputSize(renderer, &windowWidth, &windowHeight);

				// Calculate scaling factors before zoom change
				double xscale = ((double)windowWidth - BORDER_THRES * 2) / sdlwin->raster.w;
				double yscale = ((double)windowHeight - BORDER_THRES * 2) / sdlwin->raster.h;
				double arscale = xscale > yscale ? yscale : xscale;
				double oldZoom = sdlwin->zoomFactor;

				// Calculate current visible region dimensions
				double currentVisibleWidth = arscale * oldZoom * sdlwin->raster.w;
				double currentVisibleHeight = arscale * oldZoom * sdlwin->raster.h;

				// Center position of the visible region
				double centerX = (windowWidth - currentVisibleWidth) / 2.0;
				if (centerX < BORDER_THRES)
					centerX = BORDER_THRES;
				double centerY = (windowHeight - currentVisibleHeight) / 2.0;
				if (centerY < BORDER_THRES)
					centerY = BORDER_THRES;

				// Calculate the true image coordinates under the mouse pointer
				// This is in the original image space coordinates (before any zoom)
				double imageX = (mouseX - centerX) / (arscale * oldZoom) - sdlwin->zoomOffsetX;
				double imageY = (mouseY - centerY) / (arscale * oldZoom) - sdlwin->zoomOffsetY;

				// Check for zoom out at minimum zoom level
				bool resetPosition = false;

				// Change zoom factor
				if (e.wheel.y > 0) // Scroll up (zoom in)
				{
					sdlwin->zoomFactor *= 1.1;	  // Increase zoom by 10%
					if (sdlwin->zoomFactor > 5.0) // Limit maximum zoom
						sdlwin->zoomFactor = 5.0;
				}
				else if (e.wheel.y < 0) // Scroll down (zoom out)
				{
					// Check if we're already at min zoom and trying to zoom out further
					if (sdlwin->zoomFactor <= 1.0)
					{
						resetPosition = true;	  // Signal to reset position
						sdlwin->zoomFactor = 1.0; // Keep zoom at minimum
					}
					else
					{
						sdlwin->zoomFactor /= 1.1;	  // Decrease zoom by 10%
						if (sdlwin->zoomFactor < 1.0) // Limit minimum zoom
							sdlwin->zoomFactor = 1.0;
					}
				}

				// If zoom changed or we need to reset position
				if (oldZoom != sdlwin->zoomFactor || resetPosition)
				{
					if (resetPosition)
					{
						// Reset position to center
						sdlwin->zoomOffsetX = 0.0;
						sdlwin->zoomOffsetY = 0.0;
					}
					else
					{
						// Calculate new visible dimensions after zoom
						double newVisibleWidth = arscale * sdlwin->zoomFactor * sdlwin->raster.w;
						double newVisibleHeight = arscale * sdlwin->zoomFactor * sdlwin->raster.h;

						// New center position
						double newCenterX = (windowWidth - newVisibleWidth) / 2.0;
						if (newCenterX < BORDER_THRES)
							newCenterX = BORDER_THRES;
						double newCenterY = (windowHeight - newVisibleHeight) / 2.0;
						if (newCenterY < BORDER_THRES)
							newCenterY = BORDER_THRES;

						// Calculate where the same image point would be displayed after zoom
						double newDisplayX = imageX * arscale * sdlwin->zoomFactor + newCenterX;
						double newDisplayY = imageY * arscale * sdlwin->zoomFactor + newCenterY;

						// Calculate the adjustment needed to keep mouse over the same point
						double deltaX = mouseX - newDisplayX;
						double deltaY = mouseY - newDisplayY;

						// Update zoom offsets - convert to image space coordinates
						sdlwin->zoomOffsetX = deltaX / (arscale * sdlwin->zoomFactor);
						sdlwin->zoomOffsetY = deltaY / (arscale * sdlwin->zoomFactor);
					}
				}
			}
			else if (e.type == SDL_MOUSEBUTTONDOWN)
			{
				if (e.button.button == SDL_BUTTON_MIDDLE) // Middle mouse button
				{
					sdlwin->zoomFactor = 1.0;  // Reset zoom
					sdlwin->zoomOffsetX = 0.0; // Reset zoom offsets
					sdlwin->zoomOffsetY = 0.0;
				}
				else if (e.button.button == SDL_BUTTON_LEFT) // Left mouse button
				{
					Uint32 currentTime = SDL_GetTicks();
					int currentX = e.button.x;
					int currentY = e.button.y;

					// Check for double-click (within 500ms and 10 pixels)
					if (currentTime - lastClickTime < 500 &&
						abs(currentX - lastClickX) < 10 &&
						abs(currentY - lastClickY) < 10)
					{

						handleDoubleClickWithRenderer(currentX, currentY, renderer);
					}
					else
					{
						// Start dragging based on zoom level
						if (sdlwin->zoomFactor > 1.0)
						{
							// When zoomed in, drag the image
							isDraggingImage = true;
							SDL_GetMouseState(&dragStartX, &dragStartY);
							SDL_SetCursor(handCursor); // Change to hand cursor
						}
						else
						{
							// When not zoomed in, drag the window
							isDraggingWindow = true;
							SDL_GetMouseState(&dragStartX, &dragStartY);
						}
					}

					lastClickTime = currentTime;
					lastClickX = currentX;
					lastClickY = currentY;
				}
			}
			else if (e.type == SDL_MOUSEBUTTONUP)
			{
				if (e.button.button == SDL_BUTTON_LEFT)
				{
					if (isDraggingImage)
					{
						SDL_SetCursor(defaultCursor); // Change back to default cursor
					}
					isDraggingImage = false;
					isDraggingWindow = false;
				}
			}
			else if (e.type == SDL_MOUSEMOTION)
			{
				if (isDraggingImage)
				{
					int currentX, currentY;
					SDL_GetMouseState(&currentX, &currentY);

					// Simple direct translation at higher zoom levels
					if (sdlwin->zoomFactor > 1.0)
					{
						// Calculate delta in screen pixels
						int deltaX = currentX - dragStartX;
						int deltaY = currentY - dragStartY;

						// Skip tiny movements
						if (abs(deltaX) < 2 && abs(deltaY) < 2)
						{
							dragStartX = currentX;
							dragStartY = currentY;
						}
						else
						{
							// Get the scale factor (how many screen pixels per image pixel)
							int windowWidth, windowHeight;
							SDL_GetRendererOutputSize(renderer, &windowWidth, &windowHeight);

							double xscale = ((double)windowWidth - BORDER_THRES * 2) / sdlwin->raster.w;
							double yscale = ((double)windowHeight - BORDER_THRES * 2) / sdlwin->raster.h;
							double scale = (xscale < yscale) ? xscale : yscale;
							scale *= sdlwin->zoomFactor;

							// Convert to image space (simpler approach)
							sdlwin->zoomOffsetX += (double)deltaX / scale;
							sdlwin->zoomOffsetY += (double)deltaY / scale;

							dragStartX = currentX;
							dragStartY = currentY;
						}
					}
				}
				else if (isDraggingWindow)
				{
					int currentX, currentY;
					SDL_GetMouseState(&currentX, &currentY);

					// Calculate delta for window movement
					int deltaX = currentX - dragStartX;
					int deltaY = currentY - dragStartY;

					// Move the window
					int windowX, windowY;
					SDL_GetWindowPosition(window, &windowX, &windowY);
					SDL_SetWindowPosition(window, windowX + deltaX, windowY + deltaY);

					// Don't update dragStart coordinates since window moved
				}
			}
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
	if (handCursor)
		SDL_FreeCursor(handCursor);
	if (defaultCursor)
		SDL_FreeCursor(defaultCursor);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();

	if (font)
	{
		TTF_CloseFont(font);
		font = nullptr;
	}

	TTF_Quit();

	return 0;
}
