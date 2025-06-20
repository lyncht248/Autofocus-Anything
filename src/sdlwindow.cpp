#include "sdlwindow.hpp"
#include "sdlwinchild.hpp"
#include "logfile.hpp"
#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <iostream>
#include <limits>
#include "system.hpp"

using namespace SDLWindow;

// only meaningful in child process
SDLWin *SDLWindow::sdlwin = nullptr;

SDLWin *SDLWindow::sdlwin_open()
{
	// Fork a process and use shared memory because the SDL renderer doesn't like running on any thread other than main, and this process already has GTK on the main thread.
	//  This spawns sdlwinchild, which runs on a process from child_main()
	SDLWin *out = (struct SDLWin *)mmap(NULL, sizeof(struct SDLWin), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	if (out == MAP_FAILED)
	{
		return NULL;
	}
	memset(out, 0, sizeof(struct SDLWin));

	pthread_mutexattr_t mattr;
	pthread_mutexattr_init(&mattr);
	pthread_mutexattr_setpshared(&mattr, 1);

	pthread_condattr_t cattr;
	pthread_condattr_init(&cattr);
	pthread_condattr_setpshared(&cattr, 1);

	if (pthread_mutex_init(&out->mutex, &mattr))
	{
		munmap(out, sizeof(struct SDLWin));
		return NULL;
	}
	if (pthread_cond_init(&out->hasCommand, &cattr))
	{
		pthread_mutex_destroy(&out->mutex);
		munmap(out, sizeof(struct SDLWin));
		return NULL;
	}
	if (pthread_cond_init(&out->hasResponse, &cattr))
	{
		pthread_cond_destroy(&out->hasCommand);
		pthread_mutex_destroy(&out->mutex);
		munmap(out, sizeof(struct SDLWin));
		return NULL;
	}

	out->command = CMD_NONE;
	out->response = RE_NONE;
	out->zoomFactor = 1.0;
	out->zoomOffsetX = 0.0;
	out->zoomOffsetY = 0.0;

	// Initialize ROI parameters
	out->system = nullptr;
	out->showROI = false;
	out->roiCenterX = -1;
	out->roiCenterY = -1;
	out->roiWidth = 150;
	out->roiHeight = 150;

	// Initialize stabilization parameters
	out->stabOffsetX = 0.0;
	out->stabOffsetY = 0.0;
	out->stabActive = false;

	// Initialize depth map parameters
	out->showDepthMap = false;
	out->hasDepthMap = false;
	out->depthMapWidth = 0;
	out->depthMapHeight = 0;
	out->depthMapReady = false;
	out->depthDataVersion = 0;
	out->focusMin = 190.0f; // Default values, will be updated with actual data
	out->focusMax = 450.0f;
	for (int y = 0; y < SDLWin::MAX_DEPTH_HEIGHT; y++)
	{
		for (int x = 0; x < SDLWin::MAX_DEPTH_WIDTH; x++)
		{
			out->focusPositions[y][x] = -1.0f; // Negative indicates no data
		}
	}

	// Does the fork here. PID is process ID
	pid_t cpid = fork();
	if (cpid > 0)
	{
		out->pid = cpid;
		return out;
	}
	else if (cpid == 0)
	{
		sdlwin = out;
		exit(child_main());
	}
	else
	{
		pthread_cond_destroy(&out->hasCommand);
		pthread_mutex_destroy(&out->mutex);
		munmap(out, sizeof(struct SDLWin));
		return NULL;
	}
}

void SDLWindow::sdlwin_close(SDLWin *win)
{
	kill(win->pid, SIGINT);
	waitpid(win->pid, NULL, 0);
	pthread_cond_destroy(&win->hasResponse);
	pthread_cond_destroy(&win->hasCommand);
	pthread_mutex_destroy(&win->mutex);
	munmap(win, sizeof(struct SDLWin));
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
		memcpy(win->buser, ((uint8_t *)buf) + off, n);
	else
		memcpy(win->bcmd, ((uint8_t *)buf) + off, n);
	if (lock)
		pthread_mutex_unlock(&win->mutex);
	return off + n;
}

Response SDLWindow::updateMap(SDLWin *win, const void *buf, int width, int height, int pitch)
{
	lockcmd(win);
	upload(win, buf, height * pitch, false, -1);
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

void SDLWindow::resetZoom(SDLWin *win)
{
	lockcmd(win);
	win->zoomFactor = 1.0;	// Reset zoom
	win->zoomOffsetX = 0.0; // Reset zoom offsets
	win->zoomOffsetY = 0.0;
	win->command = CMD_RESET_ZOOM;
	pthread_cond_broadcast(&win->hasCommand);
	waitForResponse(win);
	unlockcmd(win);
}

void handleDoubleClick(SDLWin *win, int x, int y)
{
	if (win && win->system)
	{
		// Convert screen coordinates to image coordinates
		// Account for any scaling/offset in the display
		int imageX = x; // You may need to adjust this based on your display scaling
		int imageY = y; // You may need to adjust this based on your display scaling

		// Update the ROI center in the system
		win->system->updateROICenter(imageX, imageY);

		// Update display parameters
		win->roiCenterX = imageX;
		win->roiCenterY = imageY;
		win->showROI = true;

		std::cout << "[SDL Window] Double-click at (" << imageX << ", " << imageY << "), ROI updated" << std::endl;
	}
}

void updateROIDisplay(SDLWin *win)
{
	if (win && win->system && win->system->getImagingCam())
	{
		// Get ROI info directly from imaging camera to avoid mutex deadlock
		int centerX, centerY, width, height;
		win->system->getImagingCam()->getCurrentROI(centerX, centerY, width, height);

		// Update SDL window parameters
		pthread_mutex_lock(&win->mutex);
		win->roiCenterX = centerX;
		win->roiCenterY = centerY;
		win->roiWidth = width;
		win->roiHeight = height;
		win->showROI = (centerX >= 0 && centerY >= 0);
		pthread_mutex_unlock(&win->mutex);
	}
}

// In your main SDL event loop, add double-click detection:
void handleSDLEvents(SDLWin *win)
{
	SDL_Event event;
	static Uint32 lastClickTime = 0;
	static int lastClickX = 0, lastClickY = 0;

	while (SDL_PollEvent(&event))
	{
		switch (event.type)
		{
		case SDL_QUIT:
			quit(win);
			break;

		case SDL_MOUSEBUTTONDOWN:
			if (event.button.button == SDL_BUTTON_LEFT)
			{
				Uint32 currentTime = SDL_GetTicks();
				int currentX = event.button.x;
				int currentY = event.button.y;

				// Check for double-click (within 500ms and 10 pixels)
				if (currentTime - lastClickTime < 500 &&
					abs(currentX - lastClickX) < 10 &&
					abs(currentY - lastClickY) < 10)
				{

					handleDoubleClick(win, currentX, currentY);
				}

				lastClickTime = currentTime;
				lastClickX = currentX;
				lastClickY = currentY;
			}
			break;

			// existing event handling...
		}
	}
}

// Add ROI overlay rendering to your frame drawing function:
void drawROIOverlay(SDLWin *win, SDL_Renderer *renderer)
{
	if (!win->showROI)
	{
		return;
	}

	// Update ROI parameters from system
	updateROIDisplay(win);

	if (win->roiCenterX >= 0 && win->roiCenterY >= 0)
	{
		// Calculate ROI rectangle
		int roiX = win->roiCenterX - win->roiWidth / 2;
		int roiY = win->roiCenterY - win->roiHeight / 2;

		// Draw ROI rectangle
		SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255); // Yellow color

		SDL_Rect roiRect = {roiX, roiY, win->roiWidth, win->roiHeight};
		SDL_RenderDrawRect(renderer, &roiRect);

		// Draw thicker border by drawing multiple rectangles
		for (int i = 1; i < 3; i++)
		{
			SDL_Rect thickRect = {roiX - i, roiY - i, win->roiWidth + 2 * i, win->roiHeight + 2 * i};
			SDL_RenderDrawRect(renderer, &thickRect);
		}

		// Draw center crosshair
		SDL_RenderDrawLine(renderer, win->roiCenterX - 5, win->roiCenterY, win->roiCenterX + 5, win->roiCenterY);
		SDL_RenderDrawLine(renderer, win->roiCenterX, win->roiCenterY - 5, win->roiCenterX, win->roiCenterY + 5);
	}
}

void SDLWindow::transferDepthMapData(SDLWin *win, const std::vector<std::vector<std::pair<double, double>>> &depthImage, int width, int height)
{
	std::cout << "[Debug] transferDepthMapData() called with " << width << "x" << height << " depth image" << std::endl;

	if (!win || width <= 0 || height <= 0)
		return;

	pthread_mutex_lock(&win->mutex);

	// Clear previous depth map data (set to negative values indicating no data)
	for (int y = 0; y < SDLWin::MAX_DEPTH_HEIGHT; y++)
	{
		for (int x = 0; x < SDLWin::MAX_DEPTH_WIDTH; x++)
		{
			win->focusPositions[y][x] = -1.0f; // Negative indicates no data
		}
	}

	// Check if dimensions fit in our fixed-size array
	if (width <= SDLWin::MAX_DEPTH_WIDTH && height <= SDLWin::MAX_DEPTH_HEIGHT)
	{
		win->depthMapWidth = width;
		win->depthMapHeight = height;

		// Calculate dynamic min/max focus values from actual depth map data
		float minFocus = std::numeric_limits<float>::max();
		float maxFocus = std::numeric_limits<float>::lowest();
		bool hasValidData = false;

		// Transfer actual focus positions and find min/max
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				const auto &pixel = depthImage[y][x];
				if (pixel.first > 0) // Valid pixel with depth data
				{
					float focusPosition = static_cast<float>(pixel.second);
					win->focusPositions[y][x] = focusPosition;

					// Update min/max focus values
					if (focusPosition < minFocus)
						minFocus = focusPosition;
					if (focusPosition > maxFocus)
						maxFocus = focusPosition;
					hasValidData = true;
				}
				else
				{
					win->focusPositions[y][x] = -1.0f; // No data
				}
			}
		}

		// Store the calculated min/max focus values
		if (hasValidData)
		{
			win->focusMin = minFocus;
			win->focusMax = maxFocus;
			std::cout << "[Debug] Calculated dynamic focus range: " << minFocus << " - " << maxFocus << std::endl;
		}
		else
		{
			// Fallback to default values if no valid data
			win->focusMin = 190.0f;
			win->focusMax = 450.0f;
			std::cout << "[Debug] No valid focus data found, using default range: " << win->focusMin << " - " << win->focusMax << std::endl;
		}

		win->depthMapReady = true;
		win->hasDepthMap = true;
		win->depthDataVersion++; // Increment version to indicate new data

		// Force texture update since we have new data
		forceDepthMapTextureUpdate();
	}
	else
	{
		std::cout << "[SDL Window] Depth map dimensions too large: " << width << "x" << height
				  << " (max: " << SDLWin::MAX_DEPTH_WIDTH << "x" << SDLWin::MAX_DEPTH_HEIGHT << ")" << std::endl;
		win->depthMapReady = false;
	}

	pthread_mutex_unlock(&win->mutex);
}

void SDLWindow::clearDepthMapData(SDLWin *win)
{
	if (!win)
		return;

	pthread_mutex_lock(&win->mutex);
	for (int y = 0; y < SDLWin::MAX_DEPTH_HEIGHT; y++)
	{
		for (int x = 0; x < SDLWin::MAX_DEPTH_WIDTH; x++)
		{
			win->focusPositions[y][x] = -1.0f; // Negative indicates no data
		}
	}
	win->depthMapReady = false;
	win->hasDepthMap = false;
	win->depthMapWidth = 0;
	win->depthMapHeight = 0;
	// Reset focus range to defaults
	win->focusMin = 190.0f;
	win->focusMax = 450.0f;
	win->depthDataVersion++; // Increment version to indicate data cleared
	pthread_mutex_unlock(&win->mutex);

	// Force texture update to clear the old texture
	forceDepthMapTextureUpdate();
}

void SDLWindow::quit(SDLWin *win)
{
	lockcmd(win);
	win->command = CMD_QUIT;
	pthread_cond_broadcast(&win->hasCommand);
	unlockcmd(win);
}
