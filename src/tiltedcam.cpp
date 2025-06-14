#include "tiltedcam.hpp"
#include "autofocus.hpp"
#include "logfile.hpp"
#include "notificationCenter.hpp"
#include <iostream>
#include <unistd.h>
#include <string.h> // For memcpy

#include <atomic>
#include <thread>
#include <chrono>

bool bTiltedCamLogFlag = 0; // 1 = log, 0 = don't log

tiltedcam::tiltedcam() : 
    m_captureBuffer(nullptr),
    m_processingBuffer(nullptr),
    m_newFrameAvailable(false),
    m_stopThread(false),
    m_bufferSize(0),
    m_threadRunning(false)
{
    // Constructor properly initializes all members
}

bool tiltedcam::initialize()
{
    bool noError = true;

    int iNumofConnectCameras = ASIGetNumOfConnectedCameras();
    std::cout << "Number of connected cameras: " << iNumofConnectCameras << "\n";

    if (iNumofConnectCameras > 0)
    {
        if (bTiltedCamLogFlag)
            logger->info("[tiltedcam::tiltedcam()] connected to the tilted camera");
    }
    else
    {
        logger->error("[tiltedcam::tiltedcam()] failed to connect to the tilted camera");
        noError = false;
    }

    if (ASIOpenCamera(0) == ASI_SUCCESS)
    {
        if (bTiltedCamLogFlag)
            logger->info("[tiltedcam::tiltedcam()] opened the tilted camera");
    }
    else
    {
        logger->error("[tiltedcam::tiltedcam()] failed to open the tilted camera");
        noError = false;
    }

    if (ASIInitCamera(0) == ASI_SUCCESS)
    {
        if (bTiltedCamLogFlag)
            logger->info("[tiltedcam::tiltedcam()] initialized the tilted camera");
    }
    else
    {
        logger->error("[tiltedcam::tiltedcam()] failed to initialize the tilted camera");
        noError = false;
    }

    // 16ms exposure, the highest possible at 60fps. Its already pretty dark at this exposure, so wouldn't reduce more to minimize motion blur.
    if (ASISetControlValue(0, ASI_EXPOSURE, 16000, ASI_FALSE) == ASI_SUCCESS)
    {
        if (bTiltedCamLogFlag)
            logger->info("[tiltedcam::tiltedcam()] set tilted camera exposure to auto mode");
    }
    else
    {
        logger->error("[tiltedcam::tiltedcam()] failed to set tilted camera exposure");
        noError = false;
    }

    if (ASISetControlValue(0, ASI_GAIN, 0, ASI_TRUE) == ASI_SUCCESS)
    {
        if (bTiltedCamLogFlag)
            logger->info("[tiltedcam::tiltedcam()] set tilted camera gain to automatically adjust");
    }
    else
    {
        logger->error("[tiltedcam::tiltedcam()] failed to set tilted camera gain");
        noError = false;
    }

    if (ASISetControlValue(0, ASI_HIGH_SPEED_MODE, 1, ASI_FALSE) == ASI_SUCCESS)
    {
        if (bTiltedCamLogFlag)
            logger->info("[tiltedcam::tiltedcam()] set tilted camera to highspeed mode");
    }
    else
    {
        logger->error("[tiltedcam::tiltedcam()] failed to set tilted camera to highspeed mode");
        noError = false;
    }

    // ASISetControlValue(0, ASI_AUTO_MAX_EXP, 15, ASI_TRUE);
    // ASISetControlValue(0,  ASI_EXPOSURE, 15, ASI_TRUE);
    // ASISetControlValue(0, ASI_BANDWIDTHOVERLOAD, 90, ASI_TRUE);

    if (ASISetROIFormat(0, 1280, 960, 1, ASI_IMG_RAW8) == ASI_SUCCESS)
    {
        // if (ASISetROIFormat(0, 320, 240, 1, ASI_IMG_RAW8) == ASI_SUCCESS) {
        if (bTiltedCamLogFlag)
            logger->info("[tiltedcam::tiltedcam()] set tilted camera image to Raw8 and 1 bin, with image size 1280x960");
    }
    else
    {
        logger->error("[tiltedcam::tiltedcam()] failed to set ROI format for tilted camera");
        noError = false;
    };

    usleep(100000);

    if (ASIStartVideoCapture(0) == ASI_SUCCESS)
    {
        if (bTiltedCamLogFlag)
            logger->info("[tiltedcam::tiltedcam()] started tilted camera video capture");
    }
    else
    {
        logger->error("[tiltedcam::tiltedcam()] failed to start video capture");
        noError = false;
    };

    std::cout << "final result of TiltedCam::noError: " << noError << "\n";
    if (noError)
    {
        if (bTiltedCamLogFlag)
            logger->info("[tiltedcam::tiltedcam()] tilted camera initialized successfully");
        return noError;
    }
    else
    {
        logger->error("[tiltedcam::tiltedcam()] tilted camera failed to initialize");
        NotificationCenter::instance().postNotification("TiltedCamDisconnected");
        return noError;
    }

    if (bTiltedCamLogFlag)
        logger->info("[tiltedcam::tiltedcam()] constructor finished");
}

tiltedcam::~tiltedcam()
{
    // First stop video capture and the thread
    ASIStopVideoCapture(0);
    stopCaptureThread();

    // Then close the camera with proper delay
    usleep(50000);  // 50ms to ensure camera operations are complete
    ASICloseCamera(0);
    
    logger->info("[tiltedcam::~tiltedcam] Camera closed and resources freed");
}

// Getters
long tiltedcam::getGain()
{
    long gain; // new variable of type settings
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetControlValue(0, ASI_GAIN, &gain, &asiAuto);
    if (bTiltedCamLogFlag)
        logger->info("[tiltedcam::getGain()] got gain = {}", gain);
    return gain;
}
long tiltedcam::getExposure()
{
    long exposure; // new variable of type settings
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetControlValue(0, ASI_EXPOSURE, &exposure, &asiAuto);
    if (bTiltedCamLogFlag)
        logger->info("[tiltedcam::getExposure()] got exposure = {}", exposure);
    return exposure;
}
long tiltedcam::getGamma()
{
    long gamma; // new variable of type settings
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetControlValue(0, ASI_GAMMA, &gamma, &asiAuto);
    if (bTiltedCamLogFlag)
        logger->info("[tiltedcam::getGamma()] got gamma = {}", gamma);
    return gamma;
}
long tiltedcam::getHighSpeedMode()
{
    long highSpeedMode; // new variable of type settings
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetControlValue(0, ASI_HIGH_SPEED_MODE, &highSpeedMode, &asiAuto);
    if (bTiltedCamLogFlag)
        logger->info("[tiltedcam::getHighSpeedMode()] got high speed mode = {}", highSpeedMode);
    return highSpeedMode;
}
int tiltedcam::getImageWidth()
{
    int width; // new variable of type settings
    int height;
    int bins;
    ASI_IMG_TYPE imageType;
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetROIFormat(0, &width, &height, &bins, &imageType);
    if (bTiltedCamLogFlag)
        logger->info("[tiltedcam::getImageWidth()] got image width = {}", width);
    return width;
}
int tiltedcam::getImageHeight()
{
    int width; // new variable of type settings
    int height;
    int bins;
    ASI_IMG_TYPE imageType;
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetROIFormat(0, &width, &height, &bins, &imageType);
    if (bTiltedCamLogFlag)
        logger->info("[tiltedcam::getImageHeight()] got image height = {}", height);
    return height;
}
int tiltedcam::getBins()
{
    int width; // new variable of type settings
    int height;
    int bins;
    ASI_IMG_TYPE imageType;
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetROIFormat(0, &width, &height, &bins, &imageType);
    if (bTiltedCamLogFlag)
        logger->info("[tiltedcam::getBins()] got image bins = {}", bins);
    return bins;
}
ASI_IMG_TYPE tiltedcam::getImageType()
{
    int width; // new variable of type settings
    int height;
    int bins;
    ASI_IMG_TYPE imageType;
    ASI_BOOL asiAuto = ASI_FALSE;
    ASIGetROIFormat(0, &width, &height, &bins, &imageType);
    if (bTiltedCamLogFlag)
        logger->info("[tiltedcam::getImageType()] got image type = {}", imageType);
    return imageType;
}

// Setters
void tiltedcam::setGain(long newGain)
{
    if (ASISetControlValue(0, ASI_GAIN, newGain, ASI_FALSE) == ASI_SUCCESS)
    {
        if (bTiltedCamLogFlag)
            logger->info("[tiltedcam::setGain()] set tilted camera gain to {}", newGain);
    }
    else
    {
        logger->error("[tiltedcam::setGain()] failed to change setting of tilted camera");
    }
}
void tiltedcam::setExposure(long newExposure)
{
    if (ASISetControlValue(0, ASI_EXPOSURE, newExposure, ASI_FALSE) == ASI_SUCCESS)
    {
        if (bTiltedCamLogFlag)
            logger->info("[tiltedcam::setExposure()] set tilted camera exposure to {}", newExposure);
    }
    else
    {
        logger->error("[tiltedcam::setExposure()] failed to change setting of tilted camera");
    }
}
void tiltedcam::setGamma(long newGamma)
{
    if (ASISetControlValue(0, ASI_GAMMA, newGamma, ASI_FALSE) == ASI_SUCCESS)
    {
        if (bTiltedCamLogFlag)
            logger->info("[tiltedcam::setGamma()] set tilted camera gamma to {}", newGamma);
    }
    else
    {
        logger->error("[tiltedcam::setGamma()] failed to change setting of tilted camera");
    }
}
void tiltedcam::setHighSpeedMode(long newHighSpeedMode)
{
    if (ASISetControlValue(0, ASI_HIGH_SPEED_MODE, newHighSpeedMode, ASI_FALSE) == ASI_SUCCESS)
    {
        if (bTiltedCamLogFlag)
            logger->info("[tiltedcam::setHighSpeedMode()] set tilted camera high speed mode to {}", newHighSpeedMode);
    }
    else
    {
        logger->error("[tiltedcam::setHighSpeedMode()] failed to change setting of tilted camera");
    }
}
// PREVIOUSLY USED CODE
// void tiltedcam::startCaptureThread() {
//     // Make sure we don't start twice
//     if (m_threadRunning) {
//         return;
//     }

//     // Make sure buffers are allocated
//     if (!m_captureBuffer) {
//         int width, height;
//         ASI_IMG_TYPE type;
//         int bin;
        
//         if (ASIGetROIFormat(0, &width, &height, &bin, &type) != ASI_SUCCESS) {
//             logger->error("[tiltedcam::startCaptureThread] Failed to get ROI format");
//             return;
//         }
        
//         m_bufferSize = width * height; // Assuming 8-bit images
        
//         try {
//             m_captureBuffer = (unsigned char*)malloc(m_bufferSize);
//             if (!m_captureBuffer) {
//                 logger->error("[tiltedcam::startCaptureThread] Failed to allocate capture buffer");
//                 return;
//             }
            
//             m_processingBuffer = (unsigned char*)malloc(m_bufferSize);
//             if (!m_processingBuffer) {
//                 logger->error("[tiltedcam::startCaptureThread] Failed to allocate processing buffer");
//                 free(m_captureBuffer);
//                 m_captureBuffer = nullptr;
//                 return;
//             }
//         } catch (const std::exception& e) {
//             logger->error("[tiltedcam::startCaptureThread] Exception during buffer allocation: {}", e.what());
//             return;
//         }
//     }
    
//     logger->info("[tiltedcam::startCaptureThread] Starting capture thread");
    
//     m_stopThread = false;
//     m_newFrameAvailable = false;
    
//     try {
//         m_captureThread = std::thread(&tiltedcam::captureThreadFunc, this);
//         m_threadRunning = true;
//     } catch (const std::exception& e) {
//         logger->error("[tiltedcam::startCaptureThread] Failed to create thread: {}", e.what());
//         free(m_captureBuffer);
//         free(m_processingBuffer);
//         m_captureBuffer = nullptr;
//         m_processingBuffer = nullptr;
//     }
// }



void tiltedcam::startCaptureThread() {
    if (m_threadRunning) {
        logger->info("[tiltedcam::startCaptureThread] Thread already running");
        return;
    }
    
    // Calculate actual buffer size based on current ROI settings
    int width, height, bins;
    ASI_IMG_TYPE imageType;
    ASIGetROIFormat(0, &width, &height, &bins, &imageType);
    
    // Calculate buffer size based on image format
    long calculatedBufferSize;
    switch(imageType) {
        case ASI_IMG_RAW8:
            calculatedBufferSize = width * height;
            break;
        case ASI_IMG_RAW16:
            calculatedBufferSize = width * height * 2;
            break;
        case ASI_IMG_RGB24:
            calculatedBufferSize = width * height * 3;
            break;
        default:
            calculatedBufferSize = width * height; // Default to RAW8
            break;
    }
    
    m_bufferSize = calculatedBufferSize;
    logger->info("[tiltedcam::startCaptureThread] Calculated buffer size: {} bytes for {}x{} image", 
                 m_bufferSize, width, height);
    
    // Free existing buffers if they exist
    if (m_captureBuffer) {
        free(m_captureBuffer);
        m_captureBuffer = nullptr;
    }
    if (m_processingBuffer) {
        free(m_processingBuffer);
        m_processingBuffer = nullptr;
    }
    
    // Allocate buffers with proper size
    try {
        m_captureBuffer = (unsigned char*)malloc(m_bufferSize);
        if (!m_captureBuffer) {
            logger->error("[tiltedcam::startCaptureThread] Failed to allocate capture buffer");
            return;
        }
        
        m_processingBuffer = (unsigned char*)malloc(m_bufferSize);
        if (!m_processingBuffer) {
            logger->error("[tiltedcam::startCaptureThread] Failed to allocate processing buffer");
            free(m_captureBuffer);
            m_captureBuffer = nullptr;
            return;
        }
        
        // Initialize buffers to zero
        memset(m_captureBuffer, 0, m_bufferSize);
        memset(m_processingBuffer, 0, m_bufferSize);
        
    } catch (const std::exception& e) {
        logger->error("[tiltedcam::startCaptureThread] Exception during buffer allocation: {}", e.what());
        return;
    }
    
    logger->info("[tiltedcam::startCaptureThread] Starting capture thread");
    
    m_stopThread = false;
    m_newFrameAvailable = false;
    
    try {
        m_captureThread = std::thread(&tiltedcam::captureThreadFunc, this);
        m_threadRunning = true;
    } catch (const std::exception& e) {
        logger->error("[tiltedcam::startCaptureThread] Failed to create thread: {}", e.what());
        free(m_captureBuffer);
        free(m_processingBuffer);
        m_captureBuffer = nullptr;
        m_processingBuffer = nullptr;
    }
}

void tiltedcam::stopCaptureThread() {
    if (!m_threadRunning) {
        return;
    }
    
    logger->info("[tiltedcam::stopCaptureThread] Stopping capture thread");
    
    // Signal thread to stop
    m_stopThread = true;
    
    // Add a short delay to ensure the thread sees the flag
    usleep(100000);  // 100ms
    
    // Join the thread - with timeout protection
    if (m_captureThread.joinable()) {
        try {
            m_captureThread.join();
            logger->info("[tiltedcam::stopCaptureThread] Thread joined successfully");
        } catch (const std::exception& e) {
            logger->error("[tiltedcam::stopCaptureThread] Exception during thread join: {}", e.what());
        }
    }
    
    // Make sure camera is stopped before freeing buffers
    ASIStopVideoCapture(0);
    
    // Free the buffers - with null checks
    if (m_captureBuffer) {
        void* bufToFree = m_captureBuffer;
        m_captureBuffer = nullptr;  // Set to null before freeing to prevent use-after-free
        free(bufToFree);
    }
    
    if (m_processingBuffer) {
        void* bufToFree = m_processingBuffer;
        m_processingBuffer = nullptr;
        free(bufToFree);
    }
    
    m_threadRunning = false;
    logger->info("[tiltedcam::stopCaptureThread] Cleanup completed");
}

// PREVIOUSLY USED CODE
// void tiltedcam::captureThreadFunc() {
//     logger->info("[tiltedcam::captureThreadFunc] Capture thread started");
    
//     while (!m_stopThread) {
//         // Get the image from the camera
//         ASI_ERROR_CODE err = ASIGetVideoData(0, m_captureBuffer, m_bufferSize, 200); // Use timeout to avoid hanging
        
//         if (err == ASI_SUCCESS) {
//             // Lock the mutex to safely update the processing buffer
//             std::lock_guard<std::mutex> lock(m_bufferMutex);
            
//             // Copy new frame to processing buffer
//             memcpy(m_processingBuffer, m_captureBuffer, m_bufferSize);
            
//             // Set flag that new frame is available
//             m_newFrameAvailable = true;
//         } else {
//             logger->error("[tiltedcam::captureThreadFunc] Error getting image from camera: {}", err);
//             // Short sleep to avoid hammering the camera on errors
//             usleep(10000); // 10ms
//         }
//     }
    
//     logger->info("[tiltedcam::captureThreadFunc] Capture thread ending");
// }

void tiltedcam::captureThreadFunc() {
    logger->info("[tiltedcam::captureThreadFunc] Capture thread started");
    
    int droppedFrameCount = 0;
    
    while (!m_stopThread) {
        // Use longer timeout as recommended: exposure*2 + 500ms
        // For typical exposures, use at least 1000ms timeout
        ASI_ERROR_CODE err = ASIGetVideoData(0, m_captureBuffer, m_bufferSize, 1000);
        
        if (err == ASI_SUCCESS) {
            // Copy the captured image to the processing buffer
            {
                std::lock_guard<std::mutex> lock(m_bufferMutex);
                memcpy(m_processingBuffer, m_captureBuffer, m_bufferSize);
                m_newFrameAvailable = true;
            }
            
        } else if (err == ASI_ERROR_TIMEOUT) {
            // Timeout is normal, just continue
            continue;
            
        } else {
            logger->error("[tiltedcam::captureThreadFunc] Error getting image: {}", err);
            
            // Check for dropped frames
            int currentDroppedFrames = 0;
            if (ASIGetDroppedFrames(0, &currentDroppedFrames) == ASI_SUCCESS) {
                if (currentDroppedFrames > droppedFrameCount) {
                    logger->warn("[tiltedcam::captureThreadFunc] Dropped frames detected: {} total", 
                                currentDroppedFrames);
                    droppedFrameCount = currentDroppedFrames;
                }
            }
            
            // Short delay before retrying
            usleep(5000); // 5ms
        }
    }
    
    logger->info("[tiltedcam::captureThreadFunc] Capture thread ended");
}

bool tiltedcam::getLatestFrame(unsigned char* destination, long size) {
    if (!m_threadRunning || !destination || size != m_bufferSize) {
        return false;
    }
    
    bool result = false;
    
    {
        std::lock_guard<std::mutex> lock(m_bufferMutex);
        if (m_newFrameAvailable) {
            memcpy(destination, m_processingBuffer, m_bufferSize);
            m_newFrameAvailable = false;
            result = true;
        }
    }
    
    return result;
}
