/**
 * @file recorder.hpp
 * @brief Declares the `Recorder` class for managing video frame operations.
 * 
 * This file provides the declaration of the `Recorder` class, which handles recording,
 * buffering, and playback of video frames. It includes methods for saving, loading,
 * and clearing frames, as well as managing signals for system interaction.
 */

#ifndef HVIGTK_RECORDER_H
#define HVIGTK_RECORDER_H

#include <tuple>
#include <vector>
#include <gtkmm.h>

#include "thread.hpp"
#include "vidframe.hpp"

class System;

/**
 * @class Recorder
 * @brief Manages the recording, buffering, and playback of video frames.
 * 
 * The Recorder class provides functionality to save, load, and buffer video frames,
 * as well as manage frame operations and signals for system interaction.
 */
class Recorder
{
public:
    /**
     * @enum Operation
     * @brief Defines the types of operations performed by the Recorder.
     */
    enum Operation
    {
        RECOP_SAVE, ///< Save frames to a location.
        RECOP_LOAD, ///< Load frames from a location.
        RECOP_BUFFER, ///< Buffer frames for playback.
        RECOP_FILLED, ///< Indicates the frame buffer is filled.
        RECOP_EMPTIED, ///< Indicates the frame buffer is emptied.
        RECOP_ADDFRAME ///< Add a frame to the buffer.
    };

    /**
     * @brief Constructor for the Recorder class.
     * 
     * Initializes the Recorder with a reference to the System object.
     * @param sys Reference to the System object.
     */
    Recorder(System &sys);

    /**
     * @brief Destructor for the Recorder class.
     * 
     * Cleans up resources and deletes any remaining frames.
     */
    ~Recorder();

    /**
     * @brief Gets a specific frame by index.
     * 
     * @param n Index of the frame to retrieve.
     * @return Pointer to the requested frame, or nullptr if the index is out of bounds.
     */
    std::shared_ptr<VidFrame> getFrame(int n);

    /**
     * @brief Adds a frame to the buffer.
     * 
     * Emits signals to indicate the buffer state.
     * @param frame Pointer to the frame to add.
     * @return Current size of the frame buffer.
     */
    int putFrame(std::shared_ptr<VidFrame> frame);

    /**
     * @brief Saves all frames to the specified location.
     * 
     * Frames are saved as PGM files in the given directory.
     * @param location Directory to save the frames.
     */
    void saveFrames(const std::string &location);

    /**
     * @brief Loads frames from the specified location.
     * 
     * Frames are loaded from PGM files in the given directory.
     * @param location Directory to load the frames from.
     */
    void loadFrames(const std::string &location);

    /**
     * @brief Checks if the Recorder is currently buffering frames.
     * 
     * @return True if buffering, false otherwise.
     */
    bool isBuffering() const;

    /**
     * @brief Stops the buffering process.
     */
    void stopBuffering();

    /**
     * @brief Counts the number of frames in the buffer.
     * 
     * @return Number of frames in the buffer.
     */
    int countFrames();

    /**
     * @brief Gets the current frame being processed.
     * 
     * @return Pointer to the current frame.
     */
    std::shared_ptr<VidFrame> getFrame();

    /**
     * @brief Clears all frames from the buffer.
     * 
     * Deletes all frames and emits a signal indicating the buffer is emptied.
     */
    void clearFrames();

    /**
     * @brief Resets the current frame pointer.
     * 
     * Clears the reference to the current frame being processed.
     */
    void resetCurrent();

	void setBufferFrameRate();
    
    /**
     * @brief Connects the Recorder to a signal dispatcher for operation completion.
     * 
     * @param sOperationComplete Pointer to the signal dispatcher.
     */
    void connectTo(VDispatcher<std::tuple<Operation, bool> > *sOperationComplete);

    /**
     * @brief Gets the signal dispatcher for loading operations.
     * 
     * @return Reference to the signal dispatcher for loading operations.
     */
    VDispatcher<std::string>& signalOperationLoad();

    /**
     * @brief Gets the signal dispatcher for saving operations.
     * 
     * @return Reference to the signal dispatcher for saving operations.
     */
    VDispatcher<std::string>& signalOperationSave();

    /**
     * @brief Gets the signal dispatcher for buffering operations.
     * 
     * @return Reference to the signal dispatcher for buffering operations.
     */
    VDispatcher<std::pair<int, int> >& signalBuffer();

private:
    /**
     * @brief Emits a signal indicating the completion of an operation.
     * 
     * @param op Operation type.
     * @param success True if the operation was successful, false otherwise.
     */
    void emitOperationComplete(Operation op, bool success);

    /**
     * @brief Buffers frames for playback.
     * 
     * Loads frames into the frame queue at the specified frame rate.
     * @param data Pair containing the start frame index and frame rate.
     */
    void bufferFrames(std::pair<int, int> data);


    std::vector<std::chrono::system_clock::time_point> frame_times; // Stores timestamps for each frame

    System &system; ///< Reference to the System object.
    bool buffering; ///< Indicates whether the Recorder is buffering frames.
    std::vector<std::shared_ptr<VidFrame>> frames; ///< Buffer for storing frames.
    VDispatcher<std::tuple<Operation, bool> > *sigOperationComplete; ///< Signal dispatcher for operation completion.
    VDispatcher<std::string> sigOperationSave, sigOperationLoad;; ///< Signal dispatcher for saving and loading operations.
    VDispatcher<std::pair<int, int> > sigBuffer; ///< Signal dispatcher for buffering operations.

    Glib::Threads::Mutex mutex; ///< Mutex for thread-safe access to frames.
    Glib::Threads::Cond frameReleased; ///< Condition variable for frame release.

    std::shared_ptr<VidFrame> current; ///< Pointer to the current frame being processed.
    int bufSleep; ///< Sleep duration between frames during buffering.
};

/**
 * @typedef RecOpRes
 * @brief Alias for the tuple representing operation results.
 */
using RecOpRes = std::tuple<Recorder::Operation, bool>;

/**
 * @def OPRESEXPAND
 * @brief Macro for expanding operation result tuples.
 */
#define OPRESEXPAND(res, op, success) Recorder::Operation op; bool success; std::tie(op, success) = (res)

#endif
