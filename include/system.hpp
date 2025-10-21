/**
 * @file system.hpp
 * @brief Declares the `System` class and related components for managing the
 * application.
 *
 * This file provides the declaration of the `System` class, which oversees the
 * application's GUI, frame processing, camera operations, and stabilization
 * components. It also includes supporting classes and namespaces for handling
 * frames, signals, and system interactions.
 */

#ifndef HVIGTK_SYSTEM_H
#define HVIGTK_SYSTEM_H

#include <fstream>
#include <iostream>
#include <queue>

#include "autofocus.hpp"
#include "cairomm/surface.h"
#include "cond.hpp"
#include "framefilter.hpp"
#include "logfile.hpp"
#include "mainwindow.hpp"
#include "phasecorr2_stabiliser.hpp"
#include "recorder.hpp"
#include "sdlwincommon.hpp"
#include "sensorfusion.hpp"
#include "sharpness_analyzer.hpp"
#include "sharpness_graph.hpp"
#include "stabiliser.hpp"
#include "thread.hpp"
#include "tsqueue.hpp"
#include "vidframe.hpp"

namespace Vimba = AVT::VmbAPI;
class System;
/**
 * @class FrameProcessor
 * @brief Handles frame processing and stabilization in the system.
 *
 * This class manages threads for processing and stabilizing frames, and
 * provides methods for releasing frames, resetting raster positions, and
 * accessing processed frames.
 */
class FrameProcessor {
  friend class System;

public:
  /**
   * @brief Constructor for FrameProcessor.
   *
   * Initializes the frame processor with a reference to the system.
   *
   * @param sys Reference to the System object.
   */
  FrameProcessor(System &sys);

  /**
   * @brief Destructor for FrameProcessor.
   *
   * Cleans up resources used by the frame processor.
   */
  ~FrameProcessor();

  /**
   * @brief Releases the current frame.
   */
  void releaseFrame();

  /**
   * @brief Resets the raster position.
   */
  void resetRaster();

  /**
   * @brief Gets the processed frame.
   *
   * @return A Cairo surface containing the processed frame.
   */
  ::Cairo::RefPtr<::Cairo::Surface> getFrame();

  /**
   * @brief Resets the zoom level.
   */
  void resetZoom();

protected:
  /**
   * @brief Stabilizes frames using phase correlation.
   */
  void stabilise();

  /**
   * @brief Processes each frame received from the FrameObserver.
   */
  void processFrame();

  ::Cairo::RefPtr<::Cairo::ImageSurface>
      processed;      ///< Processed frame surface.
  VidFrame *vidFrame; ///< Pointer to the current video frame.
  std::unordered_map<std::string, FrameFilter *>
      filters;         ///< Filters applied to frames.
  bool running;        ///< Indicates whether the processor is running.
  bool _stabNewFrame;  ///< Flag for new stabilization frame.
  bool _stabComplete;  ///< Flag for stabilization completion.
  bool _frameReleased; ///< Flag for frame release.

  Glib::Threads::Mutex mutex,
      stabMutex; ///< Mutexes for thread synchronization.
  Glib::Threads::Cond frameReleased, stabNewFrame,
      stabComplete; ///< Conditions for thread synchronization.

  System &system; ///< Reference to the System object.

  TooN::Vector<2> offset, currentOff,
      rasterPos; ///< Vectors for raster positions and offsets.
  TSQueue<VidFrame *> stabQueue,
      released; ///< Queues for stabilization and released frames.

  Glib::Threads::Thread *processorThread,
      *stabThread; ///< Threads for processing and stabilization.
};

/**
 * @class System
 * @brief Manages the overall system, including GUI, frame processing, and
 * camera operations.
 *
 * This class is responsible for initializing the system, handling GUI
 * interactions, managing threads, and operating the camera and stabilization
 * components.
 */
class System {
  friend class FrameProcessor; ///< Allows FrameProcessor to access private
                               ///< methods.

public:
  /**
   * @brief Constructor for System.
   *
   * Initializes the system with command-line arguments.
   *
   * @param argc Number of command-line arguments.
   * @param argv Array of command-line arguments.
   */
  System(int argc, char **argv);

  /**
   * @brief Destructor for System.
   *
   * Cleans up resources used by the system.
   */
  ~System();

  /**
   * @brief Gets the main window object.
   *
   * @return Reference to the MainWindow object.
   */
  MainWindow &getWindow();

  /**
   * @brief Gets the recorder object.
   *
   * @return Reference to the Recorder object.
   */
  Recorder &getRecorder();

  /**
   * @brief Starts streaming frames from the camera.
   *
   * @return True if streaming started successfully, false otherwise.
   */
  bool startStreaming();

  /**
   * @brief Stops streaming frames from the camera.
   */
  void stopStreaming();

  /**
   * @brief Sets a camera feature to a specified value.
   *
   * @tparam T Type of the feature value.
   * @param fname Name of the feature.
   * @param val Value to set.
   * @return True if the feature was set successfully, false otherwise.
   */
  template <typename T> bool setFeature(const std::string &fname, T val) {
    if (cam != nullptr) {
      Vimba::FeaturePtr pFeature;
      if (cam->GetFeatureByName(fname.c_str(), pFeature) == VmbErrorSuccess &&
          pFeature->SetValue(val) == VmbErrorSuccess) {
        logger->info("[System.hpp] Set {} to: {}", fname, val);
        return true;
      } else {
        logger->error("[System.hpp] FAILED to set {} to: {}", fname, val);
        return false;
      }
    }
    return false;
  }

  /**
   * @brief Gets the value of a camera feature.
   *
   * @tparam T Type of the feature value.
   * @param fname Name of the feature.
   * @return The value of the feature.
   */
  template <typename T> T getFeature(const std::string &fname) {
    if (cam != nullptr) {
      Vimba::FeaturePtr pFeature;
      if (cam->GetFeatureByName(fname.c_str(), pFeature) == VmbErrorSuccess) {
        T out;
        pFeature->GetValue(out);
        logger->info("[System.hpp] Got {} = {}", fname, out);
        return out;
      } else {
        logger->error("[System.hpp] FAILED to get {}", fname);
      }
    }

    return T();
  }

  /**
   * @brief Gets the frame queue.
   *
   * @return Reference to the frame queue.
   */
  TSQueue<VidFrame *> &getFrameQueue();

  /**
   * @brief Gets the dispatcher for new frame signals.
   *
   * @return Reference to the dispatcher.
   */
  Glib::Dispatcher &signalNewFrame();

  /**
   * @brief Gets the current frame.
   *
   * @return Pointer to the current video frame.
   */
  VidFrame *getFrame();

  /**
   * @brief Gets the current frames per second (FPS).
   *
   * @return The current FPS.
   */
  double getFPS();



  double getGamma();
  double getExposure();
  

  void onWindowHomePositionChanged(double val);
  void onWindowPGainChanged(double val);
  void onWindowSettingsChanged();

  /**
   * @brief Updates the center of the region of interest (ROI).
   *
   * @param x X-coordinate of the center.
   * @param y Y-coordinate of the center.
   */
  void updateROICenter(int x, int y);

  /**
   * @brief Clears the ROI display.
   */
  void clearROIDisplay();

  /**
   * @brief Gets the current ROI information.
   *
   * @param centerX Reference to store the X-coordinate of the center.
   * @param centerY Reference to store the Y-coordinate of the center.
   * @param width Reference to store the width of the ROI.
   * @param height Reference to store the height of the ROI.
   */
  void getCurrentROI(int &centerX, int &centerY, int &width, int &height) const;

  /**
   * @brief Sets the SDL window reference for ROI updates.
   *
   * @param win Pointer to the SDL window.
   */
  void setSDLWindow(SDLWindow::SDLWin *win);

  /**
   * @brief Gets the sensor fusion object.
   *
   * @return Pointer to the SensorFusion object.
   */
  SensorFusion *getSensorFusion() { return sensorFusion.get(); }

  /**
   * @brief Gets the current stabilization offset.
   *
   * @param offsetX Reference to store the X-offset.
   * @param offsetY Reference to store the Y-offset.
   * @return True if stabilization is active, false otherwise.
   */
  bool getStabilizationOffset(double &offsetX, double &offsetY);

  /**
   * @brief Handles the "Get Depths" button click.
   */
  void onGetDepthsClicked();

  /**
   * @brief Handles toggling of the "Get Depths" condition.
   *
   * @param gettingDepths True if "Get Depths" is active, false otherwise.
   */
  void whenGetDepthsToggled(bool gettingDepths);

  /**
   * @brief Handles toggling of the "View Depths" condition.
   *
   * @param viewingDepths True if "View Depths" is active, false otherwise.
   */
  void whenViewDepthsToggled(bool viewingDepths);

  /**
   * @struct DepthMapData
   * @brief Represents depth mapping data.
   *
   * This structure contains information about depth mapping, including
   * sharpness values, focus positions, dimensions, and validity status.
   */
  struct DepthMapData {
    std::vector<std::vector<std::pair<double, double>>>
        depthImage; /**< Pairs of max sharpness and actual focus position. */
    int width;      /**< Width of the depth map. */
    int height;     /**< Height of the depth map. */
    bool isValid;   /**< Indicates whether the depth map is valid. */

    /**
     * @brief Default constructor for DepthMapData.
     *
     * Initializes the depth map with default values.
     */
    DepthMapData() : width(0), height(0), isValid(false) {}
  };

  DepthMapData currentDepthMap;

private:
  /**
   * @brief Creates a stabilizer map with default parameters.
   *
   * @param frame Reference to the video frame.
   */
  void createStabiliserMapWithDefaults(const VidFrame &frame);

  /**
   * @brief Calculates the threshold for a target percentage of pixels.
   *
   * @param image Reference to the image.
   * @param vesselSize Size of the vessel.
   * @param targetPercentage Target percentage of pixels.
   * @return The calculated threshold.
   */
  double calculateThresholdForPercentage(const CVD::Image<unsigned char> &image,
                                         double vesselSize,
                                         double targetPercentage);

  void renderFrame();  ///< Renders the current frame.
  void releaseFrame(); ///< Releases the current frame.

  void
  whenLiveViewToggled(bool viewingLive); ///< Handles toggling of the live view.
  void whenStabiliseToggled(
      bool stabilising); ///< Handles toggling of stabilization.
  void
  whenHoldFocusToggled(bool holdingFocus); ///< Handles toggling of hold focus.
  void
  when3DStabToggled(bool active); ///< Handles toggling of 3D stabilization.
  void
  when2DStabToggled(bool active2); ///< Handles toggling of 2D stabilization.
  void whenLoadingToggled(bool loading);     ///< Handles toggling of loading.
  void whenSavingToggled(bool saving);       ///< Handles toggling of saving.
  void whenPlayingToggled(bool playing);     ///< Handles toggling of playing.
  void whenSeekingToggled(bool seeking);     ///< Handles toggling of seeking.
  void whenRecordingToggled(bool recording); ///< Handles toggling of recording.

  void onRecorderOperationComplete(
      RecOpRes res); ///< Handles recorder operation completion.
  void onWindowFeatureUpdated(
      std::string fname,
      double val); ///< Handles feature updates from the window.
  void onWindowThresholdChanged(
      double val); ///< Handles threshold changes from the window.
  void
  onWindowScaleChanged(double val); ///< Handles scale changes from the window.
  void onWindowBestFocusChanged(
      double val);             ///< Handles best focus changes from the window.
  void onWindowPauseClicked(); ///< Handles pause button clicks.
  void onFindFocusClicked();   ///< Handles find focus button clicks.
  void onResetClicked();       ///< Handles reset button clicks.
  void onRecenterClicked();    ///< Handles recenter button clicks.
  void onWindowEnterClicked(); ///< Handles enter button clicks.

  bool
  onCloseClicked(const GdkEventAny *event); ///< Handles window close events.

  void on_error();            ///< Handles errors.
  void handleDisconnection(); ///< Handles device disconnections.

  struct Private; ///< Private implementation details.

  MainWindow window;        ///< Main window object.
  autofocus AF;             ///< Autofocus object.
  Vimba::VimbaSystem &vsys; ///< Vimba system object.
  Vimba::CameraPtr cam;     ///< Camera pointer.

  CVD::ImageRef size;          ///< Image size reference.
  CVD::Image<unsigned int> im; ///< Image object.

  struct Private *priv; ///< Private implementation details.

  Glib::Dispatcher sigNewFrame; ///< Dispatcher for new frame signals.

  TSQueue<VidFrame *> frameQueue; ///< Queue for video frames.

  FrameProcessor frameProcessor; ///< Frame processor object.

  ObjectThread thread; ///< Thread object.

  std::unique_ptr<Recorder> recorder;               ///< Recorder object.
  VDispatcher<RecOpRes> sRecorderOperationComplete; ///< Dispatcher for recorder
                                                    ///< operation completion.

  Stabiliser stabiliser; ///< Stabilizer object.
  bool madeMap;          ///< Flag indicating whether a map was made.

  bool errorDialogShown =
      false; ///< Flag indicating whether an error dialog was shown.

  // Flags for disconnection states
  bool tiltedCamDisconnected = false;
  bool sensorFusionDisconnected = false;
  bool lensDisconnected = false;

  // Add a boolean to enable or disable PhaseCorr usage.
  bool usePhaseCorr = true;

  // Add a PhaseCorrStabiliser2 object
  PhaseCorrStabiliser2 phaseCorrStabiliser{
      8, 1.0}; // 1.0 = complete replacement like old stabilizer
  // Add a boolean to check if the reference frame is set
  bool phaseCorrStabiliserReferenceNotSet = true;

  SharpnessAnalyzer sharpnessAnalyzer;  ///< Sharpness analyzer object.
  Glib::Dispatcher sigSharpnessUpdated; ///< Dispatcher for sharpness updates.
  std::vector<double> currentSharpnessValues; ///< Current sharpness values.
  bool sharpnessUpdateEnabled; ///< Flag for enabling sharpness updates.
  std::chrono::time_point<std::chrono::steady_clock>
      lastSharpnessUpdate;       ///< Last sharpness update time.
  SharpnessGraph sharpnessGraph; ///< Sharpness graph object.

  // Method to update the sharpness graph in the UI
  void updateSharpnessGraph();

  std::unique_ptr<SensorFusion> sensorFusion; ///< Sensor fusion object.

  bool sensorFusionUpdatingFocus =
      false; ///< Flag for sensor fusion focus updates.
};

#endif
