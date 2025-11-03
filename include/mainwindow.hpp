/**
 * @file mainwindow.hpp
 * @brief Defines the MainWindow class for managing the main application GUI and
 * its components.
 */

#ifndef HVIGTK_MAINWINDOW_H
#define HVIGTK_MAINWINDOW_H

#include <cairomm/cairomm.h>
#include <gtkmm.h>
#include <unordered_map>

#include "cond.hpp"
#include "gtkmm/drawingarea.h"
#include "settings.hpp"
#include "sharpness_graph.hpp"
#include "sigc++/connection.h"
#include "vidframe.hpp"

/**
 * @class ScaleWidget
 * @brief A custom GTK widget combining a scale and spin button for numeric
 * input.
 *
 * This widget allows users to input numeric values using a slider (scale) or a
 * spin button. It supports customization of precision, size, and limits.
 */
class ScaleWidget : public Gtk::Bin {
public:
  /**
   * @brief Constructor for ScaleWidget.
   *
   * Initializes the widget with specified range, increment, default value, and
   * dimensions.
   *
   * @param lower Lower limit of the scale.
   * @param upper Upper limit of the scale.
   * @param inc Increment step for the scale.
   * @param def Default value for the scale.
   * @param spinButtonWidth Width of the spin button (optional).
   * @param scaleWidth Width of the scale (optional).
   * @param stepSnap Whether the scale snaps to steps (optional).
   */
  ScaleWidget(double lower, double upper, double inc, double def,
              int spinButtonWidth = 0, int scaleWidth = 0,
              bool stepSnap = false);

  /**
   * @brief Destructor for ScaleWidget.
   */
  virtual ~ScaleWidget();

  /**
   * @brief Signal emitted when the value changes.
   *
   * @return SignalChanged object for connecting to the signal.
   */
  using SignalChanged = sigc::signal<void(double)>;
  SignalChanged signalChanged();

  /**
   * @brief Sets the precision of the spin button.
   *
   * @param digits Number of decimal places.
   */
  void setSpinButtonPrec(int digits);

  /**
   * @brief Sets the width of the spin button.
   *
   * @param width Width in pixels.
   */
  void setSpinButtonWidth(int width);

  /**
   * @brief Sets the size of the scale.
   *
   * @param width Width in pixels.
   * @param height Height in pixels.
   */
  void setScaleSizeRequest(int width, int height);

  /**
   * @brief Gets the current value of the widget.
   *
   * @return Current value.
   */
  double getValue() const;

  /**
   * @brief Sets the value of the widget.
   *
   * @param v New value.
   */
  void setValue(double v);

  /**
   * @brief Sets the upper limit of the scale.
   *
   * @param value New upper limit.
   */
  void setUpperLimit(double value);

private:
  /**
   * @brief Handles changes to the spin button.
   */
  void spinButtonChanged();

  /**
   * @brief Handles changes to the scale.
   */
  void scaleChanged();

  SignalChanged sigChanged;

  Gtk::Scale scale;           ///< Scale widget for slider input.
  Gtk::SpinButton spinButton; ///< Spin button for numeric input.
  Gtk::Grid grid;             ///< Layout grid for the widget.

  sigc::connection spinButtonConnection,
      scaleConnection; ///< Signal connections.
  bool stepSnap;       ///< Whether the scale snaps to steps.
};

/**
 * @class RenderFilter
 * @brief Abstract base class for rendering filters.
 *
 * This class defines an interface for custom rendering filters that can be
 * applied to graphical contexts.
 */
class RenderFilter {
public:
  /**
   * @brief Draws the filter on the given Cairo context.
   *
   * @param cr Cairo context to draw on.
   */
  virtual void draw(const ::Cairo::RefPtr<::Cairo::Context> &cr) = 0;
};

/**
 * @class MainWindow
 * @brief Main application window for the GTK GUI.
 *
 * This class manages the main GUI elements, including buttons, sliders, and
 * toggles. It handles user interactions, updates camera values, and renders
 * frames.
 */
class MainWindow : public Gtk::Window {
public:
  /**
   * @brief Constructor for MainWindow.
   *
   * Initializes the main window and its components.
   */
  struct Private;
  MainWindow();

  /**
   * @brief Destructor for MainWindow.
   */
  virtual ~MainWindow();

  /**
   * @brief Gets the value of the stabilization wait scale.
   *
   * @return Current value of the stabilization wait scale.
   */
  double getStabWaitScaleValue() const;

  /**
   * @brief Gets the value of the recording size scale.
   *
   * @return Current value of the recording size scale.
   */
  double getRecordingSizeScaleValue() const;

  /**
   * @brief Gets the value of the best focus scale.
   *
   * @return Current value of the best focus scale.
   */
  double getBestFocusScaleValue() const;

  /**
   * @brief Sets the value of the best focus scale.
   *
   * @param v New value for the best focus scale.
   */
  void setBestFocusScaleValue(double v);

  /**
   * @brief Gets the dimensions of the display.
   *
   * @param w Reference to store the width.
   * @param h Reference to store the height.
   */
  void getDisplayDimensions(double &w, double &h) const;

  /**
   * @brief Updates camera values.
   *
   * @param gain Gain value.
   * @param expose Exposure value.
   * @param gamma Gamma value.
   */
  void updateCameraValues(double gain, double expose, double gamma);

  /**
   * @brief Displays a message in the FPS label.
   *
   * @param msg Message to display.
   */
  void displayMessageFPS(const std::string &msg);

  /**
   * @brief Displays a message in the load/save label.
   *
   * @param msg Message to display.
   */
  void displayMessageLoadSave(const std::string &msg);

  /**
   * @brief Displays a message in the error label.
   *
   * @param msg Message to display.
   */
  void displayMessageError(const std::string &msg);

  void displayWarningMessage(const std::string &msg);

  /**
   * @brief Renders a frame on the display.
   *
   * @param frame Pointer to the frame to render.
   */
  void renderFrame(VidFrame *frame);

  /**
   * @brief Updates the sharpness graph with new values.
   *
   * @param values Vector of sharpness values.
   */

  using SignalFrameDrawn = sigc::signal<void()>;
  SignalFrameDrawn signalFrameDrawn();

  using SignalFeatureUpdated = sigc::signal<void(std::string, double)>;
  SignalFeatureUpdated signalFeatureUpdated();

  // using SignalThresholdChanged = sigc::signal<void(double)>;
  // SignalThresholdChanged signalThresholdChanged();

  // using SignalScaleChanged = sigc::signal<void(double)>;
  // SignalScaleChanged signalScaleChanged();

  using SignalBestFocusChanged = sigc::signal<void(double)>;
  SignalBestFocusChanged signalBestFocusChanged();

  using SignalPauseClicked = sigc::signal<void()>;
  SignalPauseClicked signalPauseClicked();

  using SignalEnterClicked = sigc::signal<void()>;
  SignalEnterClicked signalEnterClicked();

  using SignalFindFocusClicked = sigc::signal<void()>;
  SignalFindFocusClicked signalFindFocusClicked();

  using SignalResetClicked = sigc::signal<void()>;
  SignalResetClicked signalResetClicked();

  using SignalRecenterClicked = sigc::signal<void()>;
  SignalRecenterClicked signalRecenterClicked();

  using SignalScaleBarToggled = sigc::signal<void()>;
  SignalScaleBarToggled signalScaleBarToggled();

  using SignalGetDepthsClicked = sigc::signal<void()>;
  SignalGetDepthsClicked signalGetDepthsClicked();

  void setHasBuffer(bool val);
  void setLiveView(bool val);
  void setLoading(bool val);
  void setSaving(bool val);
  void setPlayingBuffer(bool val);
  void setSeeking(bool val);
  void setRecording(bool val);
  void setMakingMap(bool val);
  void setShowingMap(bool val);
  void set3DStab(bool val);
  void setHoldFocus(bool val);
  void setFindFocus(bool val);
  void setStabiliseActive(bool val);
  void setViewDepths(bool val);

  // Condition& getMakeMapActive(); DEPRECATED
  Condition &getStabiliseActive();
  // Condition& getShowMapActive(); DEPRECATED
  Condition &getHoldFocusActive();
  Condition &get3DStabActive();
  Condition &get2DStabActive();
  Condition &getHasBuffer();
  Condition &getLiveView();
  Condition &getLoading();
  Condition &getSaving();
  Condition &getPlayingBuffer();
  Condition &getSeeking();
  Condition &getRecording();
  Condition &getPausedRecording();
  Condition &getFindFocusActive();
  Condition &getViewDepthsActive();
  Condition &getGetDepthsActive();
  Condition &getScaleBarActive();

  int getFrameSliderValue() const;
  double getFrameRateEntryBox() const;

  double getGammaScaleBox() const;
  double getExposureScaleBox() const;

  std::string getFileLocation() const;

  void addRenderFilter(const std::string &key, RenderFilter *filter);
  RenderFilter *removeRenderFilter(const std::string &key);

  void setTrackingFPS(bool val);

  // CONCERNING
  void showOutOfBoundsWarning();
  void hideOutOfBoundsWarning();

  using SignalHomePositionChanged = sigc::signal<void(double)>;
  SignalHomePositionChanged signalHomePositionChanged();

  using SignalPGainChanged = sigc::signal<void(double)>;
  SignalPGainChanged signalPGainChanged();

  using SignalSettingsChanged = sigc::signal<void()>;
  SignalSettingsChanged signalSettingsChanged();

  void updateSharpnessGraph(const std::vector<double> &values);

protected:
  /**
   * @brief Called when the window is realized.
   */
  virtual void on_realize() override;

  /**
   * @brief Called when the window is shown.
   */
  virtual void on_show() override;

  /**
   * @brief Handles enter key press events (connects to enter button).
   *
   * @param event Pointer to the GdkEventKey structure containing event data.
   * @return True if the event was handled, false otherwise.
   */
  bool on_key_press_event(GdkEventKey *event) override;

  bool _on_state_event(GdkEventWindowState *window_state_event);

private:
  const std::string DEFAULT_HVI_PATH = std::string(std::getenv("HOME")) + "/Desktop/HVI-data";
  std::string current_file_path = DEFAULT_HVI_PATH;

  /*
   * This is the function which will draw the currently loaded frame.
   * Currently the Cairo library is used. To trigger this function
   * call MainWindow::renderFrame(...).
   */
  bool renderDisplay(const ::Cairo::RefPtr<::Cairo::Context> &cr);

  void onFrameDrawn();
  bool updateFPSCounter();

  // void whenMakeMapToggled(bool makingMap); DEPRECATED
  void whenStabiliseToggled(bool stabilising);
  // void whenShowMapToggled(bool showingMap); DEPRECATED

  void whenHoldFocusToggled(bool holdingFocus);
  void when3DStabToggled(bool active);
  void when2DStabToggled(bool active2);
  void onFindFocusClicked();
  void onResetClicked();
  void onRecenterClicked();

  void onScaleBarToggled();

  void bufferFilled();
  void bufferEmptied();

  void viewingLive();
  void viewingBuffer();

  void whenLoadingToggled(bool loading);
  void whenSavingToggled(bool saving);
  void whenRecordingToggled(bool recording);
  void whenPausedRecordingToggled(bool paused);
  void whenTrackingFPSToggled(bool tracking);

  void onLoadButtonClicked();
  void onSaveButtonClicked();

  void onPlayButtonClicked();
  void onLiveToggled();
  void onRecordClicked();
  void onPauseClicked();
  void onBackButtonClicked();

  void whenPlayingBufferToggled(bool playing);

  void onFrameSliderChange(double val);

  void onGainScaleChange(double val);
  void onExposeScaleChange(double val);
  void onGammaScaleChange(double val);
  void onFrameRateChange(double val);

  // void onThresScaleChange(double val); DEPRECATED
  // void onScaleScaleChange(double val); DEPRECATED
  void onRecordingSizeScaleChange(double val);
  void onBestFocusScaleChange(double val);
  void onHomePositionScaleChange(double val);
  void onPGainScaleChange(double val);
  // Add the missing function declaration for handling findFocusToggle
  void onFindFocusToggled();

  void onGetDepthsClicked();
  void whenGetDepthsToggled(bool gettingDepths);
  void whenViewDepthsToggled(bool viewingDepths);

  void openSettingsDialog(); // Add a method to open the settings dialog

  struct Private *priv;

  ScaleWidget gainScale, exposeScale, gammaScale, frameSlider, waitScale,
      recordingSizeScale, bestFocusScale, homePositionScale, pGainScale;
  Gtk::Button recordButton, backToStartButton, pauseButton, playButton,
      fileLoadButton, fileSaveButton, settingsButton, resetButton, enterButton,
      recenterButton;
  Gtk::ToggleButton scaleBarToggle;
  Gtk::ToggleButton getDepthsToggle;
  Gtk::ToggleButton liveToggle, makeMapToggle, stabiliseToggle, showMapToggle,
      holdFocusToggle, threedStabToggle, twodStabToggle, findFocusToggle,
      viewDepthsToggle;
  Gtk::Label fpsLabel, loadSaveLabel, errorLabel, outOfBoundsWarningLabel;
  Gtk::Entry frameRateEntry;

  VidFrame *drawFrame;
  bool newDrawFrame;
  int countFrames;

  Condition makeMapActive, stabiliseActive, showMapActive, holdFocusActive,
      threedStabActive, twodStabActive, hasBuffer, liveView, loading, saving,
      playingBuffer, seeking, recording, pausedRecording, trackingFPS,
      viewDepthsActive, getDepthsActive, scaleBarActive;

  SignalFrameDrawn sigFrameDrawn;
  SignalFeatureUpdated sigFeatureUpdated;
  // SignalThresholdChanged sigThresholdChanged;
  // SignalScaleChanged sigScaleChanged;
  SignalBestFocusChanged sigBestFocusChanged;
  SignalPauseClicked sigPauseClicked;
  SignalEnterClicked sigEnterClicked;
  SignalFindFocusClicked sigFindFocusClicked;
  SignalResetClicked sigResetClicked;
  SignalScaleBarToggled sigScaleBarToggled;
  SignalRecenterClicked sigRecenterClicked;
  SignalGetDepthsClicked sigGetDepthsClicked;
  sigc::connection gainScaleConnection, exposeScaleConnection,
      gammaScaleConnection, frameRateScaleConnection, frameSliderConnection,
      stateChangeConnection;

  std::unordered_map<std::string, RenderFilter *> renderFilters;

  SignalHomePositionChanged sigHomePositionChanged;
  sigc::connection homePositionScaleConnection;

  SignalPGainChanged sigPGainChanged;
  sigc::connection pGainScaleConnection;

  SignalSettingsChanged sigSettingsChanged;

  Gtk::Label sharpnessLabel;
  SharpnessGraph sharpnessGraph;

  // Sophia: Added file title label
  Gtk::Label fileTitle;

  // Gtk::Button settingsButton; // Add a button for settings

  Settings settings; // Add a Settings object to manage application settings
};

#endif
