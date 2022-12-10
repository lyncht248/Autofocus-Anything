#ifndef HVIGTK_MAINWINDOW_H
#define HVIGTK_MAINWINDOW_H

#include <gtkmm.h>
#include <cairomm/cairomm.h>

#include "cond.hpp"
#include "gtkmm/drawingarea.h"
#include "gtkmm/glarea.h"
#include "sigc++/connection.h"
#include "vidframe.hpp"

#include "stabiliser.hpp"
//Defines the slider widget
class ScaleWidget : public Gtk::Bin
{
public: 
	//Creates the widget
    ScaleWidget(double lower, double upper, double inc, double def);

	//Destroys the widget
    virtual ~ScaleWidget();
    
    using SignalChanged = sigc::signal<void(double)>;
    SignalChanged signalChanged();

	using SignalUserChanged = sigc::signal<void(Gtk::ScrollType scroll, double newValue)>;
	SignalUserChanged signalUserChanged();
    
    void setSpinButtonPrec(int digits);
    void setSpinButtonWidth(int width);
    void setScaleSizeRequest(int width, int height);
    
	//Functions that get and set value from slider
    double getValue() const;
    void setValue(double v);
private:
    void spinButtonChanged();
    void scaleChanged();
	bool changeScale(Gtk::ScrollType scroll, double newValue);
    
    SignalChanged sigChanged;
	SignalUserChanged sigUserChanged;
    
    Gtk::Scale scale;
    Gtk::SpinButton spinButton;
    Gtk::Grid grid;
    
    sigc::connection spinButtonConnection, scaleConnection;
};

class MainWindow : public Gtk::Window
{
public: 
    struct Private;
	// Lays out the GUI
    MainWindow();
    virtual ~MainWindow();

	// Call to update the Camera values
	void updateCameraValues(double gain, double expose, double gamma);

	// For the fps feature
	void displayMessage(const std::string &msg);

	// Call for renderDisplay, which puts the camera frame in the GUI
	void renderFrame(VidFrame *frame);

	// Signals used to cause logic changes in system.cpp
    using SignalFrameDrawn = sigc::signal<void()>;
    SignalFrameDrawn signalFrameDrawn();

	using SignalFeatureUpdated = sigc::signal<void(std::string, double)>;
	SignalFeatureUpdated signalFeatureUpdated();

	using SignalThresholdChanged = sigc::signal<void(double)>;
	SignalThresholdChanged signalThresholdChanged();

	using SignalScaleChanged = sigc::signal<void(double)>;
	SignalScaleChanged signalScaleChanged();

	using SignalBestFocusChanged = sigc::signal<void(double)>;
	SignalBestFocusChanged signalBestFocusChanged();

	void setHasBuffer(bool val);
	void setLiveView(bool val);
	void setLoading(bool val);
	void setSaving(bool val);
	void setPlayingBuffer(bool val);
	void setSeeking(bool val);
	void setRecording(bool val);

	Condition& getMakeMapActive();
	Condition& getStabiliseActive();
	Condition& getShowMapActive();
	Condition& getFindFocusActive();
	Condition& getHoldFocusActive();
	Condition& get3DStabActive();
	Condition& getHasBuffer();
	Condition& getLiveView();
	Condition& getLoading();
	Condition& getSaving();
	Condition& getPlayingBuffer();
	Condition& getSeeking();
	Condition& getRecording();

	int getFrameSliderValue() const;
	std::string getFileLocation() const;
    
private:
	/*
	 * This is the function which will draw the currently loaded frame.
	 * Currently the Cairo library is used. To trigger this function
	 * call MainWindow::renderFrame(...).
	 */
	bool renderDisplay(const ::Cairo::RefPtr< ::Cairo::Context>& cr);

	void onFrameDrawn();
	bool updateFPSCounter();

	void whenMakeMapToggled(bool makingMap);
	void whenStabiliseToggled(bool stabilising);
	void whenShowMapToggled(bool showingMap);

	void whenFindFocusToggled(bool findingFocus);
	void whenHoldFocusToggled(bool holdingFocus);
	void when3DStabToggled(bool active);
    
    void bufferFilled();
    void bufferEmptied();
    
    void viewingLive();
    void viewingBuffer();

	void whenLoadingToggled(bool loading);
	void whenSavingToggled(bool saving);
	void whenRecordingToggled(bool recording);
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

	void onThresScaleChange(double val);
	void onScaleScaleChange(double val);
	void onBestFocusScaleChange(double val);
    
    struct Private *priv;
    
    ScaleWidget gainScale, exposeScale, gammaScale, frameSlider, thresScale, scaleScale, bestFocusScale;
    Gtk::Button recordButton, backToStartButton, pauseButton, playButton, fileLoadButton, fileSaveButton;
    Gtk::Entry fileNameEntry;
    Gtk::FileChooserButton fileChooseButton;
    Gtk::ToggleButton liveToggle, makeMapToggle, stabiliseToggle, showMapToggle, findFocusToggle, holdFocusToggle, tdStabToggle;
    Gtk::Label fpsLabel;


	Gtk::DrawingArea display;
	int displayW, displayH;
	VidFrame *drawFrame;
	CVD::Image<unsigned char> im;
	bool newDrawFrame;
	int countFrames;
    
    Condition makeMapActive, stabiliseActive, showMapActive, findFocusActive, holdFocusActive, tdStabActive, 
			  hasBuffer, liveView, loading, saving, playingBuffer, seeking, recording, trackingFPS;

	SignalFrameDrawn sigFrameDrawn;
	SignalFeatureUpdated sigFeatureUpdated;
	SignalThresholdChanged sigThresholdChanged;
	SignalScaleChanged sigScaleChanged;
	SignalBestFocusChanged sigBestFocusChanged;

	sigc::connection gainScaleConnection, exposeScaleConnection, gammaScaleConnection, frameSliderConnection;

	Stabiliser my_stabiliser; 
	TooN::Vector<2> offset;

};

#endif
