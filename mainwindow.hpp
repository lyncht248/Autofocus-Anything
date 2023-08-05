#ifndef HVIGTK_MAINWINDOW_H
#define HVIGTK_MAINWINDOW_H

#include <gtkmm.h>
#include <cairomm/cairomm.h>
#include <unordered_map>

#include "cond.hpp"
#include "gtkmm/drawingarea.h"
#include "sigc++/connection.h"
#include "vidframe.hpp"

class ScaleWidget : public Gtk::Bin
{
public: 
    ScaleWidget(double lower, double upper, double inc, double def, int spinButtonWidth=0, int scaleWidth=0, bool stepSnap=false);
    virtual ~ScaleWidget();
    
    using SignalChanged = sigc::signal<void(double)>;
    SignalChanged signalChanged();

    void setSpinButtonPrec(int digits);
    void setSpinButtonWidth(int width);
    void setScaleSizeRequest(int width, int height);
    
    double getValue() const;
    void setValue(double v);
	void setUpperLimit(double value);
private:
    void spinButtonChanged();
    void scaleChanged();
    
    SignalChanged sigChanged;
    
    Gtk::Scale scale;
    Gtk::SpinButton spinButton;
    Gtk::Grid grid;
    
    sigc::connection spinButtonConnection, scaleConnection;
	bool stepSnap;
};

class RenderFilter
{
public:
	virtual void draw(const ::Cairo::RefPtr< ::Cairo::Context>& cr) = 0;
};


/*
This class is responsible for the GTK GUI buttons, logic, etc., and is a derived class from GTK::Window
*/
class MainWindow : public Gtk::Window
{
public: 
    struct Private;
    MainWindow();
    virtual ~MainWindow();

	double getFrameRateScaleValue() const;
	double getStabWaitScaleValue() const;
	double getRecordingSizeScaleValue() const;
	double getBestFocusScaleValue() const;

	void setBestFocusScaleValue(double v);

	void getDisplayDimensions(double &w, double &h) const;

	void updateCameraValues(double gain, double expose, double gamma);

	void displayMessage(const std::string &msg);

	void renderFrame(VidFrame *frame);

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

    using SignalPauseClicked = sigc::signal<void()>;
    SignalPauseClicked signalPauseClicked();

	using SignalFindFocusClicked = sigc::signal<void()>;
    SignalFindFocusClicked signalFindFocusClicked();

	using SignalResetClicked = sigc::signal<void()>;
    SignalResetClicked signalResetClicked();

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

	Condition& getMakeMapActive();
	Condition& getStabiliseActive();
	Condition& getShowMapActive();
	Condition& getFindFocusActive();
	Condition& getHoldFocusActive();
	Condition& get3DStabActive();
	Condition& get2DStabActive();
	Condition& getHasBuffer();
	Condition& getLiveView();
	Condition& getLoading();
	Condition& getSaving();
	Condition& getPlayingBuffer();
	Condition& getSeeking();
	Condition& getRecording();
	Condition& getPausedRecording();

	int getFrameSliderValue() const;
	std::string getFileLocation() const;

	void addRenderFilter(const std::string &key, RenderFilter *filter);
	RenderFilter* removeRenderFilter(const std::string &key);
	
protected:
	virtual void on_realize() override;
	virtual void on_show() override;

	bool _on_state_event(GdkEventWindowState* window_state_event);

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

	void whenHoldFocusToggled(bool holdingFocus);
	void when3DStabToggled(bool active);
	void when2DStabToggled(bool active2);
    void onFindFocusClicked(); 
    void onResetClicked(); 


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

	void onThresScaleChange(double val);
	void onScaleScaleChange(double val);
	void onRecordingSizeScaleChange(double val);
	void onBestFocusScaleChange(double val);
    
    struct Private *priv;
    
    ScaleWidget gainScale, exposeScale, gammaScale, frameRateScale, frameSlider, thresScale, scaleScale, waitScale, recordingSizeScale, bestFocusScale;
    Gtk::Button recordButton, backToStartButton, pauseButton, playButton, fileLoadButton, fileSaveButton;
    Gtk::Entry fileNameEntry;
    Gtk::FileChooserButton fileChooseButton;
    Gtk::ToggleButton liveToggle, makeMapToggle, stabiliseToggle, showMapToggle, holdFocusToggle, threedStabToggle, twodStabToggle;
	Gtk::Button findFocusButton, resetButton;

    Gtk::Label fpsLabel;


	VidFrame *drawFrame;
	bool newDrawFrame;
	int countFrames;
    
    Condition makeMapActive, stabiliseActive, showMapActive, findFocusActive, holdFocusActive, threedStabActive, twodStabActive,
			  hasBuffer, liveView, loading, saving, playingBuffer, seeking, recording, pausedRecording, trackingFPS;

	SignalFrameDrawn sigFrameDrawn;
	SignalFeatureUpdated sigFeatureUpdated;
	SignalThresholdChanged sigThresholdChanged;
	SignalScaleChanged sigScaleChanged;
	SignalBestFocusChanged sigBestFocusChanged;
	SignalPauseClicked sigPauseClicked;
	SignalFindFocusClicked sigFindFocusClicked;
	SignalResetClicked sigResetClicked;

	sigc::connection gainScaleConnection, exposeScaleConnection, gammaScaleConnection, frameRateScaleConnection, frameSliderConnection, stateChangeConnection;

	std::unordered_map<std::string, RenderFilter*> renderFilters;
};

#endif
