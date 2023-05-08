#include "mainwindow.hpp"

#include <cvd/image.h>
#include <gtkmm.h>
#include <string>
#include <chrono>
#include <thread>

#include "VimbaC/Include/VmbCommonTypes.h"
#include "cairomm/context.h"
#include "cairomm/enums.h"
#include "gdkmm/rectangle.h"
#include "glibmm/main.h"
#include "gtkmm/adjustment.h"
#include "gtkmm/enums.h"
#include "gtkmm/requisition.h"
#include "sdlwindow.hpp"
#include "sigc++/functors/mem_fun.h"
#include "version.hpp"
#include "main.hpp"
#include "logfile.hpp"
#include "autofocus.hpp"

struct MainWindow::Private
{
    Gtk::Label gainLabel, exposeLabel, gammaLabel, frameRateLabel, thresLabel, scaleLabel, recordingSizeLabel, bestFocusLabel;
	//Gtk::Label waitLabel; //This was used to time-out the stabilizer.cc code, but is deprecated now
	Gtk::Frame controlFrame;
    Gtk::Grid controlGrid;
    Gtk::VBox rootBox;
    
    Gtk::Label space4[16];
	//Gtk::Label verticalLine[8];
	Gtk::Label autofocusTitle, stabilizationTitle, playbackTitle, cameraSettingsTitle, fileTitle;
	Gtk::Separator verticalSeparator1, verticalSeparator2, verticalSeparator3, verticalSeparator4;
    
    Gtk::HBox mediaBox, fileChooserBox;
	Gtk::Box displayBox;
    
    public:
        Private();
};

// For GUI elements that are static
MainWindow::Private::Private() :
    gainLabel("Gain: "),
    exposeLabel("Expose: "),
    gammaLabel("Gamma: "),
    frameRateLabel("Frame rate: "),
    thresLabel("Threshold: "),
    scaleLabel("Scale: "),
	//waitLabel("Optimize: "),
	recordingSizeLabel("Rec. Size: "),
    bestFocusLabel("Best-Focal Plane"),
	autofocusTitle("Autofocus"),
	stabilizationTitle("2D Stabilization"),
	playbackTitle("Playback Controls"),
	cameraSettingsTitle("Camera Settings"),
	fileTitle("File Load and Save"),
	controlFrame(),
    controlGrid(),
	verticalSeparator1(),
	verticalSeparator2(),
	verticalSeparator3(),
	verticalSeparator4(),
    rootBox(),
    space4{Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    ")},
    //verticalLine{Gtk::Label(" | "), Gtk::Label("  |  "), Gtk::Label("  |  "), Gtk::Label("  |  "), Gtk::Label("  |  "), Gtk::Label("  |  "), Gtk::Label("  |  "), Gtk::Label("  |  ")},
	mediaBox(Gtk::Orientation::ORIENTATION_HORIZONTAL),
    fileChooserBox(),
	displayBox()
{
    gainLabel.set_justify(Gtk::Justification::JUSTIFY_RIGHT);
    gainLabel.set_halign(Gtk::Align::ALIGN_END);
    exposeLabel.set_justify(Gtk::Justification::JUSTIFY_RIGHT);
    exposeLabel.set_halign(Gtk::Align::ALIGN_END);
    gammaLabel.set_justify(Gtk::Justification::JUSTIFY_RIGHT);
    gammaLabel.set_halign(Gtk::Align::ALIGN_END);
    frameRateLabel.set_justify(Gtk::Justification::JUSTIFY_RIGHT);
    frameRateLabel.set_halign(Gtk::Align::ALIGN_END);
    thresLabel.set_justify(Gtk::Justification::JUSTIFY_RIGHT);
    thresLabel.set_halign(Gtk::Align::ALIGN_END);
    scaleLabel.set_justify(Gtk::Justification::JUSTIFY_RIGHT);
    scaleLabel.set_halign(Gtk::Align::ALIGN_END);
    // waitLabel.set_justify(Gtk::Justification::JUSTIFY_RIGHT);
    // waitLabel.set_halign(Gtk::Align::ALIGN_END);
   	recordingSizeLabel.set_justify(Gtk::Justification::JUSTIFY_RIGHT);
    recordingSizeLabel.set_halign(Gtk::Align::ALIGN_END);
    
    controlGrid.set_hexpand();
    controlGrid.set_halign(Gtk::Align::ALIGN_FILL);
    //controlGrid.set_row_spacing(10);
    controlGrid.set_row_spacing(3);

    rootBox.set_hexpand();
    rootBox.set_halign(Gtk::Align::ALIGN_FILL);

    //rootBox.set_vexpand();
    //rootBox.set_valign(Gtk::Align::ALIGN_FILL);
    
    for (Gtk::Label &label : space4)
        label.set_justify(Gtk::Justification::JUSTIFY_CENTER);
	    
    mediaBox.set_hexpand(false);
    mediaBox.set_halign(Gtk::Align::ALIGN_CENTER);

   	autofocusTitle.set_justify(Gtk::Justification::JUSTIFY_CENTER);
    autofocusTitle.set_halign(Gtk::Align::ALIGN_CENTER);
   	stabilizationTitle.set_justify(Gtk::Justification::JUSTIFY_CENTER);
    stabilizationTitle.set_halign(Gtk::Align::ALIGN_CENTER);
   	playbackTitle.set_justify(Gtk::Justification::JUSTIFY_CENTER);
    playbackTitle.set_halign(Gtk::Align::ALIGN_CENTER);
   	cameraSettingsTitle.set_justify(Gtk::Justification::JUSTIFY_CENTER);
    cameraSettingsTitle.set_halign(Gtk::Align::ALIGN_CENTER);
   	fileTitle.set_justify(Gtk::Justification::JUSTIFY_CENTER);
    fileTitle.set_halign(Gtk::Align::ALIGN_CENTER);

	autofocusTitle.set_margin_top(6);
	stabilizationTitle.set_margin_top(6);
	playbackTitle.set_margin_top(6);
	cameraSettingsTitle.set_margin_top(6);
	fileTitle.set_margin_top(6);

	autofocusTitle.set_margin_bottom(6);
	stabilizationTitle.set_margin_bottom(6);
	playbackTitle.set_margin_bottom(6);
	cameraSettingsTitle.set_margin_bottom(6);
	fileTitle.set_margin_bottom(6);

	// verticalLine[0].override_color (Gdk::RGBA("grey"), Gtk::STATE_FLAG_NORMAL);
	// verticalLine[1].override_color (Gdk::RGBA("grey"), Gtk::STATE_FLAG_NORMAL);
	// verticalLine[2].override_color (Gdk::RGBA("grey"), Gtk::STATE_FLAG_NORMAL);
	// verticalLine[3].override_color (Gdk::RGBA("grey"), Gtk::STATE_FLAG_NORMAL);
	// verticalLine[4].override_color (Gdk::RGBA("grey"), Gtk::STATE_FLAG_NORMAL);
	// verticalLine[5].override_color (Gdk::RGBA("grey"), Gtk::STATE_FLAG_NORMAL);
	// verticalLine[6].override_color (Gdk::RGBA("grey"), Gtk::STATE_FLAG_NORMAL);
	// verticalLine[7].override_color (Gdk::RGBA("grey"), Gtk::STATE_FLAG_NORMAL);
	// verticalLine[8].override_color (Gdk::RGBA("grey"), Gtk::STATE_FLAG_NORMAL);

    verticalSeparator1.set_orientation(Gtk::ORIENTATION_VERTICAL);
    verticalSeparator2.set_orientation(Gtk::ORIENTATION_VERTICAL);
    verticalSeparator3.set_orientation(Gtk::ORIENTATION_VERTICAL);
    verticalSeparator4.set_orientation(Gtk::ORIENTATION_VERTICAL);
	
	verticalSeparator1.override_color (Gdk::RGBA("black"), Gtk::STATE_FLAG_NORMAL);
	verticalSeparator2.override_color (Gdk::RGBA("black"), Gtk::STATE_FLAG_NORMAL);
	verticalSeparator3.override_color (Gdk::RGBA("black"), Gtk::STATE_FLAG_NORMAL);
	verticalSeparator4.override_color (Gdk::RGBA("black"), Gtk::STATE_FLAG_NORMAL);


	/*
	displayBox.set_hexpand(true);
	displayBox.set_vexpand(true);
	displayBox.set_halign(Gtk::Align::ALIGN_FILL);
	displayBox.set_valign(Gtk::Align::ALIGN_FILL);
	*/

	/*
	Glib::RefPtr<Gtk::CssProvider> cssBlackBG = Gtk::CssProvider::create();
	cssBlackBG->load_from_data("* {background-color: #000000;}");
	displayBox.get_style_context()->add_provider(cssBlackBG, 0);
	*/
}

ScaleWidget::ScaleWidget(double lower, double upper, double inc, double def, int spinButtonWidth, int scaleWidth, bool stepSnap) : Gtk::Bin(),
    sigChanged(),
    scale(),
    spinButton(),
    grid(),
	stepSnap(stepSnap)
{
	if (stepSnap)
	{
    	scale.set_range(lower, (inc*lower-lower+upper)/inc);
    	scale.set_increments(1, 10);
		scale.set_round_digits(0);
		scale.set_digits(0);
    	scale.set_value( (inc*lower-lower+def)/inc);
	}
	else
	{
    	scale.set_range(lower, upper);
    	scale.set_increments(inc, inc*10);
    	scale.set_value(def);
	}

    scale.set_draw_value(false);
    scaleConnection = scale.signal_value_changed().connect(sigc::mem_fun(*this, &ScaleWidget::scaleChanged) );
	if (scaleWidth)
		scale.set_size_request(scaleWidth);

    spinButton.set_range(lower, upper);
    spinButton.set_increments(inc, inc*10);
	spinButton.set_snap_to_ticks(stepSnap);
    spinButton.set_value(def);
    spinButtonConnection = spinButton.signal_value_changed().connect(sigc::mem_fun(*this, &ScaleWidget::spinButtonChanged) );
	if (spinButtonWidth)
		spinButton.set_width_chars(spinButtonWidth);
	scale.set_slider_size_fixed();
    
    grid.attach(spinButton, 0, 0);
    grid.attach(scale, 1, 0);
    
    add(grid);
}

ScaleWidget::~ScaleWidget()
{
};

ScaleWidget::SignalChanged ScaleWidget::signalChanged()
{
    return sigChanged;
}

void ScaleWidget::setSpinButtonPrec(int digits)
{
    spinButton.set_digits(digits);
}

void ScaleWidget::setSpinButtonWidth(int width)
{
    spinButton.set_width_chars(width);
}

void ScaleWidget::setScaleSizeRequest(int width, int height)
{
    scale.set_size_request(width, height);
}

double ScaleWidget::getValue() const
{
    return spinButton.get_value();
}

void ScaleWidget::setValue(double v)
{
	spinButton.set_value(v);
}

void ScaleWidget::setUpperLimit(double value)
{
	double old = spinButton.get_value();
	double min, max;
	spinButtonConnection.block();
	spinButton.get_range(min, max);
	spinButton.set_range(min, value);
	spinButtonConnection.unblock();

	scaleConnection.block();
	auto a = scale.get_adjustment();
	if (stepSnap)
	{
    	scale.set_range(min, (a->get_step_increment()*min-min+value)/a->get_step_increment() );
		scale.set_value(min + (spinButton.get_value() - min)*a->get_step_increment() );
	}
	else
	{
		scale.set_range(min, value);
		scale.set_value(spinButton.get_value() );
	}
	scaleConnection.unblock();
	if (spinButton.get_value() != old)
		sigChanged.emit(spinButton.get_value() );
}

void ScaleWidget::spinButtonChanged()
{
    scaleConnection.block();
    scale.set_value(spinButton.get_value() );
    scaleConnection.unblock();
    sigChanged.emit(spinButton.get_value() );
}

void ScaleWidget::scaleChanged()
{
    spinButtonConnection.block();
	if (stepSnap)
	{
		auto adj = scale.get_adjustment();
		double lower = adj->get_lower();
		int value = scale.get_value();
		double inc, page;
		spinButton.get_increments(inc, page);
    	spinButton.set_value(lower + (value - lower)*inc);
	}
	else
    	spinButton.set_value(scale.get_value() );
    spinButtonConnection.unblock();
    sigChanged.emit(scale.get_value() );
}

bool MainWindow::_on_state_event(GdkEventWindowState* window_state_event)
{
	if (!(window_state_event->new_window_state & (GdkWindowState::GDK_WINDOW_STATE_WITHDRAWN | GdkWindowState::GDK_WINDOW_STATE_ICONIFIED) ) )
	{
		SDLWindow::raise(childwin);
	}
	else
	{
		SDLWindow::hide(childwin);
	}

	if (window_state_event->new_window_state & GdkWindowState::GDK_WINDOW_STATE_FOCUSED)
	{
		SDLWindow::raise(childwin);
	}
	else
	{
		SDLWindow::unraise(childwin);
	}

	return false;
}

MainWindow::MainWindow() : Gtk::Window(),
    priv(new Private() ),
    gainScale(0, 50, 1, 0, 7, 150),
    exposeScale(100, 50000, 1000, 15000, 7, 150),
    gammaScale(0.5, 2.5, 0.01, 1, 7, 150),
	frameRateScale(20, 80, 5, 60, 7, 150, true),
    frameSlider(0, 1799, 1, 0, 5, 200),
    thresScale(0, 1, 0.01, 0.4, 6, 100),
    scaleScale(0, 5, 0.01, 2.0, 6, 100),
    waitScale(0, HVIGTK_STAB_LIM, 100, 0, 6, 100),
	recordingSizeScale(900, 3600, 10, 900, 6, 100),
    bestFocusScale(1, 320, 1, 200, 4, 100),
    recordButton(),
    backToStartButton(),
    pauseButton(),
    playButton(),
    fileLoadButton("Load"),
    fileSaveButton("Save"),
    fileNameEntry(),
    fileChooseButton(),
    liveToggle(),
    makeMapToggle("Make Map"),
    stabiliseToggle("Stabilise"),
    showMapToggle("Show Map"),
    findFocusToggle("Find Focus"),
    holdFocusToggle("Hold Focus"),
    tdStabToggle("3D Stab."),
    fpsLabel(""),
	//display(*this),
	drawFrame(nullptr),
	newDrawFrame(false),
	countFrames(),
	makeMapActive(),
	stabiliseActive(),
    showMapActive(),
    findFocusActive(),
	holdFocusActive(),
	tdStabActive(),
    hasBuffer(),
    liveView(),
	loading(),
	saving(),
	playingBuffer(),
	seeking(),
	recording(),
	pausedRecording(),
	trackingFPS(),
	sigFrameDrawn(),
	sigFeatureUpdated(),
	sigThresholdChanged(),
	sigScaleChanged(),
	sigBestFocusChanged(),
	gainScaleConnection(),
	exposeScaleConnection(),
	gammaScaleConnection(),
	frameRateScaleConnection(),
	frameSliderConnection(),
	stateChangeConnection()
{
    //set_default_size(1700,200);
    set_title("HVI-GTK " HVIGTK_VERSION_STR);
    add_events(Gdk::STRUCTURE_MASK);
	stateChangeConnection = signal_window_state_event().connect(sigc::mem_fun(*this, &MainWindow::_on_state_event) );
    //gainScale.setScaleSizeRequest(150, 0);
    //gainScale.setSpinButtonWidth(7);
	gainScaleConnection = gainScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onGainScaleChange) );
    
    //exposeScale.setScaleSizeRequest(150, 0);
    //exposeScale.setSpinButtonWidth(7);
	exposeScaleConnection = exposeScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onExposeScaleChange) );
    
    //gammaScale.setScaleSizeRequest(150, 0);
    //gammaScale.setSpinButtonWidth(7);
    gammaScale.setSpinButtonPrec(2);
	gammaScaleConnection = gammaScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onGammaScaleChange) );

	frameRateScaleConnection = frameRateScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onFrameRateChange) );
    
    recordButton.set_image_from_icon_name("media-record");
    recordButton.set_tooltip_text("Begin recording");
    recordButton.set_hexpand(false);
    recordButton.set_valign(Gtk::Align::ALIGN_FILL);
    recordButton.set_halign(Gtk::Align::ALIGN_START);
	recordButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onRecordClicked) );
    
    backToStartButton.set_image_from_icon_name("media-skip-backward");
    backToStartButton.set_tooltip_text("Go back to start of recording");
    backToStartButton.set_hexpand(false);
    backToStartButton.set_valign(Gtk::Align::ALIGN_FILL);
    backToStartButton.set_halign(Gtk::Align::ALIGN_START);
	backToStartButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onBackButtonClicked) );

	backToStartButton.set_sensitive(false);
    
    pauseButton.set_image_from_icon_name("media-playback-pause");
    pauseButton.set_tooltip_text("Pause playback of recording");
    pauseButton.set_hexpand(false);
    pauseButton.set_valign(Gtk::Align::ALIGN_FILL);
    pauseButton.set_halign(Gtk::Align::ALIGN_START);
	pauseButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onPauseClicked) );
	pauseButton.set_sensitive(false);
    
    playButton.set_image_from_icon_name("media-playback-start");
    playButton.set_tooltip_text("Start/Resume playback of recording");
    playButton.set_hexpand(false);
    playButton.set_valign(Gtk::Align::ALIGN_FILL);
    playButton.set_halign(Gtk::Align::ALIGN_START);
	playButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onPlayButtonClicked) );
	playButton.set_sensitive(false);
    
    liveToggle.set_image_from_icon_name("camera-web");
    liveToggle.set_tooltip_text("Switch to/from live camera feed");
    liveToggle.set_hexpand(false);
    liveToggle.set_valign(Gtk::Align::ALIGN_FILL);
    liveToggle.set_halign(Gtk::Align::ALIGN_START);
	liveToggle.signal_toggled().connect(sigc::mem_fun(*this, &MainWindow::onLiveToggled) );
	liveToggle.set_sensitive(false);
    
    priv->mediaBox.add(recordButton);
    priv->mediaBox.add(backToStartButton);
    priv->mediaBox.add(pauseButton);
    priv->mediaBox.add(playButton);
    priv->mediaBox.add(liveToggle);
    
    //frameSlider.setScaleSizeRequest(200, 0);
    //frameSlider.setSpinButtonWidth(5);
	frameSliderConnection = frameSlider.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onFrameSliderChange) );
    
    fileNameEntry.set_width_chars(16);
    
    fileChooseButton.set_tooltip_text("Choose another location");
    fileChooseButton.set_hexpand(false);
    fileChooseButton.set_valign(Gtk::Align::ALIGN_FILL);
    fileChooseButton.set_halign(Gtk::Align::ALIGN_START);
    fileChooseButton.set_action(Gtk::FileChooserAction::FILE_CHOOSER_ACTION_SELECT_FOLDER);
    fileChooseButton.set_filename(hvigtk_startdir);
    
    fileLoadButton.set_tooltip_text("Load frames from this location");
	fileLoadButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onLoadButtonClicked) );

    fileSaveButton.set_tooltip_text("Save frames to this location");
    fileSaveButton.set_sensitive(false);
	fileSaveButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onSaveButtonClicked) );
    
    stabiliseToggle.set_sensitive(false);
    showMapToggle.set_sensitive(false);
	showMapToggle.signal_clicked();
    
    //thresScale.setScaleSizeRequest(100, 0);
    //thresScale.setSpinButtonWidth(4);
    thresScale.setSpinButtonPrec(2);
	thresScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onThresScaleChange) );
    
    //scaleScale.setScaleSizeRequest(100, 0);
    //scaleScale.setSpinButtonWidth(4);
    scaleScale.setSpinButtonPrec(2);
	scaleScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onScaleScaleChange) );

	waitScale.set_tooltip_text("Optimize the stabiliser (0: no optimization; 100,000: max optimization)");

	recordingSizeScale.set_tooltip_text("Set the maximum number of frames for a single recording");
	recordingSizeScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onRecordingSizeScaleChange) );
    
    holdFocusToggle.set_sensitive(true);
    tdStabToggle.set_sensitive(true);
	bestFocusScale.set_sensitive(false);
    
    //bestFocusScale.setScaleSizeRequest(100, 0);
    //bestFocusScale.setSpinButtonWidth(4);
	bestFocusScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onBestFocusScaleChange) );
//
	//display.set_hexpand(true);
	//display.set_vexpand(true);
	//display.set_halign(Gtk::Align::ALIGN_FILL);
	//display.set_valign(Gtk::Align::ALIGN_FILL);
	
	//display.signal_draw().connect(sigc::mem_fun(*this, &MainWindow::renderDisplay) );

	setLiveView(true);

    //CONDITIONS 

	makeMapActive.toggleOnSignal(makeMapToggle.signal_toggled() );
    makeMapActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenMakeMapToggled) );

	stabiliseActive.toggleOnSignal(stabiliseToggle.signal_toggled() );
	stabiliseActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenStabiliseToggled) );

	showMapActive.toggleOnSignal(showMapToggle.signal_toggled() );
	showMapActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenShowMapToggled) );
	
	// findFocusActive.toggleOnSignal(findFocusToggle.signal_toggled() );
	// findFocusActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenFindFocusToggled) );

	findFocusToggle.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_button_clicked));
    findFocusToggle.set_tooltip_text("Finds image with highest sharpness");

	holdFocusActive.toggleOnSignal(holdFocusToggle.signal_toggled() );
	holdFocusActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenHoldFocusToggled) );

	tdStabActive.toggleOnSignal(tdStabToggle.signal_toggled() );
	tdStabActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::when3DStabToggled) );

	hasBuffer.signalTrue().connect(sigc::mem_fun(*this, &MainWindow::bufferFilled) );
	hasBuffer.signalFalse().connect(sigc::mem_fun(*this, &MainWindow::bufferEmptied) );

	Condition &viewing = !liveView && hasBuffer;

	viewing.signalFalse().connect(sigc::mem_fun(*this, &MainWindow::viewingLive) );
	viewing.signalTrue().connect(sigc::mem_fun(*this, &MainWindow::viewingBuffer) );

	loading.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenLoadingToggled) );
	saving.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenSavingToggled) );

	recording.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenRecordingToggled) );
	pausedRecording.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenPausedRecordingToggled) );

	trackingFPS.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenTrackingFPSToggled) );
	
	playingBuffer.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenPlayingBufferToggled) );

	sigFrameDrawn.connect(sigc::mem_fun(*this, &MainWindow::onFrameDrawn) );

	//ATTACHING

    priv->controlGrid.attach(priv->space4[0], 0, 0);

	priv->controlGrid.attach(priv->autofocusTitle, 1, 3, 3);
    priv->controlGrid.attach(findFocusToggle, 1, 0);
    priv->controlGrid.attach(holdFocusToggle, 1, 1);
    priv->controlGrid.attach(tdStabToggle, 1, 2);

    priv->controlGrid.attach(priv->space4[1], 2, 0);

    priv->controlGrid.attach(priv->bestFocusLabel, 3, 0);
    priv->controlGrid.attach(bestFocusScale, 3, 1);

	priv->controlGrid.attach(priv->space4[2], 4, 0);
    priv->controlGrid.attach(priv->verticalSeparator1, 5, 0, 1, 4);
	priv->controlGrid.attach(priv->space4[3], 6, 0);

	priv->controlGrid.attach(priv->stabilizationTitle, 7, 3, 4);
    priv->controlGrid.attach(makeMapToggle, 7, 0);
    priv->controlGrid.attach(stabiliseToggle, 7, 1);
    priv->controlGrid.attach(showMapToggle, 7, 2);

    priv->controlGrid.attach(priv->space4[4], 8, 0);

    priv->controlGrid.attach(priv->thresLabel, 9, 0);
    priv->controlGrid.attach(priv->scaleLabel, 9, 1);
    priv->controlGrid.attach(priv->recordingSizeLabel, 9, 2);

    priv->controlGrid.attach(thresScale, 10, 0);
    priv->controlGrid.attach(scaleScale, 10, 1);
    priv->controlGrid.attach(recordingSizeScale, 10, 2);

	priv->controlGrid.attach(priv->space4[5], 11, 0);
	priv->controlGrid.attach(priv->verticalSeparator2, 12, 0, 1, 4);
	priv->controlGrid.attach(priv->space4[6], 13, 0);

	priv->controlGrid.attach(priv->playbackTitle, 14, 3, 1);
    priv->controlGrid.attach(priv->mediaBox, 14, 0);
    priv->controlGrid.attach(frameSlider, 14, 1);
    priv->controlGrid.attach(fpsLabel, 14, 2);

	priv->controlGrid.attach(priv->space4[7], 15, 0);
	priv->controlGrid.attach(priv->verticalSeparator3, 16, 0, 1, 4);
	priv->controlGrid.attach(priv->space4[8], 17, 0);

	priv->controlGrid.attach(priv->cameraSettingsTitle, 18, 3, 2);
    priv->controlGrid.attach(priv->gainLabel, 18, 0);
    priv->controlGrid.attach(priv->exposeLabel, 18, 1);

    priv->controlGrid.attach(priv->gammaLabel, 23, 2);

	priv->controlGrid.attach(priv->frameRateLabel, 18, 2);
 
    priv->controlGrid.attach(gainScale, 19, 0);
    priv->controlGrid.attach(exposeScale, 19, 1);

    priv->controlGrid.attach(gammaScale, 24, 2);

	priv->controlGrid.attach(frameRateScale, 19, 2);

	priv->controlGrid.attach(priv->space4[9], 20, 0);
	priv->controlGrid.attach(priv->verticalSeparator4, 21, 0, 1, 4);
	priv->controlGrid.attach(priv->space4[10], 22, 0);

	priv->controlGrid.attach(priv->fileTitle, 23, 3, 2);
    priv->fileChooserBox.add(fileNameEntry);
    priv->fileChooserBox.add(fileChooseButton);
    priv->controlGrid.attach(priv->fileChooserBox, 23, 0, 2, 1);
    priv->controlGrid.attach(fileLoadButton, 23, 1);

    priv->controlGrid.attach(fileSaveButton, 24, 1);
    
    priv->controlGrid.attach(priv->space4[11], 25, 0);

    
    //priv->controlGrid.attach(priv->waitLabel, 7, 2);
    //priv->controlGrid.attach(waitScale, 8, 2);

	priv->controlFrame.add(priv->controlGrid);

	priv->controlFrame.set_vexpand(false);
	priv->controlFrame.set_valign(Gtk::Align::ALIGN_START);

    ///////////
            

    
    //priv->displayBox.pack_start(display, true, true);

    priv->rootBox.pack_start(priv->controlFrame, false, true);
    //priv->rootBox.pack_start(priv->displayBox, true, true);
	//priv->rootBox.add(display);
    
    add(priv->rootBox);
	resize(priv->rootBox.get_width(), priv->rootBox.get_height() );
    
    priv->rootBox.show_all();
}

double MainWindow::getFrameRateScaleValue() const
{
	return frameRateScale.getValue();
}

double MainWindow::getBestFocusScaleValue() const
{
	return bestFocusScale.getValue();
}

double MainWindow::getStabWaitScaleValue() const
{
	return waitScale.getValue();
}

double MainWindow::getRecordingSizeScaleValue() const
{
	return recordingSizeScale.getValue();
}

void MainWindow::setBestFocusScaleValue(double v) {
	bestFocusScale.setValue(v);
};

void MainWindow::updateCameraValues(double gain, double expose, double gamma)
{
	gainScaleConnection.block();
	exposeScaleConnection.block();
	gammaScaleConnection.block();

	gainScale.setValue(gain);
	exposeScale.setValue(expose);
	gammaScale.setValue(gamma);

	gainScaleConnection.unblock();
	exposeScaleConnection.unblock();
	gammaScaleConnection.unblock();
}

void MainWindow::getDisplayDimensions(double &w, double &h) const
{
	//w = display.get_width();
	//h = display.get_height();
	
	w = 800;
	h = 600;
}

void MainWindow::displayMessage(const std::string &msg)
{
	fpsLabel.set_text(msg);
}

void MainWindow::renderFrame(VidFrame *frame)
{
	newDrawFrame = frame != drawFrame;
	if (newDrawFrame)
	{
		drawFrame = frame;
		//display.queue_draw();
		
		CVD::ImageRef dim = frame->size();
		SDLWindow::renderFrameG8(childwin, frame->data(), dim.x * dim.y);
	}
	sigFrameDrawn.emit();
}

MainWindow::SignalFrameDrawn MainWindow::signalFrameDrawn()
{
	return sigFrameDrawn;
}

MainWindow::SignalFeatureUpdated MainWindow::signalFeatureUpdated()
{
	return sigFeatureUpdated;
}

MainWindow::SignalThresholdChanged MainWindow::signalThresholdChanged()
{
	return sigThresholdChanged;
}

MainWindow::SignalScaleChanged MainWindow::signalScaleChanged()
{
	return sigScaleChanged;
}

MainWindow::SignalBestFocusChanged MainWindow::signalBestFocusChanged()
{
	return sigBestFocusChanged;
}

MainWindow::SignalPauseClicked MainWindow::signalPauseClicked()
{
	return sigPauseClicked;
}

MainWindow::SignalFindFocusClicked MainWindow::signalFindFocusClicked()
{
	return sigFindFocusClicked;
}


void MainWindow::setHasBuffer(bool val)
{
	hasBuffer.setValue(val);
}

Condition& MainWindow::getLoading()
{
	return loading;
}

Condition& MainWindow::getSaving()
{
	return saving;
}

void MainWindow::setLiveView(bool val)
{
	liveToggle.set_active(val); //also sets condition
	//liveView.setValue(val);
}

void MainWindow::setLoading(bool val)
{
	loading.setValue(val);
}

void MainWindow::setSaving(bool val)
{
	saving.setValue(val);
}

void MainWindow::setPlayingBuffer(bool val)
{
	if (!val)
	{
		playingBuffer.setValue(seeking.getValue() );
	}
	seeking.setValue(false);
}

void MainWindow::setSeeking(bool val)
{
	seeking.setValue(val);
}

void MainWindow::setRecording(bool val)
{
	recording.setValue(val);
}

void MainWindow::setMakingMap(bool val)
{
	//makeMapActive.setValue(val);
	makeMapToggle.set_active(val);
}

void MainWindow::setShowingMap(bool val)
{
	showMapToggle.set_active(val);
}

Condition& MainWindow::getMakeMapActive()
{
	return makeMapActive;
}

Condition& MainWindow::getStabiliseActive()
{
	return stabiliseActive;
}

Condition& MainWindow::getShowMapActive()
{
	return showMapActive;
}

Condition& MainWindow::getFindFocusActive()
{
	return findFocusActive;
}

Condition& MainWindow::getHoldFocusActive()
{
	return holdFocusActive;
}

Condition& MainWindow::get3DStabActive()
{
	return tdStabActive;
}

Condition& MainWindow::getHasBuffer()
{
	return hasBuffer;
}

Condition& MainWindow::getLiveView()
{
	return liveView;
}

Condition& MainWindow::getPlayingBuffer()
{
	return playingBuffer;
}

Condition& MainWindow::getSeeking()
{
	return seeking;
}

Condition& MainWindow::getRecording()
{
	return recording;
}

Condition& MainWindow::getPausedRecording()
{
	return pausedRecording;
}

int MainWindow::getFrameSliderValue() const
{
	return frameSlider.getValue();
}

std::string MainWindow::getFileLocation() const
{
	std::string out = fileChooseButton.get_filename();
	if (out[out.size() - 1] == '/')
		return out + fileNameEntry.get_text();
	else
		return out + '/' + fileNameEntry.get_text();
}

void MainWindow::addRenderFilter(const std::string &key, RenderFilter *filter)
{
	renderFilters[key] = filter;
}

RenderFilter* MainWindow::removeRenderFilter(const std::string &key)
{
	if (renderFilters.count(key) )
	{
		RenderFilter *out = renderFilters[key];
		renderFilters.erase(key);
		return out;
	}
	return nullptr;
}

void MainWindow::on_realize()
{
	Gtk::Window::on_realize();
	int monIndex = get_screen()->get_monitor_at_window(get_window() );
	Gdk::Rectangle dim;
	get_screen()->get_monitor_geometry(monIndex, dim);
	move(dim.get_x() + (dim.get_width() - get_width() ) / 2, 100);
	
}

void MainWindow::on_show()
{
	Gtk::Window::on_show();
	std::this_thread::sleep_for(std::chrono::milliseconds(500) );
	Gdk::Rectangle dim, frameDim;
	int monIndex = get_screen()->get_monitor_at_window(get_window() );
	get_screen()->get_monitor_geometry(monIndex, dim);

	int x,y;
	get_window()->get_origin(x, y);

	//std::cout << x << " " << y << " " << x2 << " " << y2 << std::endl;

	SDLWindow::move(childwin, (dim.get_width() - 800) / 2, y + get_height() );
}
 
bool MainWindow::renderDisplay(const ::Cairo::RefPtr< ::Cairo::Context>& cr)
{
	/*
	if (drawFrame)
	{
		double xscale = (double) (display.get_width() ) / drawFrame->get_width();
		double yscale = (double) (display.get_height() ) / drawFrame->get_height();
		if (xscale < 1.0 || yscale < 1.0)
		{
			double arscale = drawFrame->get_width() > drawFrame->get_height() ? yscale : xscale;
			cr->scale(arscale, arscale);
		}

		cr->set_source(drawFrame, 0, 0);
		cr->paint();

		if (newDrawFrame)
		{
			newDrawFrame = false;
			sigFrameDrawn.emit();
		}
	}
	*/

	return true;
}

void MainWindow::onFrameDrawn()
{
	if (newDrawFrame && playingBuffer.getValue() )
	{
		frameSliderConnection.block();
		frameSlider.setValue(frameSlider.getValue() + 1);
		frameSliderConnection.unblock();
	}

	if (trackingFPS.getValue() )
		countFrames++;
}

bool MainWindow::updateFPSCounter()
{
	if (trackingFPS.getValue() )
	{
		fpsLabel.set_text(std::to_string(countFrames) + " fps");
		countFrames = 0;
	}
	return trackingFPS.getValue();
}

void MainWindow::whenMakeMapToggled(bool makingMap)
{
	if (makingMap)
	{
    	stabiliseToggle.set_sensitive(true);
    	showMapToggle.set_sensitive(true);
		//showMapToggle.set_active();
		//GUI CHANGES WHEN "MAKE MAP" IS ENABLED GO HERE
	}
	else
	{
    	stabiliseToggle.set_active(false);
    	stabiliseToggle.set_sensitive(false);
		setShowingMap(false);
    	showMapToggle.set_sensitive(false);
		//GUI CHANGES WHEN "MAKE MAP" IS DISABLED GO HERE
	}
}

void MainWindow::whenStabiliseToggled(bool stabilising)
{
	if (stabilising)
	{
		//GUI CHANGES WHEN STABILISER IS ENABLED GO HERE
	}
	else
	{
		//GUI CHANGES WHEN STABILISER IS DISABLED GO HERE
	}
}

void MainWindow::whenShowMapToggled(bool showingMap)
{
	if (showingMap)
	{
		//GUI CHANGES WHEN "SHOW MAP" IS ENABLED GO HERE
	}
	else
	{
		//GUI CHANGES WHEN "SHOW MAP" IS DISABLED GO HERE
	}
}

void MainWindow::whenFindFocusToggled(bool findingFocus)
{
	if (findingFocus)
	{
		//holdFocusToggle.set_sensitive(false);
		//tdStabToggle.set_sensitive(false);
		
		std::cout << "Find focus clicked (in mainwindow.cpp)" << std::endl;


		// //Wait 1 seconds and then disable the toggle, making it a button
		// usleep(1000000);
		// std::cout << "Is this before system is opened?";
		// findFocusToggle.set_active(false);

		//GUI CHANGES WHEN "FINDING FOCUS" IS ENABLED GO HERE
	}
	else
	{
		// holdFocusToggle.set_sensitive(true);
		// tdStabToggle.set_sensitive(true);
		std::cout << "Find focus un-clicked (in mainwindow.cpp)" << std::endl;

		//GUI CHANGES WHEN "FINDING FOCUS" IS DISABLED GO HERE
	}
}

void MainWindow::on_button_clicked()
{
	//GUI CHANGES WHEN "FINDING FOCUS" IS CLICKED
	imgcount = 0;
	bFindFocus = 1;

	usleep(1000000);
	bFindFocus = 0;
}

void MainWindow::whenHoldFocusToggled(bool holdingFocus)
{
	if (holdingFocus)
	{
		bestFocusScale.set_sensitive(true);

		findFocusToggle.set_sensitive(false);
		tdStabToggle.set_sensitive(false);
		//GUI CHANGES WHEN "HOLD FOCUS" IS ENABLED GO HERE
	}
	else
	{
		findFocusToggle.set_sensitive(true);
		tdStabToggle.set_sensitive(true);

		//GUI CHANGES WHEN "FIND FOCUS" IS DISABLED GO HERE
	}
}

void MainWindow::when3DStabToggled(bool active)
{
	if (active)
	{
		holdFocusToggle.set_active(true);
		makeMapToggle.set_active(true);
		stabiliseToggle.set_active(true);
		usleep(1000000);
		showMapToggle.set_active(false);
		//GUI CHANGES WHEN "3D STABILISER" IS ENABLED GO HERE
	}
	else
	{
		holdFocusToggle.set_active(false);
		stabiliseToggle.set_active(false);
		//GUI CHANGES WHEN "3D STABILISER" IS DISABLED GO HERE
	}
}

void MainWindow::bufferFilled()
{
	loading.setValue(false);
	recording.setValue(false);
    backToStartButton.set_sensitive(true);
    pauseButton.set_sensitive(true);
    playButton.set_sensitive(true);
    fileSaveButton.set_sensitive(true);
	liveToggle.set_sensitive(true);
}

void MainWindow::bufferEmptied()
{
	if (!recording.getValue() )
	{
		backToStartButton.set_sensitive(false);
		pauseButton.set_sensitive(false);
		playButton.set_sensitive(false);
		fileSaveButton.set_sensitive(false);
		liveToggle.set_sensitive(false);
		liveToggle.set_active(true);
	}
}

void MainWindow::viewingLive()
{
	playingBuffer.setValue(false);
	recordButton.set_sensitive(true);
	frameSlider.set_sensitive(false);
	trackingFPS.setValue();
}

void MainWindow::viewingBuffer()
{
	recordButton.set_sensitive(false);
	frameSlider.set_sensitive(true);
	trackingFPS.setValue(playingBuffer.getValue() );
}

void MainWindow::whenLoadingToggled(bool loading)
{
	if (loading)
	{
		fileLoadButton.set_sensitive(false);
		hasBuffer.setValue(false);
	}
	else
	{
		fileLoadButton.set_sensitive(true);
	}
}

void MainWindow::whenSavingToggled(bool saving)
{
	if (saving)
	{
		fileLoadButton.set_sensitive(false);
		fileSaveButton.set_sensitive(false);
		recordButton.set_sensitive(false);
	}
	else
	{
		fileLoadButton.set_sensitive(true);
		fileSaveButton.set_sensitive(true);
		recordButton.set_sensitive(true);
	}
}

void MainWindow::whenRecordingToggled(bool recording)
{
	if (recording)
	{
		recordingSizeScale.set_sensitive(false);
		frameSliderConnection.block();
		recordButton.set_sensitive(false);
		backToStartButton.set_sensitive(false);
		pauseButton.set_sensitive(true);
		playButton.set_sensitive(false);
		liveToggle.set_sensitive(false);
		
		trackingFPS.setValue(false);
	}
	else
	{
		recordingSizeScale.set_sensitive(true);
		frameSliderConnection.unblock();
		recordButton.set_sensitive(true);
		pauseButton.set_sensitive(false);
		liveToggle.set_sensitive(true);
		trackingFPS.setValue();
	}
}

void MainWindow::whenPausedRecordingToggled(bool paused)
{
	if (paused)
	{
		recordButton.set_sensitive(true);
	}
	else
	{
		recordButton.set_sensitive(false);
	}
}

void MainWindow::whenTrackingFPSToggled(bool tracking)
{
	if (tracking)
	{
		countFrames = 0;
		Glib::signal_timeout().connect(sigc::mem_fun(*this, &MainWindow::updateFPSCounter), 1000);
	}
}

/*
void MainWindow::whenNotLoading()
{
}
*/

void MainWindow::onLoadButtonClicked()
{
	loading.setValue();
}

void MainWindow::onSaveButtonClicked()
{
	saving.setValue();
}

void MainWindow::onPlayButtonClicked()
{
	setLiveView(false);
	playingBuffer.setValue();
}

void MainWindow::onLiveToggled()
{
	liveView.setValue(liveToggle.get_active() );
}

void MainWindow::onRecordClicked()
{
	if (pausedRecording.getValue() )
		pausedRecording.toggle();
	else
		recording.setValue();
}

void MainWindow::onPauseClicked()
{
	if (recording.getValue() )
	{
		pausedRecording.setValue();
	}
	else
	{
		playingBuffer.setValue(false);
		sigPauseClicked.emit();
	}
}

void MainWindow::onBackButtonClicked()
{
	playingBuffer.setValue(false);
	frameSlider.setValue(0);
}

void MainWindow::whenPlayingBufferToggled(bool playing)
{
	if (playing)
	{
		playButton.set_sensitive(false);
		trackingFPS.setValue();
		fileLoadButton.set_sensitive(false);
		recordingSizeScale.set_sensitive(false);
	}
	else
	{
		playButton.set_sensitive(true);
		trackingFPS.setValue(false);
		fileLoadButton.set_sensitive(true);
		recordingSizeScale.set_sensitive(true);
	}
}

void MainWindow::onFrameSliderChange(double)
{
	seeking.setValue(true);
}

void MainWindow::onGainScaleChange(double val)
{
	sigFeatureUpdated.emit("Gain", val);
}

void MainWindow::onExposeScaleChange(double val)
{
	sigFeatureUpdated.emit("ExposureTime", val);
	//According to Vimba changing ExposureTime may also change camera frame rate
	sigFeatureUpdated.emit("AcquisitionFrameRate", frameRateScale.getValue() );
}

void MainWindow::onFrameRateChange(double val)
{
	sigFeatureUpdated.emit("AcquisitionFrameRate", val);
}

void MainWindow::onGammaScaleChange(double val)
{
	sigFeatureUpdated.emit("Gamma", val);
}

void MainWindow::onThresScaleChange(double val)
{
	sigThresholdChanged.emit(val);
}

void MainWindow::onScaleScaleChange(double val)
{
	sigScaleChanged.emit(val);
}

void MainWindow::onRecordingSizeScaleChange(double val)
{
	frameSlider.setUpperLimit(val-1);
}

void MainWindow::onBestFocusScaleChange(double val)
{
	sigBestFocusChanged.emit(val);
}

MainWindow::~MainWindow()
{
    delete priv;
}
