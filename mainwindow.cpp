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

bool bMainWindowLogFlag = 0; // 1 = log, 0 = no log

struct MainWindow::Private
{
	Gtk::Label gainLabel, exposeLabel, gammaLabel, frameRateLabel, thresLabel, scaleLabel, recordingSizeLabel, bestFocusLabel, homePositionLabel;
	// Gtk::Label waitLabel; //This was used to time-out the stabilizer.cc code, but is deprecated now
	Gtk::Frame controlFrame;
	Gtk::Grid controlGrid;
	Gtk::VBox rootBox;

	Gtk::Label space4[16];
	// Gtk::Label verticalLine[8];
	Gtk::Label autofocusTitle, stabilizationTitle, playbackTitle, cameraSettingsTitle, fileTitle;
	Gtk::Separator verticalSeparator1, verticalSeparator2, verticalSeparator3, verticalSeparator4;

	Gtk::HBox mediaBox, fileChooserBox;
	Gtk::Box displayBox;

public:
	Private();
};

// For GUI elements that are static
MainWindow::Private::Private() : gainLabel("Gain: "),
								 exposeLabel("Expose (us): "),
								 gammaLabel("Gamma: "),
								 frameRateLabel("Frame rate: "),
								 // thresLabel("Threshold: "),
								 // scaleLabel("Scale: "),
								 // waitLabel("Optimize: "),
								 recordingSizeLabel("Rec. Size: "),
								 bestFocusLabel("Best-Focal Plane"),
								 autofocusTitle("Autofocus"),
								 stabilizationTitle("XY Stabilization"),
								 playbackTitle("Playback Controls"),
								 cameraSettingsTitle("Camera Settings"),
								 fileTitle("File Load and Save"),
								 homePositionLabel("Home Position (mm): "),
								 controlFrame(),
								 controlGrid(),
								 verticalSeparator1(),
								 verticalSeparator2(),
								 verticalSeparator3(),
								 verticalSeparator4(),
								 rootBox(),
								 space4{Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    ")},
								 // verticalLine{Gtk::Label(" | "), Gtk::Label("  |  "), Gtk::Label("  |  "), Gtk::Label("  |  "), Gtk::Label("  |  "), Gtk::Label("  |  "), Gtk::Label("  |  "), Gtk::Label("  |  ")},
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
	// thresLabel.set_justify(Gtk::Justification::JUSTIFY_RIGHT);
	// thresLabel.set_halign(Gtk::Align::ALIGN_END);
	// scaleLabel.set_justify(Gtk::Justification::JUSTIFY_RIGHT);
	// scaleLabel.set_halign(Gtk::Align::ALIGN_END);
	//  waitLabel.set_justify(Gtk::Justification::JUSTIFY_RIGHT);
	//  waitLabel.set_halign(Gtk::Align::ALIGN_END);
	recordingSizeLabel.set_justify(Gtk::Justification::JUSTIFY_RIGHT);
	recordingSizeLabel.set_halign(Gtk::Align::ALIGN_END);

	controlGrid.set_hexpand();
	controlGrid.set_halign(Gtk::Align::ALIGN_FILL);
	// controlGrid.set_row_spacing(10);
	controlGrid.set_row_spacing(3);

	rootBox.set_hexpand();
	rootBox.set_halign(Gtk::Align::ALIGN_FILL);

	// rootBox.set_vexpand();
	// rootBox.set_valign(Gtk::Align::ALIGN_FILL);

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

	verticalSeparator1.override_color(Gdk::RGBA("black"), Gtk::STATE_FLAG_NORMAL);
	verticalSeparator2.override_color(Gdk::RGBA("black"), Gtk::STATE_FLAG_NORMAL);
	verticalSeparator3.override_color(Gdk::RGBA("black"), Gtk::STATE_FLAG_NORMAL);
	verticalSeparator4.override_color(Gdk::RGBA("black"), Gtk::STATE_FLAG_NORMAL);

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
		scale.set_range(lower, (inc * lower - lower + upper) / inc);
		scale.set_increments(1, 10);
		scale.set_round_digits(0);
		scale.set_digits(0);
		scale.set_value((inc * lower - lower + def) / inc);
	}
	else
	{
		scale.set_range(lower, upper);
		scale.set_increments(inc, inc * 10);
		scale.set_value(def);
	}

	scale.set_draw_value(false);
	scaleConnection = scale.signal_value_changed().connect(sigc::mem_fun(*this, &ScaleWidget::scaleChanged));
	if (scaleWidth)
		scale.set_size_request(scaleWidth);

	spinButton.set_range(lower, upper);
	spinButton.set_increments(inc, inc * 10);
	spinButton.set_snap_to_ticks(stepSnap);
	spinButton.set_value(def);
	spinButtonConnection = spinButton.signal_value_changed().connect(sigc::mem_fun(*this, &ScaleWidget::spinButtonChanged));
	if (spinButtonWidth)
		spinButton.set_width_chars(spinButtonWidth);
	scale.set_slider_size_fixed();

	grid.attach(spinButton, 0, 0);
	grid.attach(scale, 1, 0);

	add(grid);
}

ScaleWidget::~ScaleWidget() {
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
		scale.set_range(min, (a->get_step_increment() * min - min + value) / a->get_step_increment());
		scale.set_value(min + (spinButton.get_value() - min) * a->get_step_increment());
	}
	else
	{
		scale.set_range(min, value);
		scale.set_value(spinButton.get_value());
	}
	scaleConnection.unblock();
	if (spinButton.get_value() != old)
		sigChanged.emit(spinButton.get_value());
}

void ScaleWidget::spinButtonChanged()
{
	scaleConnection.block();
	scale.set_value(spinButton.get_value());
	scaleConnection.unblock();
	sigChanged.emit(spinButton.get_value());
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
		spinButton.set_value(lower + (value - lower) * inc);
	}
	else
		spinButton.set_value(scale.get_value());
	spinButtonConnection.unblock();
	sigChanged.emit(scale.get_value());
}

bool MainWindow::_on_state_event(GdkEventWindowState *window_state_event)
{
	if (!(window_state_event->new_window_state & (GdkWindowState::GDK_WINDOW_STATE_WITHDRAWN | GdkWindowState::GDK_WINDOW_STATE_ICONIFIED)))
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
						   priv(new Private()),
						   gainScale(0, 50, 1, 0, 7, 150),
						   exposeScale(100, 16000, 1000, 15000, 7, 150),
						   gammaScale(0.5, 2.5, 0.01, 1, 7, 150),
						   // frameRateScale(20, 80, 5, 30, 7, 150),
						   frameSlider(0, 1799, 1, 0, 5, 200),
						   // thresScale(0, 1, 0.01, 0.4, 6, 100),
						   // scaleScale(0, 5, 0.01, 2.0, 6, 100),
						   waitScale(0, HVIGTK_STAB_LIM, 100, 0, 6, 100),
						   recordingSizeScale(100, 1800, 10, 900, 6, 100),
						   // bestFocusScale(8, 632, 5, 200, 4, 100),
						   bestFocusScale(130, 510, 5, 240, 4, 100),
						   homePositionScale(-14.0, 0.0, 0.1, -9.1, 4, 100),
						   outOfBoundsWarningLabel(""),
						   recordButton(),
						   backToStartButton(),
						   pauseButton(),
						   playButton(),
						   resetButton("Reset Lens"),
						   recenterButton("Recenter"),
						   fileLoadButton("Load"),
						   fileSaveButton("Save"),
						   frameRateEntry(),
						   enterButton("Enter"),
						   liveToggle(),
						   // makeMapToggle("Make Map"),
						   stabiliseToggle("XY Stab."),
						   showMapToggle("Show Map"),
						   findFocusButton("Find Focus"),
						   holdFocusToggle("Hold Focus"),
						   threedStabToggle("3D Stab."),
						   twodStabToggle("2D Stab."),
						   fpsLabel(""),
						   loadSaveLabel(""),
						   errorLabel(""),
						   // display(*this),
						   drawFrame(nullptr),
						   newDrawFrame(false),
						   countFrames(),
						   // makeMapActive(),
						   stabiliseActive(),
						   showMapActive(),
						   holdFocusActive(),
						   threedStabActive(),
						   twodStabActive(),
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
						   // sigThresholdChanged(),
						   // sigScaleChanged(),
						   sigBestFocusChanged(),
						   gainScaleConnection(),
						   exposeScaleConnection(),
						   gammaScaleConnection(),
						   // frameRateScaleConnection(),
						   frameSliderConnection(),
						   stateChangeConnection(),
						   homePositionScaleConnection()
{
	// set_default_size(1700,200);
	set_title("HVI-GTK " HVIGTK_VERSION_STR);
	add_events(Gdk::STRUCTURE_MASK);

	stateChangeConnection = signal_window_state_event().connect(sigc::mem_fun(*this, &MainWindow::_on_state_event));
	// gainScale.setScaleSizeRequest(150, 0);
	// gainScale.setSpinButtonWidth(7);
	gainScaleConnection = gainScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onGainScaleChange));

	// exposeScale.setScaleSizeRequest(150, 0);
	// exposeScale.setSpinButtonWidth(7);
	exposeScaleConnection = exposeScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onExposeScaleChange));

	// gammaScale.setScaleSizeRequest(150, 0);
	// gammaScale.setSpinButtonWidth(7);
	gammaScale.setSpinButtonPrec(2);
	gammaScaleConnection = gammaScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onGammaScaleChange));

	// frameRateScaleConnection = frameRateScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onFrameRateChange) );

	recenterButton.set_sensitive(false);

	// Tooltip text for the buttons
	// makeMapToggle.set_tooltip_text("Make a vessel map of the current recording using given threshold and scale values");
	stabiliseToggle.set_tooltip_text("Using vessel map, XY-stabilise the current recording");
	// showMapToggle.set_tooltip_text("Show the loaded vessel map");
	findFocusButton.set_tooltip_text("Finds focal plane with highest sharpness");
	holdFocusToggle.set_tooltip_text("Holds current focal plane in-focus (even if not ");
	threedStabToggle.set_tooltip_text("Shortcut for live angiograms which uses 'Hold Focus' and 'XY-Stab'");

	recordButton.set_image_from_icon_name("media-record");
	recordButton.set_tooltip_text("Begin recording");
	recordButton.set_hexpand(false);
	recordButton.set_valign(Gtk::Align::ALIGN_FILL);
	recordButton.set_halign(Gtk::Align::ALIGN_START);
	recordButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onRecordClicked));
	auto cssProvider = Gtk::CssProvider::create();
	Glib::ustring cssData =
		R"(
		#recordButton {
			color: #d10000;  
			-gtk-icon-style: symbolic;
		}
	)";
	cssProvider->load_from_data(cssData);
	recordButton.get_style_context()->add_provider(cssProvider, GTK_STYLE_PROVIDER_PRIORITY_USER);
	recordButton.set_name("recordButton");

	backToStartButton.set_image_from_icon_name("media-skip-backward");
	backToStartButton.set_tooltip_text("Go back to start of recording");
	backToStartButton.set_hexpand(false);
	backToStartButton.set_valign(Gtk::Align::ALIGN_FILL);
	backToStartButton.set_halign(Gtk::Align::ALIGN_START);
	backToStartButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onBackButtonClicked));
	backToStartButton.set_sensitive(false);
	auto cssProvider2 = Gtk::CssProvider::create();
	Glib::ustring cssData2 =
		R"(
		#backToStartButton {
			color: #cccccc;  
			-gtk-icon-style: symbolic;
		}
	)";
	cssProvider2->load_from_data(cssData2);
	backToStartButton.get_style_context()->add_provider(cssProvider2, GTK_STYLE_PROVIDER_PRIORITY_USER);
	backToStartButton.set_name("backToStartButton");

	pauseButton.set_image_from_icon_name("media-playback-pause");
	pauseButton.set_tooltip_text("Pause playback of recording");
	pauseButton.set_hexpand(false);
	pauseButton.set_valign(Gtk::Align::ALIGN_FILL);
	pauseButton.set_halign(Gtk::Align::ALIGN_START);
	pauseButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onPauseClicked));
	pauseButton.set_sensitive(false);
	auto cssProvider3 = Gtk::CssProvider::create();
	Glib::ustring cssData3 =
		R"(
		#pauseButton {
			color: #cccccc;  
			-gtk-icon-style: symbolic;
		}
	)";
	cssProvider3->load_from_data(cssData3);
	pauseButton.get_style_context()->add_provider(cssProvider3, GTK_STYLE_PROVIDER_PRIORITY_USER);
	pauseButton.set_name("pauseButton");

	playButton.set_image_from_icon_name("media-playback-start");
	playButton.set_tooltip_text("Start/Resume playback of recording");
	playButton.set_hexpand(false);
	playButton.set_valign(Gtk::Align::ALIGN_FILL);
	playButton.set_halign(Gtk::Align::ALIGN_START);
	playButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onPlayButtonClicked));
	playButton.set_sensitive(false);
	auto cssProvider4 = Gtk::CssProvider::create();
	Glib::ustring cssData4 =
		R"(
		#playButton {
			color: #cccccc;  
			-gtk-icon-style: symbolic;
		}
	)";
	cssProvider4->load_from_data(cssData4);
	playButton.get_style_context()->add_provider(cssProvider4, GTK_STYLE_PROVIDER_PRIORITY_USER);
	playButton.set_name("playButton");

	liveToggle.set_image_from_icon_name("camera-web");
	liveToggle.set_tooltip_text("Switch to/from live camera feed");
	liveToggle.set_hexpand(false);
	liveToggle.set_valign(Gtk::Align::ALIGN_FILL);
	liveToggle.set_halign(Gtk::Align::ALIGN_START);
	liveToggle.signal_toggled().connect(sigc::mem_fun(*this, &MainWindow::onLiveToggled));
	liveToggle.set_sensitive(false);

	priv->mediaBox.add(recordButton);
	priv->mediaBox.add(backToStartButton);
	priv->mediaBox.add(pauseButton);
	priv->mediaBox.add(playButton);
	priv->mediaBox.add(liveToggle);

	// frameSlider.setScaleSizeRequest(200, 0);
	// frameSlider.setSpinButtonWidth(5);
	frameSliderConnection = frameSlider.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onFrameSliderChange));

	fileLoadButton.set_tooltip_text("Load frames from this location");
	fileLoadButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onLoadButtonClicked));

	fileSaveButton.set_tooltip_text("Save frames to this location");
	fileSaveButton.set_sensitive(false);
	fileSaveButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onSaveButtonClicked));

	stabiliseToggle.set_sensitive(true);
	// showMapToggle.set_sensitive(false);
	// showMapToggle.signal_clicked();

	// thresScale.setScaleSizeRequest(100, 0);
	// thresScale.setSpinButtonWidth(4);
	// thresScale.setSpinButtonPrec(2);
	// thresScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onThresScaleChange) );

	// scaleScale.setScaleSizeRequest(100, 0);
	// scaleScale.setSpinButtonWidth(4);
	// scaleScale.setSpinButtonPrec(2);
	// scaleScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onScaleScaleChange) );
	recordingSizeScale.set_tooltip_text("Set the maximum number of frames for a single recording");
	recordingSizeScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onRecordingSizeScaleChange));

	holdFocusToggle.set_sensitive(true);
	threedStabToggle.set_sensitive(true);
	twodStabToggle.set_sensitive(true);
	bestFocusScale.set_sensitive(false);

	// bestFocusScale.setScaleSizeRequest(100, 0);
	// bestFocusScale.setSpinButtonWidth(4);
	bestFocusScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onBestFocusScaleChange));
	//
	// display.set_hexpand(true);
	// display.set_vexpand(true);
	// display.set_halign(Gtk::Align::ALIGN_FILL);
	// display.set_valign(Gtk::Align::ALIGN_FILL);

	// display.signal_draw().connect(sigc::mem_fun(*this, &MainWindow::renderDisplay) );

	setLiveView(true);

	// CONDITIONS

	// makeMapActive.toggleOnSignal(makeMapToggle.signal_toggled() );
	// makeMapActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenMakeMapToggled) );

	stabiliseActive.toggleOnSignal(stabiliseToggle.signal_toggled());
	stabiliseActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenStabiliseToggled));

	// showMapActive.toggleOnSignal(showMapToggle.signal_toggled() );
	// showMapActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenShowMapToggled) );

	findFocusButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onFindFocusClicked));

	resetButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onResetClicked));
	resetButton.set_tooltip_text("Resets the lens to home position");

	recenterButton.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::onRecenterClicked));
	recenterButton.set_tooltip_text("Recenter the rendered video frames");

	holdFocusActive.toggleOnSignal(holdFocusToggle.signal_toggled());
	holdFocusActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenHoldFocusToggled));

	threedStabActive.toggleOnSignal(threedStabToggle.signal_toggled());
	threedStabActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::when3DStabToggled));

	// twodStabActive.toggleOnSignal(twodStabToggle.signal_toggled() );
	// twodStabActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::when2DStabToggled) );

	hasBuffer.signalTrue().connect(sigc::mem_fun(*this, &MainWindow::bufferFilled));
	hasBuffer.signalFalse().connect(sigc::mem_fun(*this, &MainWindow::bufferEmptied));

	// Viewing means we're not in liveView, and we either have a buffer or we're loading another buffer
	Condition &viewing = !liveView && (hasBuffer || loading);

	viewing.signalFalse().connect(sigc::mem_fun(*this, &MainWindow::viewingLive));
	viewing.signalTrue().connect(sigc::mem_fun(*this, &MainWindow::viewingBuffer));

	loading.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenLoadingToggled));
	saving.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenSavingToggled));

	recording.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenRecordingToggled));
	pausedRecording.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenPausedRecordingToggled));

	trackingFPS.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenTrackingFPSToggled));
	trackingFPS.setValue(true);

	playingBuffer.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenPlayingBufferToggled));
	loadSaveLabel.set_halign(Gtk::ALIGN_CENTER);
	sigFrameDrawn.connect(sigc::mem_fun(*this, &MainWindow::onFrameDrawn));

	errorLabel.set_halign(Gtk::ALIGN_CENTER);

	frameRateEntry.set_width_chars(3);
	frameRateEntry.set_text("30");

	fileLoadButton.set_size_request(125, -1); // Width: 120px, Height: keep default (-1)
	fileSaveButton.set_size_request(125, -1); // Width: 120px, Height: keep default (-1)

	enterButton.signal_clicked().connect(sigEnterClicked.make_slot());
	Gtk::Box *entryButtonBox = Gtk::manage(new Gtk::Box(Gtk::Orientation::ORIENTATION_HORIZONTAL, 0));
	entryButtonBox->pack_start(frameRateEntry, Gtk::PackOptions::PACK_EXPAND_WIDGET);
	entryButtonBox->pack_start(enterButton, Gtk::PackOptions::PACK_SHRINK);
	enterButton.set_size_request(74, -1); // 100 pixels in width, -1 means natural height
	// Create an Alignment widget for space.
	Gtk::Alignment *spacer = Gtk::manage(new Gtk::Alignment());
	spacer->set_size_request(150, -1); // for example, 50 pixels wide
	entryButtonBox->pack_end(*spacer, Gtk::PackOptions::PACK_SHRINK);

	// Setup warning label
	outOfBoundsWarningLabel.set_markup("<span foreground='orange'>\u26A0 Lens Out-of-Bounds\n    Autofocus May Fail</span>");
	outOfBoundsWarningLabel.set_visible(false);
	outOfBoundsWarningLabel.set_halign(Gtk::ALIGN_CENTER);

	// Modify the Entry
	auto contextEntry = frameRateEntry.get_style_context();
	contextEntry->add_class("straight-edge-entry");
	contextEntry->add_class("no-right-border");
	// Modify the Button
	auto contextButton = enterButton.get_style_context();
	contextButton->add_class("straight-edge-button");

	auto css_provider = Gtk::CssProvider::create();
	css_provider->load_from_data(R"CSS(
		.straight-edge-entry {
			border-top-right-radius: 0px;
			border-bottom-right-radius: 0px;
		}
		
		.straight-edge-button {
			border-top-left-radius: 0px;
			border-bottom-left-radius: 0px;
		}
			.no-right-border {
			border-right-width: 0px;
		}
	)CSS");

	auto screen = Gdk::Screen::get_default();
	Gtk::StyleContext::add_provider_for_screen(screen, css_provider, GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);

	// ATTACHING

	priv->controlGrid.attach(priv->space4[0], 0, 0);

	priv->controlGrid.attach(priv->autofocusTitle, 1, 4, 3);
	priv->controlGrid.attach(findFocusButton, 1, 1);
	priv->controlGrid.attach(holdFocusToggle, 1, 2);
	priv->controlGrid.attach(threedStabToggle, 1, 0);
	priv->controlGrid.attach(resetButton, 1, 3);

	priv->controlGrid.attach(priv->space4[1], 2, 0);

	priv->controlGrid.attach(priv->bestFocusLabel, 3, 0);
	priv->controlGrid.attach(bestFocusScale, 3, 1);
	// priv->controlGrid.attach(returnPositionScale, 3, 2);	// Uncomment to give user control of home position
	//  Add warning label below best focus scale in the grid when lens is out of bounds
	priv->controlGrid.attach(outOfBoundsWarningLabel, 3, 2, 2, 2);

	priv->controlGrid.attach(priv->space4[2], 4, 0);
	priv->controlGrid.attach(priv->verticalSeparator1, 5, 0, 1, 4);
	priv->controlGrid.attach(priv->space4[3], 6, 0);

	priv->controlGrid.attach(priv->stabilizationTitle, 7, 4, 4);
	// priv->controlGrid.attach(makeMapToggle, 7, 0); DEPRECATED
	priv->controlGrid.attach(stabiliseToggle, 7, 0);
	priv->controlGrid.attach(recenterButton, 7, 1);
	// priv->controlGrid.attach(showMapToggle, 7, 2); DEPRECATED
	// priv->controlGrid.attach(twodStabToggle, 7, 3); Retired functionality

	priv->controlGrid.attach(priv->space4[4], 8, 0);

	// priv->controlGrid.attach(priv->thresLabel, 9, 0); DEPRECATED
	// priv->controlGrid.attach(priv->scaleLabel, 9, 1); DEPRECATED
	priv->controlGrid.attach(priv->recordingSizeLabel, 9, 0);

	// priv->controlGrid.attach(thresScale, 10, 0);
	// priv->controlGrid.attach(scaleScale, 10, 1);
	priv->controlGrid.attach(recordingSizeScale, 10, 0);

	priv->controlGrid.attach(priv->space4[5], 11, 0);
	priv->controlGrid.attach(priv->verticalSeparator2, 12, 0, 1, 4);
	priv->controlGrid.attach(priv->space4[6], 13, 0);

	priv->controlGrid.attach(priv->playbackTitle, 14, 4, 1);
	priv->controlGrid.attach(priv->mediaBox, 14, 0);
	priv->controlGrid.attach(frameSlider, 14, 1);
	priv->controlGrid.attach(fpsLabel, 14, 2);
	priv->controlGrid.attach(errorLabel, 14, 3);

	priv->controlGrid.attach(priv->space4[7], 15, 0);
	priv->controlGrid.attach(priv->verticalSeparator3, 16, 0, 1, 4);
	priv->controlGrid.attach(priv->space4[8], 17, 0);

	priv->controlGrid.attach(priv->cameraSettingsTitle, 18, 4, 2);
	priv->controlGrid.attach(priv->gainLabel, 18, 0);
	priv->controlGrid.attach(priv->exposeLabel, 18, 1);
	priv->controlGrid.attach(priv->gammaLabel, 18, 2);
	priv->controlGrid.attach(priv->frameRateLabel, 18, 3);

	priv->controlGrid.attach(gainScale, 19, 0);
	priv->controlGrid.attach(exposeScale, 19, 1);
	priv->controlGrid.attach(gammaScale, 19, 2);
	priv->controlGrid.attach(*entryButtonBox, 19, 3);

	priv->controlGrid.attach(priv->space4[9], 20, 0);
	priv->controlGrid.attach(priv->verticalSeparator4, 21, 0, 1, 4);
	priv->controlGrid.attach(priv->space4[10], 22, 0);
	priv->controlGrid.attach(loadSaveLabel, 23, 3, 2, 1);

	priv->controlGrid.attach(priv->fileTitle, 23, 4, 2);
	priv->controlGrid.attach(fileLoadButton, 23, 2);

	priv->controlGrid.attach(fileSaveButton, 24, 2);

	priv->controlGrid.attach(priv->space4[11], 25, 0);

	// priv->controlGrid.attach(priv->waitLabel, 7, 2);
	// priv->controlGrid.attach(waitScale, 8, 2);

	priv->controlFrame.add(priv->controlGrid);

	priv->controlFrame.set_vexpand(false);
	priv->controlFrame.set_valign(Gtk::Align::ALIGN_START);

	///////////

	// priv->displayBox.pack_start(display, true, true);

	priv->rootBox.pack_start(priv->controlFrame, false, true);
	// priv->rootBox.pack_start(priv->displayBox, true, true);
	// priv->rootBox.add(display);

	add(priv->rootBox);
	resize(priv->rootBox.get_width(), priv->rootBox.get_height());

	priv->rootBox.show_all();
	hideOutOfBoundsWarning();
	if (bMainWindowLogFlag)
		logger->info("[MainWindow::MainWindow] constructor completed");

	// Setup home position scale
	homePositionScale.set_tooltip_text("Set the lens return position");
	homePositionScale.setSpinButtonPrec(1);
	homePositionScaleConnection = homePositionScale.signalChanged().connect(
		sigc::mem_fun(*this, &MainWindow::onHomePositionScaleChange));
}

// double MainWindow::getFrameRateScaleValue() const
// {
// 	return frameRateScale.getValue();
// }

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

void MainWindow::setBestFocusScaleValue(double v)
{
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

	if (bMainWindowLogFlag)
	{
		logger->info("[MainWindow::updateCameraValues] gainScale, exposeScale, gammaScale updated");
	}
}

void MainWindow::getDisplayDimensions(double &w, double &h) const
{
	// w = display.get_width();
	// h = display.get_height();

	w = 800;
	h = 600;

	if (bMainWindowLogFlag)
	{
		logger->info("[MainWindow::getDisplayDimensions] GTK display dimensions of 800,600 returned");
	}
}

void MainWindow::displayMessageFPS(const std::string &msg)
{
	fpsLabel.set_text(msg);
}

void MainWindow::displayMessageLoadSave(const std::string &msg)
{
	loadSaveLabel.set_text(msg);
}

void MainWindow::displayMessageError(const std::string &msg)
{
	errorLabel.set_text(msg);
}

void MainWindow::renderFrame(VidFrame *frame)
{
	newDrawFrame = frame != drawFrame;
	if (newDrawFrame)
	{
		drawFrame = frame;
		// display.queue_draw();

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

// MainWindow::SignalThresholdChanged MainWindow::signalThresholdChanged()
// {
// 	return sigThresholdChanged;
// }

// MainWindow::SignalScaleChanged MainWindow::signalScaleChanged()
// {
// 	return sigScaleChanged;
// }

MainWindow::SignalBestFocusChanged MainWindow::signalBestFocusChanged()
{
	return sigBestFocusChanged;
}

MainWindow::SignalPauseClicked MainWindow::signalPauseClicked()
{
	return sigPauseClicked;
}

MainWindow::SignalEnterClicked MainWindow::signalEnterClicked()
{
	return sigEnterClicked;
}

MainWindow::SignalFindFocusClicked MainWindow::signalFindFocusClicked()
{
	return sigFindFocusClicked;
}

MainWindow::SignalResetClicked MainWindow::signalResetClicked()
{
	return sigResetClicked;
}

MainWindow::SignalRecenterClicked MainWindow::signalRecenterClicked()
{
	return sigRecenterClicked;
}

void MainWindow::setHasBuffer(bool val)
{
	hasBuffer.setValue(val);
}

Condition &MainWindow::getLoading()
{
	return loading;
}

Condition &MainWindow::getSaving()
{
	return saving;
}

void MainWindow::setLiveView(bool val)
{
	liveToggle.set_active(val); // also sets condition
	// liveView.setValue(val);
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
		playingBuffer.setValue(seeking.getValue());
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

// void MainWindow::setMakingMap(bool val)
// {
// 	//makeMapActive.setValue(val);
// 	makeMapToggle.set_active(val);
// }

// void MainWindow::setShowingMap(bool val)
// {
// 	showMapToggle.set_active(val);
// }

void MainWindow::set3DStab(bool val)
{
	threedStabToggle.set_active(val);
}

void MainWindow::setHoldFocus(bool val)
{
	holdFocusToggle.set_active(val);
}

void MainWindow::setTrackingFPS(bool val)
{
	trackingFPS.setValue(val);
}

// Condition& MainWindow::getMakeMapActive()
// {
// 	return makeMapActive;
// }

Condition &MainWindow::getStabiliseActive()
{
	return stabiliseActive;
}

// Condition& MainWindow::getShowMapActive()
// {
// 	return showMapActive;
// }

Condition &MainWindow::getHoldFocusActive()
{
	return holdFocusActive;
}

Condition &MainWindow::get3DStabActive()
{
	return threedStabActive;
}

Condition &MainWindow::get2DStabActive()
{
	return twodStabActive;
}

Condition &MainWindow::getHasBuffer()
{
	return hasBuffer;
}

Condition &MainWindow::getLiveView()
{
	return liveView;
}

Condition &MainWindow::getPlayingBuffer()
{
	return playingBuffer;
}

Condition &MainWindow::getSeeking()
{
	return seeking;
}

Condition &MainWindow::getRecording()
{
	return recording;
}

Condition &MainWindow::getPausedRecording()
{
	return pausedRecording;
}

int MainWindow::getFrameSliderValue() const
{
	return frameSlider.getValue();
}

std::string MainWindow::getFileLocation() const
{
	return current_file_path;
}

double MainWindow::getFrameRateEntryBox() const
{
	// convert string to double
	std::string tempText = frameRateEntry.get_text();
	double frameRate;
	try
	{
		frameRate = std::stod(tempText);
		std::cout << "Frame Rate: " << frameRate << std::endl;
	}
	catch (std::invalid_argument &e)
	{
		std::cerr << "Invalid input: std::invalid_argument thrown" << '\n';
	}
	catch (std::out_of_range &e)
	{
		std::cerr << "Invalid input: std::out_of_range thrown" << '\n';
	}
	std::cout << "Frame Rate: " << frameRate << std::endl;
	return frameRate;
}

void MainWindow::addRenderFilter(const std::string &key, RenderFilter *filter)
{
	renderFilters[key] = filter;
}

RenderFilter *MainWindow::removeRenderFilter(const std::string &key)
{
	if (renderFilters.count(key))
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
	int monIndex = get_screen()->get_monitor_at_window(get_window());
	Gdk::Rectangle dim;
	get_screen()->get_monitor_geometry(monIndex, dim);
	move(dim.get_x() + (dim.get_width() - get_width()) / 2, 100);
}

void MainWindow::on_show()
{
	Gtk::Window::on_show();
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	Gdk::Rectangle dim, frameDim;
	int monIndex = get_screen()->get_monitor_at_window(get_window());
	get_screen()->get_monitor_geometry(monIndex, dim);

	int x, y;
	get_window()->get_origin(x, y);

	// std::cout << x << " " << y << " " << x2 << " " << y2 << std::endl;

	SDLWindow::move(childwin, (dim.get_width() - 800) / 2, y + get_height());

	if (bMainWindowLogFlag)
		logger->info("[MainWindow::on_show] SDL childwin moved to " + std::to_string((dim.get_width() - 800) / 2) + "," + std::to_string(y + get_height()));
}

bool MainWindow::renderDisplay(const ::Cairo::RefPtr<::Cairo::Context> &cr)
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
	if (newDrawFrame && playingBuffer.getValue())
	{
		frameSliderConnection.block();
		frameSlider.setValue(frameSlider.getValue() + 1);
		frameSliderConnection.unblock();
	}

	if (trackingFPS.getValue())
		countFrames++;
}

bool MainWindow::updateFPSCounter()
{
	if (trackingFPS.getValue())
	{
		fpsLabel.set_text(std::to_string(countFrames) + " fps");
		countFrames = 0;
	}
	return trackingFPS.getValue();
}

// void MainWindow::whenMakeMapToggled(bool makingMap)
// {
// 	if (makingMap)
// 	{
//     	stabiliseToggle.set_sensitive(true);
//     	showMapToggle.set_sensitive(true);

// 		//showMapToggle.set_active();
// 		//GUI CHANGES WHEN "MAKE MAP" IS ENABLED GO HERE
// 	    if(bMainWindowLogFlag) {logger->info("[MainWindow::whenMakeMapToggled] MakeMap toggled on");}

// 	}
// 	else
// 	{
//     	stabiliseToggle.set_active(false);
//     	stabiliseToggle.set_sensitive(false);
// 		setShowingMap(false);
//     	showMapToggle.set_sensitive(false);
// 		//GUI CHANGES WHEN "MAKE MAP" IS DISABLED GO HERE
// 	    if(bMainWindowLogFlag) {logger->info("[MainWindow::whenMakeMapToggled] MakeMap toggled off");}
// 	}
// }

void MainWindow::whenStabiliseToggled(bool stabilising)
{
	if (stabilising)
	{
		// GUI CHANGES WHEN STABILISER IS ENABLED GO HERE
		if (bMainWindowLogFlag)
			logger->info("[MainWindow::whenStabiliseToggled] Stabilise toggled on");
		recenterButton.set_sensitive(true);
	}
	else
	{
		// GUI CHANGES WHEN STABILISER IS DISABLED GO HERE
		if (bMainWindowLogFlag)
			logger->info("[MainWindow::whenStabiliseToggled] Stabilise toggled off");
		recenterButton.set_sensitive(false);
	}
}

// void MainWindow::whenShowMapToggled(bool showingMap)
// {
// 	if (showingMap)
// 	{
// 		//GUI CHANGES WHEN "SHOW MAP" IS ENABLED GO HERE
// 		if(bMainWindowLogFlag) logger->info("[MainWindow::whenShowMapToggled] ShowMap toggled on");
// 	}
// 	else
// 	{
// 		//GUI CHANGES WHEN "SHOW MAP" IS DISABLED GO HERE
// 		if(bMainWindowLogFlag) logger->info("[MainWindow::whenShowMapToggled] ShowMap toggled off");
// 	}
// }

void MainWindow::onFindFocusClicked()
{
	imgcount = 0;
	bFindFocus = 1;
	if (bMainWindowLogFlag)
	{
		logger->info("[MainWindow::onFindFocusClicked] FindFocus clicked, so bFindFocus set to 1");
	}
	usleep(800000);
	bFindFocus = 0;
	if (bMainWindowLogFlag)
	{
		logger->info("[MainWindow::onFindFocusClicked] bFindFocus set to 0");
	}
}

void MainWindow::onResetClicked()
{
	// GUI CHANGES WHEN "RESET" IS CLICKED
	holdFocusToggle.set_active(false);
	threedStabToggle.set_active(false);
	twodStabToggle.set_active(false);
	// pause for 100ms
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	bResetLens = 1;

	// Hide out-of-bounds label
	hideOutOfBoundsWarning();

	// GUI CHANGES WHEN "RESET" IS CLICKED
	if (bMainWindowLogFlag)
	{
		logger->info("[MainWindow::onResetClicked] Reset clicked, so bResetLens set to 1");
	}
}

void MainWindow::onRecenterClicked()
{
	signalRecenterClicked().emit();
	// GUI CHANGES WHEN "RECENTER" IS CLICKED
	if (bMainWindowLogFlag)
	{
		logger->info("[MainWindow::onRecenterClicked] Recenter clicked");
	}
}

void MainWindow::whenHoldFocusToggled(bool holdingFocus)
{
	if (holdingFocus)
	{
		// Make best focus scale active and set value to the desired location of best-focus
		bestFocusScale.set_sensitive(true);
		// bestFocusScale.setValue(desiredLocBestFocus);

		findFocusButton.set_sensitive(false);
		// threedStabToggle.set_sensitive(false);
		// GUI CHANGES WHEN "HOLD FOCUS" IS ENABLED GO HERE
		if (bMainWindowLogFlag)
			logger->info("[MainWindow::whenHoldFocusToggled] HoldFocus toggled on");
	}
	else
	{
		findFocusButton.set_sensitive(true);
		// threedStabToggle.set_sensitive(true);
		bestFocusScale.set_sensitive(false);

		// GUI CHANGES WHEN "FIND FOCUS" IS DISABLED GO HERE
		if (bMainWindowLogFlag)
			logger->info("[MainWindow::whenHoldFocusToggled] HoldFocus toggled off");
	}
}

void MainWindow::when3DStabToggled(bool active)
{
	if (active)
	{
		holdFocusToggle.set_active(true);
		// makeMapToggle.set_active(true);
		stabiliseToggle.set_active(true);
		// GUI CHANGES WHEN "3D STABILISER" IS ENABLED GO HERE
		if (bMainWindowLogFlag)
			logger->info("[MainWindow::when3DStabToggled] 3DStab toggled on");
	}
	else
	{
		holdFocusToggle.set_active(false);
		// makeMapToggle.set_active(false);
		stabiliseToggle.set_active(false);
		// GUI CHANGES WHEN "3D STABILISER" IS DISABLED GO HERE
		if (bMainWindowLogFlag)
			logger->info("[MainWindow::when3DStabToggled] 3DStab toggled off");
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
	recordButton.set_sensitive(false);
	if (bMainWindowLogFlag)
	{
		logger->info("[MainWindow::bufferFilled] Recording buffer filled");
	}
}

void MainWindow::bufferEmptied()
{
	if (!recording.getValue())
	{
		backToStartButton.set_sensitive(false);
		pauseButton.set_sensitive(false);
		playButton.set_sensitive(false);
		fileSaveButton.set_sensitive(false);
		liveToggle.set_sensitive(false);
		liveToggle.set_active(true);
		recordButton.set_sensitive(true);
		frameSlider.setValue(0);
		if (bMainWindowLogFlag)
		{
			logger->info("[MainWindow::bufferEmptied] Recording buffer emptied... although nothing is deleted??");
		}
	}
}

void MainWindow::viewingLive()
{
	playingBuffer.setValue(false);
	recordButton.set_sensitive(true);
	frameSlider.set_sensitive(false);
	std::cout << "setting frameSlider to 0 because viewing live" << std::endl;
	frameSlider.setValue(0);
	trackingFPS.setValue();

	if (bMainWindowLogFlag)
	{
		logger->info("[MainWindow::viewingLive] Set to viewing live");
	}
}

void MainWindow::viewingBuffer()
{
	recordButton.set_sensitive(false);
	frameSlider.set_sensitive(true);
	trackingFPS.setValue(playingBuffer.getValue());
	if (bMainWindowLogFlag)
	{
		logger->info("[MainWindow::viewingBuffer] Set to viewing buffer");
	}
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
		if (bMainWindowLogFlag)
		{
			logger->info("[MainWindow::whenSavingToggled] Saving toggled on");
		}
	}
	else
	{
		fileLoadButton.set_sensitive(true);
		fileSaveButton.set_sensitive(true);
		recordButton.set_sensitive(true);
		if (bMainWindowLogFlag)
		{
			logger->info("[MainWindow::whenSavingToggled] Saving toggled off");
		}
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

		trackingFPS.setValue(false); // Stop tracking FPS when recording. TODO: Add second label to show FPS while recording.

		if (bMainWindowLogFlag)
		{
			logger->info("[MainWindow::whenRecordingToggled] Recording toggled on");
		}
	}
	else
	{
		recordingSizeScale.set_sensitive(true);
		frameSliderConnection.unblock();
		recordButton.set_sensitive(true);
		pauseButton.set_sensitive(false);
		liveToggle.set_sensitive(true);

		trackingFPS.setValue(true);

		if (bMainWindowLogFlag)
		{
			logger->info("[MainWindow::whenRecordingToggled] Recording toggled off");
		}
	}
}

void MainWindow::whenPausedRecordingToggled(bool paused)
{
	if (paused)
	{
		recordButton.set_sensitive(true);
		fileSaveButton.set_sensitive(true);
		if (bMainWindowLogFlag)
		{
			logger->info("[MainWindow::whenPausedRecordingToggled] Paused recording toggled on");
		}
	}
	else
	{
		recordButton.set_sensitive(false);
		fileSaveButton.set_sensitive(false);
		if (bMainWindowLogFlag)
		{
			logger->info("[MainWindow::whenPausedRecordingToggled] Paused recording toggled off");
		}
	}
}

void MainWindow::whenTrackingFPSToggled(bool tracking)
{
	if (tracking)
	{
		countFrames = 0;
		Glib::signal_timeout().connect(sigc::mem_fun(*this, &MainWindow::updateFPSCounter), 1000);
		if (bMainWindowLogFlag)
		{
			logger->info("[MainWindow::whenTrackingFPSToggled] Tracking FPS toggled on");
		}
	}
}

/*
void MainWindow::whenNotLoading()
{
}
*/

void MainWindow::onLoadButtonClicked()
{
	Gtk::FileChooserDialog dialog("Please choose a folder", Gtk::FileChooserAction::FILE_CHOOSER_ACTION_SELECT_FOLDER);
	dialog.set_transient_for(*this);
	dialog.set_modal(true);
	dialog.set_position(Gtk::WIN_POS_CENTER); // Center the dialog

	dialog.set_current_folder(DEFAULT_HVI_PATH);

	// Add response buttons
	dialog.add_button("_Cancel", Gtk::RESPONSE_CANCEL);
	dialog.add_button("_Load", Gtk::RESPONSE_OK);

	// Show the dialog and wait for a response
	int result = dialog.run();

	if (result == Gtk::RESPONSE_OK)
	{
		current_file_path = dialog.get_filename();
		if (current_file_path.empty())
		{
			displayMessageLoadSave("Error: No folder selected");
			return;
		}
		loading.setValue(true);
	}
}

void MainWindow::onSaveButtonClicked()
{
	Gtk::FileChooserDialog dialog("Enter a folder name", Gtk::FileChooserAction::FILE_CHOOSER_ACTION_CREATE_FOLDER);
	dialog.set_transient_for(*this);
	dialog.set_modal(true);
	dialog.set_position(Gtk::WIN_POS_CENTER);

	// Get current date and format it
	auto now = std::chrono::system_clock::now();
	auto time = std::chrono::system_clock::to_time_t(now);
	std::tm *ltm = std::localtime(&time);

	// Get current frame rate
	double fps = getFrameRateEntryBox();

	// Format the prefix with date and fps
	char prefix[32];
	std::snprintf(prefix, sizeof(prefix), "%04d%02d%02d_%02.0ffps_<description>",
				  1900 + ltm->tm_year, // Year since 1900
				  1 + ltm->tm_mon,	   // Month (0-11)
				  ltm->tm_mday,		   // Day (1-31)
				  fps);				   // Frame rate

	dialog.set_current_folder(DEFAULT_HVI_PATH);
	dialog.set_current_name(prefix); // Sets the suggested name

	// Add response buttons
	dialog.add_button("_Cancel", Gtk::RESPONSE_CANCEL);
	dialog.add_button("_Save", Gtk::RESPONSE_OK);

	// Show the dialog and wait for a response
	int result = dialog.run();

	if (result == Gtk::RESPONSE_OK)
	{
		current_file_path = dialog.get_filename();
		if (current_file_path.empty())
		{
			displayMessageLoadSave("Error: No folder selected");
			return;
		}
		saving.setValue(true);
	}
}

void MainWindow::onPlayButtonClicked()
{
	setLiveView(false);
	setTrackingFPS(true);
	playingBuffer.setValue();
}

void MainWindow::onLiveToggled()
{
	liveView.setValue(liveToggle.get_active());
}

void MainWindow::onRecordClicked()
{
	if (pausedRecording.getValue())
		pausedRecording.toggle();
	else
		recording.setValue(); // I think this means setValue(true)
}

void MainWindow::onPauseClicked()
{
	if (recording.getValue())
	{
		pausedRecording.setValue(); // I think this means setValue(true)
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
		trackingFPS.setValue(true); // was just ()
		fileLoadButton.set_sensitive(false);
		recordingSizeScale.set_sensitive(false);
		if (bMainWindowLogFlag)
		{
			logger->info("[MainWindow::whenPlayingBufferToggled] Playing buffer toggled on");
		}
	}
	else
	{
		playButton.set_sensitive(true);
		trackingFPS.setValue(false);
		fileLoadButton.set_sensitive(true);
		recordingSizeScale.set_sensitive(true);
		if (bMainWindowLogFlag)
		{
			logger->info("[MainWindow::whenPlayingBufferToggled] Playing buffer toggled off");
		}
	}
}

void MainWindow::onFrameSliderChange(double val)
{
	std::cout << "Frame slider changed to:" << val << std::endl;
	seeking.setValue(true);
}

void MainWindow::onGainScaleChange(double val)
{
	sigFeatureUpdated.emit("Gain", val);
}

void MainWindow::onExposeScaleChange(double val)
{
	sigFeatureUpdated.emit("ExposureTime", val);
	// According to Vimba changing ExposureTime may also change camera frame rate
	// sigFeatureUpdated.emit("AcquisitionFrameRate", frameRateScale.getValue() );

	if (bMainWindowLogFlag)
	{
		logger->info("[MainWindow::onExposeScaleChange] ExposureTime changed");
	}
}

void MainWindow::onFrameRateChange(double val)
{
	sigFeatureUpdated.emit("AcquisitionFrameRate", val);
}

void MainWindow::onGammaScaleChange(double val)
{
	sigFeatureUpdated.emit("Gamma", val);
}

// void MainWindow::onThresScaleChange(double val)
// {
// 	sigThresholdChanged.emit(val);
// }

// void MainWindow::onScaleScaleChange(double val)
// {
// 	sigScaleChanged.emit(val);
// }

void MainWindow::onRecordingSizeScaleChange(double val)
{
	frameSlider.setUpperLimit(val - 1);
}

void MainWindow::onBestFocusScaleChange(double val)
{
	sigBestFocusChanged.emit(val);
}

void MainWindow::onHomePositionScaleChange(double val)
{
	sigHomePositionChanged.emit(val);
}

MainWindow::SignalHomePositionChanged MainWindow::signalHomePositionChanged()
{
	return sigHomePositionChanged;
}

MainWindow::~MainWindow()
{
	delete priv;

	if (bMainWindowLogFlag)
	{
		logger->info("[MainWindow::~MainWindow] destructor completed, priv deleted");
	}
}

void MainWindow::showOutOfBoundsWarning()
{
	outOfBoundsWarningLabel.set_visible(true);
}

void MainWindow::hideOutOfBoundsWarning()
{
	outOfBoundsWarningLabel.set_visible(false);
}
