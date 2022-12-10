#include "mainwindow.hpp"

#include <GL/gl.h>
#include <cvd/gl_helpers.h>
#include <cvd/image.h>
#include <gtkmm.h>
#include <string>

#include "VimbaC/Include/VmbCommonTypes.h"
#include "glibmm/main.h"
#include "sigc++/functors/mem_fun.h"
#include "version.hpp"
#include "main.hpp"
#include "logfile.hpp"


struct MainWindow::Private
{
    Gtk::Label gainLabel, exposeLabel, gammaLabel, thresLabel, scaleLabel, bestFocusLabel;
	Gtk::Frame controlFrame;
    Gtk::Grid controlGrid;
    Gtk::VBox rootBox;
    
    Gtk::Label space4[8];
    
    Gtk::HBox mediaBox, fileChooserBox;
	Gtk::Box displayBox;
    
    public:
        Private();
};

MainWindow::Private::Private() :
    gainLabel("Gain: "),
    exposeLabel("Expose: "),
    gammaLabel("Gamma: "),
    thresLabel("Threshold: "),
    scaleLabel("Scale: "),
    bestFocusLabel("Best Focal Plane"),
	controlFrame(),
    controlGrid(),
    rootBox(),
    space4{Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    "), Gtk::Label("    ")},
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

    thresLabel.set_justify(Gtk::Justification::JUSTIFY_RIGHT);
    thresLabel.set_halign(Gtk::Align::ALIGN_END);
    scaleLabel.set_justify(Gtk::Justification::JUSTIFY_RIGHT);
    scaleLabel.set_halign(Gtk::Align::ALIGN_END);

	thresLabel.set_sensitive(false);
	scaleLabel.set_sensitive(false);
	bestFocusLabel.set_sensitive(false);
    
    controlGrid.set_hexpand();
    controlGrid.set_halign(Gtk::Align::ALIGN_FILL);
    controlGrid.set_row_spacing(3);
    
    rootBox.set_hexpand();
    rootBox.set_halign(Gtk::Align::ALIGN_FILL);

    rootBox.set_vexpand();
    rootBox.set_valign(Gtk::Align::ALIGN_FILL);
    
    for (Gtk::Label &label : space4)
        label.set_justify(Gtk::Justification::JUSTIFY_CENTER);
    
    mediaBox.set_hexpand(false);
    mediaBox.set_halign(Gtk::Align::ALIGN_CENTER);

	displayBox.set_hexpand(true);
	displayBox.set_vexpand(true);
	displayBox.set_halign(Gtk::Align::ALIGN_FILL);
	displayBox.set_valign(Gtk::Align::ALIGN_FILL);


	Glib::RefPtr<Gtk::CssProvider> cssBlackBG = Gtk::CssProvider::create();
	cssBlackBG->load_from_data("* {background-color: #000000;}");
	displayBox.get_style_context()->add_provider(cssBlackBG, 0);
}

ScaleWidget::ScaleWidget(double lower, double upper, double inc, double def) : Gtk::Bin(),
    sigChanged(),
	sigUserChanged(),
    scale(),
    spinButton(),
    grid()
{
    scale.set_range(lower, upper);
    scale.set_increments(inc, inc*10);
    scale.set_value(def);
    scale.set_draw_value(false);
    scaleConnection = scale.signal_value_changed().connect(sigc::mem_fun(*this, &ScaleWidget::scaleChanged) );
	scale.signal_change_value().connect(sigc::mem_fun(*this, &ScaleWidget::changeScale) );
    
    spinButton.set_range(lower, upper);
    spinButton.set_increments(inc, inc*10);
    spinButton.set_value(def);
    spinButtonConnection = spinButton.signal_value_changed().connect(sigc::mem_fun(*this, &ScaleWidget::spinButtonChanged) );
    
    grid.add(spinButton);
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

ScaleWidget::SignalUserChanged ScaleWidget::signalUserChanged()
{
	return sigUserChanged;
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
    spinButton.set_value(scale.get_value() );
    spinButtonConnection.unblock();
    sigChanged.emit(scale.get_value() );
}

bool ScaleWidget::changeScale(Gtk::ScrollType scroll, double newValue)
{
	sigUserChanged.emit(scroll, newValue);
	return false;
}

MainWindow::MainWindow() : Gtk::Window(),
    priv(new Private() ),
    gainScale(0, 50, 1, 0),
    exposeScale(100, 1000000, 1000, 15000),
    gammaScale(0.5, 2.5, 0.01, 1),
    frameSlider(0, HVIGTK_RECORD_FRAMECOUNT-1, 1, 0),
    thresScale(0, 1, 0.01, 0.4),
    scaleScale(0, 5, 0.01, 2.0),
    bestFocusScale(0, 300, 1, 100),
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
	display(),
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
	trackingFPS(),
	sigFrameDrawn(),
	sigFeatureUpdated(),
	sigThresholdChanged(),
	sigScaleChanged(),
	sigBestFocusChanged(),
	gainScaleConnection(),
	exposeScaleConnection(),
	gammaScaleConnection(),
	frameSliderConnection()
{
    set_default_size(1700,800);
    set_title("HVI-GTK " HVIGTK_VERSION_STR);
    
    gainScale.setScaleSizeRequest(150, 0);
    gainScale.setSpinButtonWidth(7);
	gainScaleConnection = gainScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onGainScaleChange) );
    
    exposeScale.setScaleSizeRequest(150, 0);
    exposeScale.setSpinButtonWidth(7);
	exposeScaleConnection = exposeScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onExposeScaleChange) );
    
    gammaScale.setScaleSizeRequest(150, 0);
    gammaScale.setSpinButtonWidth(7);
    gammaScale.setSpinButtonPrec(2);
	gammaScaleConnection = gammaScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onGammaScaleChange) );
    
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
    
    frameSlider.setScaleSizeRequest(200, 0);
    frameSlider.setSpinButtonWidth(5);
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
    
    thresScale.setScaleSizeRequest(100, 0);
    thresScale.setSpinButtonWidth(4);
    thresScale.setSpinButtonPrec(2);
	thresScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onThresScaleChange) );
	thresScale.set_sensitive(false);

    scaleScale.setScaleSizeRequest(100, 0);
    scaleScale.setSpinButtonWidth(4);
    scaleScale.setSpinButtonPrec(2);
	scaleScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onScaleScaleChange) );
    scaleScale.set_sensitive(false);

    tdStabToggle.set_sensitive(true);
    
    bestFocusScale.setScaleSizeRequest(100, 0);
    bestFocusScale.setSpinButtonWidth(4);
	bestFocusScale.signalChanged().connect(sigc::mem_fun(*this, &MainWindow::onBestFocusScaleChange) );
    bestFocusScale.set_sensitive(false);

	display.set_hexpand(true);
	display.set_vexpand(true);
	display.set_halign(Gtk::Align::ALIGN_FILL);
	display.set_valign(Gtk::Align::ALIGN_FILL);
	
	display.signal_draw().connect(sigc::mem_fun(*this, &MainWindow::renderDisplay) );

	setLiveView(true);

    //CONDITIONS 

	makeMapActive.toggleOnSignal(makeMapToggle.signal_toggled() );
    makeMapActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenMakeMapToggled) );

	stabiliseActive.toggleOnSignal(stabiliseToggle.signal_toggled() );
	stabiliseActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenStabiliseToggled) );

	showMapActive.toggleOnSignal(stabiliseToggle.signal_toggled() );
	showMapActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenShowMapToggled) );
	
	findFocusActive.toggleOnSignal(findFocusToggle.signal_toggled() );
	findFocusActive.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenFindFocusToggled) );

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

	trackingFPS.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenTrackingFPSToggled) );
	
	playingBuffer.signalToggled().connect(sigc::mem_fun(*this, &MainWindow::whenPlayingBufferToggled) );

	sigFrameDrawn.connect(sigc::mem_fun(*this, &MainWindow::onFrameDrawn) );

	//ATTACHING
    
    priv->fileChooserBox.add(fileNameEntry);
    priv->fileChooserBox.add(fileChooseButton);
    
    priv->controlGrid.attach(priv->gainLabel, 12, 0);
    priv->controlGrid.attach(gainScale, 13, 0);
    
    priv->controlGrid.attach(priv->exposeLabel, 12, 1);
    priv->controlGrid.attach(exposeScale, 13, 1);
    
    priv->controlGrid.attach(priv->gammaLabel, 12, 2);
    priv->controlGrid.attach(gammaScale, 13, 2);
    
    priv->controlGrid.attach(priv->space4[0], 0, 0);
    priv->controlGrid.attach(priv->mediaBox, 10, 0);
    priv->controlGrid.attach(frameSlider, 10, 1);
    
    priv->controlGrid.attach(priv->space4[1], 2, 0);
    priv->controlGrid.attach(priv->fileChooserBox, 15, 0, 2, 1);
    
    priv->controlGrid.attach(fileLoadButton, 15, 1);
    priv->controlGrid.attach(fileSaveButton, 16, 1);
    
    priv->controlGrid.attach(priv->space4[2], 4, 0);
    priv->controlGrid.attach(makeMapToggle, 5, 0);
    priv->controlGrid.attach(stabiliseToggle, 5, 1);
    priv->controlGrid.attach(showMapToggle, 5, 2);
    
    priv->controlGrid.attach(priv->space4[3], 6, 0);
    priv->controlGrid.attach(priv->thresLabel, 7, 0);
    priv->controlGrid.attach(thresScale, 8, 0);
    priv->controlGrid.attach(priv->scaleLabel, 7, 1);
    priv->controlGrid.attach(scaleScale, 8, 1);
    
    priv->controlGrid.attach(priv->space4[4], 9, 0);
    priv->controlGrid.attach(findFocusToggle, 1, 0);
    priv->controlGrid.attach(holdFocusToggle, 1, 1);
    priv->controlGrid.attach(tdStabToggle, 1, 2);
    
    priv->controlGrid.attach(priv->space4[5], 11, 0);
    priv->controlGrid.attach(priv->bestFocusLabel, 3, 0);
    priv->controlGrid.attach(bestFocusScale, 3, 1);
    priv->controlGrid.attach(fpsLabel, 10, 2);
    priv->controlGrid.attach(priv->space4[6], 14, 0);
    priv->controlGrid.attach(priv->space4[7], 17, 0);

	priv->controlFrame.add(priv->controlGrid);

	priv->controlFrame.set_vexpand(false);
	priv->controlFrame.set_valign(Gtk::Align::ALIGN_START);

    priv->displayBox.pack_start(display, true, true);

    priv->rootBox.pack_start(priv->controlFrame, false, true);
    priv->rootBox.pack_start(priv->displayBox, true, true);
	//priv->rootBox.add(display);
    
    add(priv->rootBox);
    
    priv->rootBox.show_all();
}

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

void MainWindow::displayMessage(const std::string &msg)
{
	fpsLabel.set_text(msg);
}

void MainWindow::renderFrame(VidFrame *frame)
{
	drawFrame = frame;
	newDrawFrame = true;
	display.queue_draw();
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
 
bool MainWindow::renderDisplay(const ::Cairo::RefPtr< ::Cairo::Context>& cr)
{
	//glClear(GL_COLOR_BUFFER_BIT);
	// glTexImage2D (GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, width, height, 0, GL_BGRA, GL_UNSIGNED_BYTE, surface_data[0]);
	if (drawFrame != nullptr)
	{
		cr->save();
		VmbUchar_t *buf = drawFrame->data();
		im = CVD::Image(drawFrame->size(), *drawFrame->data());
		TooN::Vector<2> this_offset = my_stabiliser.stabilise(im, offset);
        glBitmap(0,0,0,0,-this_offset[0],this_offset[1],0);
        offset+=this_offset;

		hvigtk_logfile << offset[0] << std::endl;
		hvigtk_logfile << offset[1] << std::endl;
		hvigtk_logfile.flush();



		int stride = Cairo::ImageSurface::format_stride_for_width(Cairo::Format::FORMAT_A8, drawFrame->size().x);
		auto surface = Cairo::ImageSurface::create(buf, Cairo::Format::FORMAT_A8, drawFrame->size().x, drawFrame->size().y, stride);

		double xscale = (double) (display.get_width() ) / drawFrame->size().x;
		double yscale = (double) (display.get_height() ) / drawFrame->size().y;

		// // scale down if the window is smaller than the image
		// double arscale;
		// if (xscale < 1.0 || yscale < 1.0)
		// {
		// 	arscale = surface->get_width() > surface->get_height() ? yscale : xscale;
		// 	cr->scale(arscale, arscale);
		// }
		// else arscale = 1.0;

		cr->set_source_rgba(1,1,1,1);
		cr->rectangle( display.get_width() / 2.0 - drawFrame->size().x/2.0 -this_offset[0],0 + this_offset[1], surface->get_width(), surface->get_height() );
		cr->fill();
		
		cr->set_operator(Cairo::Operator::OPERATOR_DEST_ATOP);
		cr->set_source(surface, display.get_width() / 2.0 - drawFrame->size().x/2.0 -this_offset[0], 0+ this_offset[1]);
		cr->paint();

		cr->restore();

		if (newDrawFrame)
		{
			newDrawFrame = false;
			sigFrameDrawn.emit();
		}
	}

	return true;
}

void MainWindow::onFrameDrawn()
{
	if (playingBuffer.getValue() )
	{
		frameSliderConnection.block();
		frameSlider.setValue(frameSlider.getValue() + 1);
		frameSliderConnection.unblock();
		if (trackingFPS.getValue() )
			countFrames++;
	}
	else if (liveView.getValue() )
	{
		countFrames++;
	}
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
    	showMapToggle.set_active(true);
		hvigtk_logfile << "show map should be getting toggled now" << std::endl;
		hvigtk_logfile.flush();


		priv->thresLabel.set_sensitive(true);
		priv->scaleLabel.set_sensitive(true);

		thresScale.set_sensitive(true);
		scaleScale.set_sensitive(true);

		//GUI CHANGES WHEN "MAKE MAP" IS ENABLED GO HERE
	}
	else
	{
    	stabiliseToggle.set_active(false);
    	stabiliseToggle.set_sensitive(false);
    	showMapToggle.set_active(false);
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
// Needs to be turned into a button at a later stage
{
	if (findingFocus)
	{
		holdFocusToggle.set_active(true);
	
		bestFocusScale.set_sensitive(true);
		priv->bestFocusLabel.set_sensitive(true);

		usleep(1000000); // CALL AUTOFOCUS HERE IF INACTIVE. SET LOC_BESTFOCUS TO 320
		
		//GUI CHANGES WHEN "FINDING FOCUS" IS ENABLED GO HERE
	}
	else
	{
		// This is never called, unless the toggle is spammed, which we need to avoid.
		// holdFocusToggle.set_active(false);
		// holdFocusToggle.set_sensitive(false);
		// tdStabToggle.set_active(false);
		// tdStabToggle.set_sensitive(false);
		//GUI CHANGES WHEN "FINDING FOCUS" IS DISABLED GO HERE
	}
}

void MainWindow::whenHoldFocusToggled(bool holdingFocus)
{
	if (holdingFocus)
	{
		bestFocusScale.set_sensitive(true);
		priv->bestFocusLabel.set_sensitive(true);

		//GUI CHANGES WHEN "HOLD FOCUS" IS ENABLED GO HERE
	}
	else
	{
		bestFocusScale.set_sensitive(false);
		priv->bestFocusLabel.set_sensitive(false);

		//GUI CHANGES WHEN "FIND FOCUS" IS DISABLED GO HERE
	}
}

void MainWindow::when3DStabToggled(bool active)
{
	if (active)
	{
		holdFocusToggle.set_active(true);
		stabiliseToggle.set_active(true);
		makeMapToggle.set_active(true);

		showMapToggle.set_active(false);

		//GUI CHANGES WHEN "3D STABILISER" IS ENABLED GO HERE
	}
	else
	{
		holdFocusToggle.set_active(false);
		stabiliseToggle.set_active(false);
		makeMapToggle.set_active(false);

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
    backToStartButton.set_sensitive(false);
    pauseButton.set_sensitive(false);
    playButton.set_sensitive(false);
    fileSaveButton.set_sensitive(false);
	liveToggle.set_sensitive(false);
	liveToggle.set_active(true);
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
		frameSliderConnection.block();
		priv->mediaBox.set_sensitive(false);
		trackingFPS.setValue(false);
	}
	else
	{
		frameSliderConnection.unblock();
		priv->mediaBox.set_sensitive(true);
		trackingFPS.setValue();
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
	recording.setValue();
}

void MainWindow::onPauseClicked()
{
	playingBuffer.setValue(false);
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
	}
	else
	{
		playButton.set_sensitive(true);
		trackingFPS.setValue(false);
		fileLoadButton.set_sensitive(true);
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
	sigFeatureUpdated.emit("AcquisitionFrameRate", 30.0);
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

void MainWindow::onBestFocusScaleChange(double val)
{
	sigBestFocusChanged.emit(val);
}

MainWindow::~MainWindow()
{
    delete priv;
}
