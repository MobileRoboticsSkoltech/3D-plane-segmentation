#include "app.h"
#include "control/StateUtil.h"
#include "utils/utilities.h"

#include <QMainWindow>
#include <QMenuBar>
#include <QFileDialog>

void App::addItemsToMenu(
        QMenu* parent,
        const QString &name,
        const QString &tooltip,
        const QString &shortCut,
        const bool isCheckable,
        bool &target,
        const QString& enableMsg,
        const QString& disableMsg
    )
{
    QAction* newAction = parent->addAction(tr(name.toStdString().data()));
    newAction->setToolTip(tr(tooltip.toStdString().data()));
    newAction->setShortcut(QKeySequence(tr(shortCut.toStdString().data())));
    newAction->setCheckable(isCheckable);
    newAction->setChecked(target);

    // this connect call was inspired by https://stackoverflow.com/a/22411267/1906572
    connect(
        newAction,
        &QAction::triggered,
        this,
        [this, &target, enableMsg, disableMsg]{
            toggleDisplayOption(target, enableMsg, disableMsg);
        }
    );
}

App::App(QWidget *parent) :
    QMainWindow(parent)/*,
    experimenter(cameraControlLoop)*/
{
    QCoreApplication::setOrganizationName(APP_AUTHOR);
    QCoreApplication::setApplicationName(APP_NAME);

	QWidget* cw = new QWidget();
	ui.setupUi(cw);
	setCentralWidget(cw); // Because it's a QMainWindow.

    // Initialize the state and the config descriptors.
    curState.init();
	config.init();
    config.load();

    // Initialize the widgets and the world.
    cameraControlLoop.init();
	configWidget.init();
    cameraViewWidget.init();

	// Build the splitter separated gui components.
	verticalSplitterTop = new QSplitter();
	verticalSplitterTop->addWidget(&configWidget);
    verticalSplitterTop->addWidget(&openGLViewWidget);
    verticalSplitterTop->addWidget(&openGLSceneWidget);
	verticalSplitterBottom = new QSplitter();
    verticalSplitterBottom->addWidget(&cameraViewWidget);

    // The scene widget is hidden by default.
    openGLSceneWidget.setVisible(false);

	horizontalSplitter = new QSplitter(Qt::Vertical);
	horizontalSplitter->addWidget(verticalSplitterTop);
	horizontalSplitter->addWidget(verticalSplitterBottom);

	QSettings settings;
	verticalSplitterTop->restoreState(settings.value("verticalSplitterTop").toByteArray());
	verticalSplitterBottom->restoreState(settings.value("verticalSplitterBottom").toByteArray());
	horizontalSplitter->restoreState(settings.value("horizontalSplitter").toByteArray());

	QHBoxLayout* centralLayout = new QHBoxLayout(ui.centralWidget);
	centralLayout->setMargin(0);
	ui.centralWidget->setLayout(centralLayout);
	centralLayout->addWidget(horizontalSplitter);

	connect(verticalSplitterTop, SIGNAL(splitterMoved(int, int)), this, SLOT(topSplitterMoved()));
	connect(verticalSplitterBottom, SIGNAL(splitterMoved(int, int)), this, SLOT(bottomSplitterMoved()));

    // initialize the camera loop
    cameraLoop.init();

    connect(&openGLViewWidget, SIGNAL(updateUI()), this, SLOT(updateUI()));
    connect(&cameraViewWidget, SIGNAL(updateUI()), this, SLOT(updateUI()));
    connect(&openGLSceneWidget, SIGNAL(updateUI()), this, SLOT(updateUI()));

    connect(&cameraLoop, SIGNAL(cameraDetected()), this, SLOT(cameraDetectedIn()));
    connect(&cameraLoop, SIGNAL(cameraLost()), this, SLOT(cameraLostIn()));
    connect(&cameraLoop, SIGNAL(messageOut(QString)), this, SLOT(messageIn(QString)));

    /***********************************************************FILE MENU***********************************************************/

	// Build the menu bar.
	QMenuBar* menuBar = new QMenuBar();
	setMenuBar(menuBar);

    QMenu* fileMenu = menuBar->addMenu(tr("&File"));

	QAction* saveStateAction = fileMenu->addAction(tr("&Save State"));
    saveStateAction->setToolTip(tr("Saves the state history."));
    saveStateAction->setShortcut(QKeySequence(tr("Ctrl+S")));
	connect(saveStateAction, SIGNAL(triggered()), this, SLOT(saveStateHistory()));

	QAction* loadStateAction = fileMenu->addAction(tr("&Load State"));
    loadStateAction->setToolTip(tr("Loads the state history."));
    loadStateAction->setShortcut(QKeySequence(tr("Ctrl+L")));
    connect(loadStateAction, SIGNAL(triggered()), this, SLOT(loadStateHistory()));

    QAction* loadSceneAction = fileMenu->addAction(tr("&Load Scene"));
    loadSceneAction->setToolTip(tr("Loads the static scene."));
    loadSceneAction->setShortcut(QKeySequence(tr("Ctrl+Shift+L")));
    connect(loadSceneAction, SIGNAL(triggered()), this, SLOT(loadScene()));

    QAction* clearStateAction = fileMenu->addAction(tr("&Clear State"));
    clearStateAction->setToolTip(tr("Clears the state history."));
    //clearStateAction->setShortcut(QKeySequence(tr("Ctrl+L")));
    connect(clearStateAction, SIGNAL(triggered()), this, SLOT(clearStateHistory()));

	fileMenu->addSeparator();

	QAction* saveConfigAction = fileMenu->addAction(tr("&Save Config"));
	saveConfigAction->setToolTip(tr("Saves the config."));
	saveConfigAction->setShortcut(QKeySequence(tr("Ctrl+C")));
	connect(saveConfigAction, SIGNAL(triggered()), this, SLOT(saveConfig()));

	QAction* loadConfigAction = fileMenu->addAction(tr("&Load Config"));
	loadConfigAction->setToolTip(tr("Loads the config."));
	loadConfigAction->setShortcut(QKeySequence(tr("Ctrl+R")));
	connect(loadConfigAction, SIGNAL(triggered()), this, SLOT(loadConfig()));

    fileMenu->addSeparator();

    QAction* exportAction = fileMenu->addAction(tr("&Export shown data"));
    exportAction->setToolTip(tr("Exports the data currently visible in the graph widget."));
    exportAction->setShortcut(QKeySequence(tr("Ctrl+Shift+E")));

    /***********************************************************VIEW MENU***********************************************************/

    QMenu* viewMenu = menuBar->addMenu(tr("&View"));

    QAction* configViewAction = viewMenu->addAction(tr("&Config"));
    configViewAction->setToolTip(tr("Toggles the config widget."));
    configViewAction->setShortcut(QKeySequence(tr("C")));
    configViewAction->setCheckable(true);
    configViewAction->setChecked(false);
    connect(configViewAction, SIGNAL(triggered()), this, SLOT(toggleConfig()));

    QAction* cameraViewAction = viewMenu->addAction(tr("&Camera view"));
    cameraViewAction->setToolTip(tr("Toggles the camera view widget."));
    cameraViewAction->setShortcut(QKeySequence(tr("G")));
    cameraViewAction->setCheckable(true);
    cameraViewAction->setChecked(false);
    connect(cameraViewAction, SIGNAL(triggered()), this, SLOT(toggleCameraView()));

    viewMenu->addSeparator();

    QAction* showAxisAction = viewMenu->addAction(tr("&Axis"));
    showAxisAction->setToolTip(tr("Toggles the axis."));
    showAxisAction->setShortcut(QKeySequence(tr("A")));
    showAxisAction->setCheckable(true);
    showAxisAction->setChecked(openGLViewWidget.axisIsDrawn());
    connect(showAxisAction, SIGNAL(triggered()), &openGLViewWidget, SLOT(toggleAxis()));
    connect(showAxisAction, SIGNAL(triggered()), &openGLSceneWidget, SLOT(toggleAxis()));

    QAction* showFloorAction = viewMenu->addAction(tr("&Show floor"));
    showFloorAction->setToolTip(tr("Shows an imaginary floor."));
    showFloorAction->setShortcut(QKeySequence(tr("F")));
    showFloorAction->setCheckable(true);
    showFloorAction->setChecked(command.showFloor);
    connect(showFloorAction, SIGNAL(triggered()), this, SLOT(toggleFloor()));

    viewMenu->addSeparator();

    QAction* showPointCloudAction = viewMenu->addAction(tr("&Point Cloud"));
    showPointCloudAction->setToolTip(tr("Toggles the point cloud view."));
    showPointCloudAction->setShortcut(QKeySequence(tr("P")));
    showPointCloudAction->setCheckable(true);
    showPointCloudAction->setChecked(openGLViewWidget.showPointCloud);
    connect(showPointCloudAction, SIGNAL(triggered()), &openGLViewWidget, SLOT(togglePointCloud()));

    addItemsToMenu(
        viewMenu,
        "&Depth dependent planar segments",
        "Toggles App planar segments.",
        "Ctrl+D",
        true,
        command.showDDPFF,
        "DDPFF Planar Segments enabled.",
        "DDPFF Planar Segments disabled."
        );

    addItemsToMenu(
        viewMenu,
        "&Show unmerged segments",
        "Toggles unmerged planar segments.",
        "U",
        true,
        command.showUnmergedPlanes,
        "Unmerged Planar Segments enabled.",
        "Unmerged Planar Segments disabled."
        );

    addItemsToMenu(
        viewMenu,
        "&Show merged segments",
        "Toggles merged planar segments.",
        "Shift+U",
        true,
        command.showMergedPlanes,
        "Merged Planar Segments enabled.",
        "Merged Planar Segments disabled."
        );

    QAction* showNormalsAction = viewMenu->addAction(tr("&Normals"));
    showNormalsAction->setToolTip(tr("Toggles display of normal vectors."));
    showNormalsAction->setCheckable(true);
    showNormalsAction->setChecked(command.normals);
    connect(showNormalsAction, SIGNAL(triggered()), this, SLOT(toggleNormals()));

    QAction* showCameraTransformAction = viewMenu->addAction(tr("&Camera Transform"));
    showCameraTransformAction->setToolTip(tr("Toggles the camera transform."));
    showCameraTransformAction->setShortcut(QKeySequence(tr("M")));
    showCameraTransformAction->setCheckable(true);
    showCameraTransformAction->setChecked(command.showCameraTransform);
    connect(showCameraTransformAction, SIGNAL(triggered()), this, SLOT(toggleCameraTransform()));

    addItemsToMenu(
        viewMenu,
        "&Image Type",
        "Toggles the image type in the camera view widget.",
        "I",
        true,
        cameraViewWidget.showDepthImage,
        "Depth image enabled.",
        "Depth image disabled."
    );

    addItemsToMenu(
        viewMenu,
        "&Show Visualization.",
        "Toggles current visualization.",
        "V",
        true,
        command.showViz,
        "Visualization enabled.",
        "Visualization disabled."
    );

    addItemsToMenu(
        viewMenu,
        "&Show Flood Start Points.",
        "Toggles showing start points in flood fill.",
        "S",
        true,
        command.showStartPoints,
        "Show start points enabled.",
        "Show start points disabled."
    );

    addItemsToMenu(
        viewMenu,
        "&Show Plane Normals.",
        "Toggles showing plane normals.",
        "N",
        true,
        command.showPlaneNormals,
        "Show plane normals enabled.",
        "Show plane normals disabled."
        );

    addItemsToMenu(
        viewMenu,
        "&Show Text.",
        "Toggles showing text.",
        "T",
        true,
        command.showText,
        "Show text enabled.",
        "Show text disabled."
        );

    addItemsToMenu(
        viewMenu,
        "&Show Ground Truth Planes.",
        "Toggles showing ground truth planes.",
        "Ctrl+G",
        true,
        command.showGTPlanes,
        "GT planes enabled.",
        "GT planes disabled."
        );

    addItemsToMenu(
        viewMenu,
        "&Show Missing Depth.",
        "Toggles showing missing depth values.",
        "",
        true,
        command.showMissingDepth,
        "Missing depth enabled.",
        "Missing depth disabled."
        );

    addItemsToMenu(
        viewMenu,
        "&Gaps",
        "Gaps On",
        "",
        true,
        command.showGaps,
        "Gaps enabled.",
        "Gaps disabled."
        );

    addItemsToMenu(
        viewMenu,
        "&Experimental Feature On",
        "Experimental Feature On",
        "",
        true,
        command.experimentalFeatureOn,
        "Experimental feature enabled.",
        "Experimental feature disabled."
        );

    /***********************************************************COMMAND MENU***********************************************************/

    QMenu* commandMenu = menuBar->addMenu(tr("&Command"));

    cameraAction = commandMenu->addAction(tr("Camera"));
    cameraAction->setEnabled(false);
    cameraAction->setCheckable(true);
    cameraAction->setChecked(false);
    cameraAction->setToolTip(tr("Toggles the camera stream."));
    cameraAction->setShortcut(QKeySequence(tr("ALT+C")));
    connect(cameraAction, SIGNAL(triggered()), this, SLOT(toggleCamera()));

    QAction* fileBufferAction = commandMenu->addAction(tr("&Buffer to file"));
    fileBufferAction->setToolTip(tr("Write out state to file."));
    fileBufferAction->setShortcut(QKeySequence(tr("CTRL+B")));
    fileBufferAction->setCheckable(true);
    connect(fileBufferAction, SIGNAL(triggered()), this, SLOT(toggleFileBuffering()));

    addItemsToMenu(
        commandMenu,
        "&Print IoUs.",
        "Toggles printing IoUs of found planes.",
        "",
        true,
        command.printIoUs,
        "IoUs enabled.",
        "IoUs disabled."
    );

    QActionGroup* cameraTransformMethodActionGroup = new QActionGroup(commandMenu);

    QAction* cameraTransformMethod1Action = cameraTransformMethodActionGroup->addAction(tr("&M = GT"));
    cameraTransformMethod1Action->setToolTip(tr("&M = GT"));
    cameraTransformMethod1Action->setCheckable(true);
    cameraTransformMethod1Action->setChecked(command.cameraTransformMethod == 1);
    connect(cameraTransformMethod1Action, SIGNAL(triggered()), this, SLOT(selectCameraTransformMethod1()));

    QAction* cameraTransformMethod2Action = cameraTransformMethodActionGroup->addAction(tr("&M = I"));
    cameraTransformMethod2Action->setToolTip(tr("&M = I"));
    cameraTransformMethod2Action->setCheckable(true);
    cameraTransformMethod2Action->setChecked(command.cameraTransformMethod == 2);
    connect(cameraTransformMethod2Action, SIGNAL(triggered()), this, SLOT(selectCameraTransformMethod2()));

    commandMenu->addAction(cameraTransformMethod1Action);
    commandMenu->addAction(cameraTransformMethod2Action);

    /***********************************************************SCENES MENU***********************************************************/

    QMenu* scenesMenu = menuBar->addMenu(tr("&Scenes"));

    addItemsToMenu(
        scenesMenu,
        "&Adjust Scene",
        "Scene adjustment on",
        "",
        true,
        command.adjustScene,
        "Scene adjustment enabled.",
        "Scene adjustment disabled."
        );

    QActionGroup* chooseSceneActionGroup = new QActionGroup(scenesMenu);

    QAction* scene1Action = chooseSceneActionGroup->addAction(tr("&Scene1"));
    scene1Action->setToolTip(tr("&Scene1"));
    scene1Action->setCheckable(true);
    scene1Action->setChecked(command.scene == 1);
    connect(scene1Action, SIGNAL(triggered()), this, SLOT(selectScene1()));

    QAction* scene2Action = chooseSceneActionGroup->addAction(tr("&Scene2"));
    scene2Action->setToolTip(tr("&Scene2"));
    scene2Action->setCheckable(true);
    scene2Action->setChecked(command.scene == 2);
    connect(scene2Action, SIGNAL(triggered()), this, SLOT(selectScene2()));

    QAction* scene3Action = chooseSceneActionGroup->addAction(tr("&Scene3"));
    scene3Action->setToolTip(tr("&Scene3"));
    scene3Action->setCheckable(true);
    scene3Action->setChecked(command.scene == 3);
    connect(scene3Action, SIGNAL(triggered()), this, SLOT(selectScene3()));

    QAction* scene4Action = chooseSceneActionGroup->addAction(tr("&Scene4"));
    scene4Action->setToolTip(tr("&Scene4"));
    scene4Action->setCheckable(true);
    scene4Action->setChecked(command.scene == 4);
    connect(scene4Action, SIGNAL(triggered()), this, SLOT(selectScene4()));

    QAction* scene5Action = chooseSceneActionGroup->addAction(tr("&Scene5"));
    scene5Action->setToolTip(tr("&Scene5"));
    scene5Action->setCheckable(true);
    scene5Action->setChecked(command.scene == 5);
    connect(scene5Action, SIGNAL(triggered()), this, SLOT(selectScene5()));

    QAction* scene6Action = chooseSceneActionGroup->addAction(tr("&Scene6"));
    scene6Action->setToolTip(tr("&Scene6"));
    scene6Action->setCheckable(true);
    scene6Action->setChecked(command.scene == 6);
    connect(scene6Action, SIGNAL(triggered()), this, SLOT(selectScene6()));

    scenesMenu->addAction(scene1Action);
    scenesMenu->addAction(scene2Action);
    scenesMenu->addAction(scene3Action);
    scenesMenu->addAction(scene4Action);
    scenesMenu->addAction(scene5Action);
    scenesMenu->addAction(scene6Action);

    /***********************************************************RECORD MENU***********************************************************/

    recordAction = menuBar->addAction(tr("Record"));
    recordAction->setCheckable(true);
    recordAction->setChecked(true);
    recordAction->setEnabled(true);
    recordAction->setToolTip(tr("Toggles the robot control loop."));
    recordAction->setShortcut(QKeySequence(tr("Return")));
    connect(recordAction, SIGNAL(triggered()), this, SLOT(record()));

    /***********************************************************RESET MENU***********************************************************/
	
	QAction* resetAction = menuBar->addAction(tr("&Reset"));
    resetAction->setToolTip(tr("Resets the simulation state."));
	resetAction->setShortcut(QKeySequence(tr("R")));
	connect(resetAction, SIGNAL(triggered()), this, SLOT(reset()));

	menuBar->addSeparator();
    QAction* fakeSeparator3 = menuBar->addAction("     ");
    fakeSeparator3->setEnabled(false);

    /***********************************************************PLAYBACK MENU***********************************************************/

	QAction* jumpToStartAction = menuBar->addAction(tr("|<"));
	jumpToStartAction->setToolTip(tr("Sets the player to the first frame."));
	jumpToStartAction->setShortcut(QKeySequence(tr("Backspace")));
	connect(jumpToStartAction, SIGNAL(triggered()), this, SLOT(jumpToStart()));

	QAction* frameBackAction = menuBar->addAction(tr("<"));
	frameBackAction->setToolTip(tr("Rewinds the player by one frame."));
	connect(frameBackAction, SIGNAL(triggered()), this, SLOT(frameBack()));

	QAction* playAction = menuBar->addAction(tr("Play"));
	playAction->setToolTip(tr("Starts the playback."));
	playAction->setShortcut(QKeySequence(tr("Space")));
	connect(playAction, SIGNAL(triggered()), this, SLOT(play()));

	QAction* frameForwardAction = menuBar->addAction(tr(">"));
	frameForwardAction->setToolTip(tr("Advances the player by one frame."));
	connect(frameForwardAction, SIGNAL(triggered()), this, SLOT(frameForward()));

	QAction* jumpToEndAction = menuBar->addAction(tr(">|"));
	jumpToEndAction->setToolTip(tr("Sets the player to the last frame."));
    jumpToEndAction->setShortcut(QKeySequence(tr("CTRL+Backspace")));
    connect(jumpToEndAction, SIGNAL(triggered()), this, SLOT(jumpToEnd()));


    connect(this, SIGNAL(progressOut(int)), ui.frameSlider, SLOT(setValue(int)));
	connect(ui.frameSlider, SIGNAL(sliderMoved(int)), this, SLOT(jumpToFrame(int)));

    connect(&configWidget, SIGNAL(configChangedOut()), &openGLViewWidget, SLOT(update()));
    connect(&configWidget, SIGNAL(configChangedOut()), &openGLSceneWidget, SLOT(update()));
    connect(&configWidget, SIGNAL(configChangedOut()), this, SLOT(updateUI())); // uncomment this to render from a moving camera!
    connect(&configWidget, SIGNAL(configChangedOut()), &cameraControlLoop, SLOT(configChangedIn()));

    connect(&cameraControlLoop, SIGNAL(messageOut(QString)), this, SLOT(messageIn(QString)));

    toggleCameraView();
    toggleConfig();

	// Animation components.
    tscale = 1;
    recording = false;
    cfi = 0;
    animationTimer.setInterval(120);
	connect(&animationTimer, SIGNAL(timeout()), this, SLOT(animate()));
    connect(this, SIGNAL(frameIndexChangedOut(int)), &openGLViewWidget, SLOT(frameIndexChangedIn(int)));
    connect(this, SIGNAL(frameIndexChangedOut(int)), &cameraViewWidget, SLOT(frameIndexChangedIn(int)));

    record();

}

App::~App()
{
    QSettings settings;
    settings.setValue("verticalSplitterTop", verticalSplitterTop->saveState());
    settings.setValue("verticalSplitterBottom", verticalSplitterBottom->saveState());
    settings.setValue("horizontalSplitter", horizontalSplitter->saveState());
}

// This is needed when the entire config has changed, e.g. when the robot model was
// changed, the config was reset or a new robot was detected.
void App::configChanged()
{
    configWidget.configChangedIn();
    openGLViewWidget.update();
}

// Toggles the config widget.
void App::toggleConfig()
{
    if (configWidget.isHidden())
        configWidget.show();
    else
        configWidget.hide();
}

// This function is called by the animation timer to update the gui.
void App::animate()
{
    cfi = utils::bound(0, cfi-tscale, curState.size()-1);
    if (recording)
        cfi = 0;
    loadFrame(cfi);
}

// Toggles the record mode. In record mode the robot control thread is
// running and data is being buffer into the state history.
void App::record()
{
    if (recording)
    {
        recording = false;
        openGLViewWidget.recording = false;
        openGLViewWidget.update();
        animationTimer.stop();
        cameraControlLoop.stop();
        cameraLoop.disableStream();
        cameraLoop.stop();
    }
    else
    {
        recording = true;
        openGLViewWidget.recording = true;
        animationTimer.start();
        cameraLoop.enableStream();
        cameraLoop.start();
        cameraControlLoop.start();
    }
}

// Play button handler.
void App::play()
{
    if (animationTimer.isActive())
    {
        stop();
    }
    else
    {
        if (!recording)
        {
            tscale = 1;
            animationTimer.start();
        }
    }
}

// Stop button handler.
void App::stop()
{
    if (!recording)
    {
        animationTimer.stop();
        update();
    }
}

void App::frameBack()
{
    if (!recording)
    {
        animationTimer.stop();
        cfi = qMin(cfi+1, curState.size()-1);
        loadFrame(cfi);
    }
}

void App::frameForward()
{
    if (!recording)
    {
        animationTimer.stop();
        if (cfi > 0)
            cfi--;
        else
            cameraControlLoop.step();
        loadFrame(cfi);
    }
}

void App::jumpToStart()
{
    if (!recording)
    {
        animationTimer.stop();
        cfi = curState.size()-1;
        loadFrame(cfi);
    }
}

// Handles the frame slider.
void App::jumpToFrame(int f)
{
    if (!recording)
    {
        animationTimer.stop();
        cfi = qMax(0, int(curState.size()-1 - (double)((curState.size()-1) * f)/1000));
        loadFrame(cfi);
    }
}

void App::saveConfig()
{
    config.save("config");
    messageIn("Config saved.");
}

void App::loadConfig()
{
    config.load("config");
    configChanged();
    messageIn("Config reset.");
}

void App::saveStateHistory()
{
    QFileDialog saveDialog;
    saveDialog.setFileMode(QFileDialog::AnyFile);
    saveDialog.setAcceptMode(QFileDialog::AcceptSave);
    QString fileName = saveDialog.getSaveFileName(this, tr("Save .dat file"), QDir::homePath(), tr("Point cloud data (*.dat)"));
    if (fileName.isNull() || fileName.isEmpty()) {
        return;
    }
    curState.saveHistory(fileName);
    messageIn("State history saved.");
}

void App::clearStateHistory()
{
    curState.clear();
}

void App::loadStateHistory()
{
    if (recording)
        record();
    messageIn("Loading...");
    QString fileName = QFileDialog::getOpenFileName(
                this, tr("Open data file"), "/home/arc/data",
                tr("All Files (*);;Point cloud data (*.dat);;Tar archives (*.tgz);;Tar archives (*.tar.gz);;PCL (*.pcd.gt.zip);; Segcomp (*.abw.tar)")
    );
    curState.loadHistory(fileName, config.bufferSize);
    jumpToEnd();
    messageIn("State history loaded.");
    command.simulatedScene = false;
}

void App::loadScene()
{
    if (command.simulatedScene) {
        command.simulatedScene = false;
        openGLSceneWidget.setVisible(false);
        this->setFocus();
        return;
    }

    if (recording)
        record();
    messageIn("Loading Scene...");
    openGLSceneWidget.swapScene(command.scene);
    openGLSceneWidget.setVisible(true);
    jumpToEnd();
    messageIn("Scene loaded.");
    command.simulatedScene = true;
}

// Global event filter.
bool App::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyPress)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
        if (command.adjustScene && command.simulatedScene) {
            openGLSceneWidget.setFocus();
            openGLSceneWidget.keyPressEvent(keyEvent);
            return true;
        }
        keyPressEvent(keyEvent);
        return true;
    }
    else if (event->type() == QEvent::KeyRelease)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
        keyReleaseEvent(keyEvent);
        return true;
    }
    else
    {
        // standard event processing
        return QObject::eventFilter(obj, event);
    }
}

// Vertical splitter synchronization.
void App::topSplitterMoved()
{
	verticalSplitterBottom->setSizes(verticalSplitterTop->sizes());
}
void App::bottomSplitterMoved()
{
	verticalSplitterTop->setSizes(verticalSplitterBottom->sizes());
}

void App::cameraDetectedIn()
{
    cameraAction->setEnabled(true);
    cameraAction->setChecked(true);
}

void App::cameraLostIn()
{
    cameraAction->setChecked(false);
    cameraAction->setEnabled(false);
}

// Slot for internal message strings.
void App::messageIn(QString m)
{
    openGLViewWidget.messageIn(m);
}

// Toggles the graph widget.
void App::toggleCameraView()
{
    if (verticalSplitterBottom->isHidden())
		verticalSplitterBottom->show();
	else
		verticalSplitterBottom->hide();
}

void App::jumpToEnd()
{
    if (!recording)
    {
        cfi = 0;
        emit progressOut(qMax(0, (int)(1000.0 * ((curState.size()-1)-cfi)/(curState.size()-1))));
        emit frameIndexChangedOut(cfi);
    }
}

void App::reset()
{
    cameraControlLoop.reset();
	messageIn("Reset");
}

// This slot is called when the user browses the state history with the slider or with
// the frame forward, frame backward, and playback functions.
void App::loadFrame(int frameIndex)
{
    if (frameIndex > 0)
    {
        // The browsing of the state history is implemented in a generic way such that
        // the current state is overwritten with an older version of the state from
        // state history.
        curState.restore(frameIndex);

        // We also step the robot control once to recompute things.
        cameraControlLoop.smallStep(frameIndex);
    }

    cfi = frameIndex;
    emit progressOut(qMax(0, (int)(1000.0 * ((curState.size()-1)-cfi)/(curState.size()-1))));
    emit frameIndexChangedOut(cfi);
}

void App::toggleNormals()
{
    command.normals = !command.normals;
    openGLViewWidget.update();
}

void App::toggleFloor()
{
    command.showFloor = !command.showFloor;
    update();
}

void App::selectCameraTransformMethod1(){
    command.cameraTransformMethod = 1;
}

void App::selectCameraTransformMethod2(){
    command.cameraTransformMethod = 2;
}

void App::selectScene1()
{
    command.scene = 1;
    openGLSceneWidget.swapScene(command.scene);
}

void App::selectScene2()
{
    command.scene = 2;
    openGLSceneWidget.swapScene(command.scene);
}

void App::selectScene3()
{
    command.scene = 3;
    openGLSceneWidget.swapScene(command.scene);
}

void App::selectScene4()
{
    command.scene = 4;
    openGLSceneWidget.swapScene(command.scene);
}

void App::selectScene5()
{
    command.scene = 5;
    openGLSceneWidget.swapScene(command.scene);
}

void App::selectScene6()
{
    command.scene = 6;
    openGLSceneWidget.swapScene(command.scene);
}

void App::toggleDisplayOption(bool &option, const QString &enableMsg, const QString &disableMsg)
{
    option = !option;
    if (option)
    {
        messageIn(enableMsg);
    }
    else
    {
        messageIn(disableMsg);
    }
    openGLViewWidget.update();
    cameraViewWidget.update();
}

// Toggles the camera stream.
void App::toggleCamera()
{
    cameraLoop.toggleStream();
    if (cameraLoop.isStreamEnabled()){
        messageIn("Camera stream enabled.");
    }
    else{
        messageIn("Camera stream disabled.");
    }
}

void App::toggleCameraTransform()
{
    command.showCameraTransform = !command.showCameraTransform;
    update();
}


void App::updateUI()
{
    openGLViewWidget.update();
    cameraViewWidget.update();
    openGLSceneWidget.update();
}


void App::toggleFileBuffering()
{
    command.bufferToFile = !command.bufferToFile;
    if (command.bufferToFile)
    {
        messageIn("File buffering is enabled.");
    }
    else
    {
        messageIn("File buffering is disabled.");
    }
}

// Keyboard handling.
void App::keyPressEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat())
        return;

    if (event->key() == Qt::Key_Backspace && event->modifiers() & Qt::ControlModifier)
    {
        jumpToEnd();
    }
    else if (event->key() == Qt::Key_Backspace)
    {
        jumpToStart();
    }
    else if (event->key() == Qt::Key_Escape)
    {
        close();
    }
    else if (event->key() == Qt::Key_Up)
    {

        if (!animationTimer.isActive())
            animationTimer.start();
        tscale = 10;
    }
    else if (event->key() == Qt::Key_Down)
    {

        if (!animationTimer.isActive())
            animationTimer.start();
        tscale = -10;
    }
    else if (event->key() == Qt::Key_Right && event->modifiers() & Qt::ControlModifier)
    {
        frameForward();
    }
    else if (event->key() == Qt::Key_Left && event->modifiers() & Qt::ControlModifier)
    {
        frameBack();
    }
    else if (event->key() == Qt::Key_Right)
    {
        frameForward();
    }
    else if (event->key() == Qt::Key_Left)
    {
        frameBack();
    }
}

void App::keyReleaseEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat())
        return;

    if (event->key() == Qt::Key_Up)
    {

        tscale = 1;
        if (!recording)
            animationTimer.stop();
    }
    else if (event->key() == Qt::Key_Down)
    {

        tscale = 1;
        if (!recording)
            animationTimer.stop();
    }
}
