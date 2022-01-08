#ifndef App_H
#define App_H

#include "ui_app.h"
#include <QMainWindow>
#include <QSplitter>
#include "gui/OpenGLSceneWidget.h"
#include "gui/ConfigWidget.h"
#include "gui/CameraViewWidget.h"
#include "gui/OpenGLViewWidget.h"
#include "control/State.h"
#include "globals/Config.h"
#include "globals/Command.h"
#include "control/CameraControlLoop.h"
#include "control/CameraLoop.h"
#include "control/CameraLoop.h"
#include "scenes/scene.h"

class App : public QMainWindow
{
    Q_OBJECT

    Ui::AppClass ui;

    ConfigWidget configWidget;
    CameraViewWidget cameraViewWidget;
    OpenGLViewWidget openGLViewWidget;
    OpenGLSceneWidget openGLSceneWidget;

    QSplitter* verticalSplitterTop;
    QSplitter* verticalSplitterBottom;
    QSplitter* horizontalSplitter;

    QAction* cameraAction;

    QAction* recordAction;

    int cfi;
    int tscale;
    bool recording;
    QTimer animationTimer;

    CameraControlLoop cameraControlLoop;
    AsusCameraLoop cameraLoop;

    /* Helper. Add menu items. */
    void addItemsToMenu(
        QMenu* parent,
        const QString& name,
        const QString& tooltip,
        const QString& shortCut,
        const bool isCheckable,
        bool& target,
        const QString& enableMsg,
        const QString& disableMsg
    );

public:
    App(QWidget *parent = 0);
    ~App();

public slots:
    void configChanged();
    void topSplitterMoved();
    void bottomSplitterMoved();
    void messageIn(QString m);
    void cameraLostIn();
    void cameraDetectedIn();
    void toggleConfig();
    void toggleCameraView();
    void toggleCamera();
    void toggleCameraTransform();
    void animate();
    void record();
    void play();
    void stop();
    void frameBack();
    void frameForward();
    void jumpToStart();
    void jumpToEnd();
    void jumpToFrame(int);

    void reset();
    void saveConfig();
    void loadConfig();
    void saveStateHistory();
    void clearStateHistory();
    void loadStateHistory();
    void loadScene();
    void loadFrame(int fi);

    void toggleFileBuffering();
    void toggleNormals();

    void toggleFloor();
    void selectCameraTransformMethod1();
    void selectCameraTransformMethod2();

    void selectScene1();
    void selectScene2();
    void selectScene3();
    void selectScene4();
    void selectScene5();
    void selectScene6();

    void toggleDisplayOption(bool& option, const QString& enableMsg, const QString& disableMsg);

    void updateUI();

signals:
    void frameIndexChangedOut(int);
    void progressOut(int);

protected:
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
    bool eventFilter(QObject *obj, QEvent *event);
};

#endif // NaoLab_H
