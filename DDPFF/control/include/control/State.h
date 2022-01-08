#ifndef STATE_H_
#define STATE_H_

#include <QDebug>
#include <QList>
#include <QStringList>
#include <QMutex>
#include <typeinfo>
#include <array>
#include <memory>

#include "globals/constants.h"
#include "utils/Pose6D.h"
#include "rep/Representation.h"

// Represents the current state of the camera perception of the world.
struct State
{
    // Stuff that's always needed.
    int frameId;
    real_t time; // Current robot control time since program start.
    real_t debug; // An all purpose debug value.
    real_t realTime; // Current real time since program start.
    real_t rcIterationTime; // How long did the last RC iteration really take?
    real_t rcExecutionTime; // The execution time of the last RC iteration.
    real_t avgExecutionTime; // Running average of the execution time.
    real_t cameraLoopIterationTime; // How long did the last camera iteration really take?
    real_t cameraLoopExecutionTime; // The execution time of the last camera iteration.

    Pose6D cameraPose; // The ground truth pose of the camera.

    // Camera transforms : The probably need to be moved to CameraControl
    Transform3D cameraTransform; // M
    Transform3D cameraTransformT; // M_t
    Transform3D invCameraTransform; // M^-1
    Transform3D invCameraTransformT; // M^-1_t
    Transform3D gtTransform;
    Transform3D floorProjection;

    // Point, color and depth buffers
    pointBuffer_t pointBuffer;
    colorBuffer_t colorBuffer;
    depthBuffer_t depthBuffer;

    // These need to be moved to CameraControl?
    normalBuffer_t gtNormalBuffer;
    planeBuffer_t gtPlaneBuffer;

    // camera's unit image
    static const pointBuffer_t unitImage;

    // The representation currently active
    Representation* currentRepresentation = nullptr;

 public:

    State();
    ~State(){}
    State& operator=(const State &other);
    void init();
    void clear();
    void bufferAppend(int maxLength = 0);
    void bufferOverwrite(int frameIndex);
    void restore(int frameIndex);
    void bufferToFile();
    void saveHistory(QString& fileName) const;
    void loadHistory(const QString& fileName, int maxLength);
    int size() const;
    State& operator[](int i);
    real_t operator()(int i) const;
    real_t operator()(QString key) const;
    real_t getMember(int i) const;
    real_t getMember(QString key) const;
    void setMember(int i, real_t v);
    void setMember(QString key, real_t v);
    void update();
    State& prev();
    State& next();

    static QStringList memberNames; // Contains the names of the members in the right order.

private:

    // Registers a member variable for index based access.
    template <typename T>
    void registerMember(QString name, T* member)
    {
        memberNames << name;
        memberOffsets << (quint64)member - (quint64)this;
        memberTypes << QString(typeid(*member).name());
        //qDebug() << name << memberTypes.last();
    }

    void loadHistoryDeserialize(const QString& fileName, int maxLength);
    void loadHistoryTum(const QString& fileName, int maxLength);
    void loadHistoryBonn(const QString& fileName, int maxLength);
    void loadHistorySegcomp(const QString& fileName, int maxLength);

    // These members are static so that buffering into history does not create copies.
    static QList<quint64> memberOffsets;
    static QList<QString> memberTypes;
    static QMutex mutex;
    static QList<State> history;
};

extern State curState;

#endif /* STATE_H_ */

