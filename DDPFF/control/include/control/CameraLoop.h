#ifndef CAMERALOOP_H_
#define CAMERALOOP_H_

#include <QThread>
#include <QMatrix4x4>
#include <openni2/OpenNI.h>
#include <array>

#if DEPTHCAM == 2
#include <librealsense2/h/rs_frame.h>
#include <librealsense2/h/rs_option.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_processing.h>
#include <librealsense2/rs.h>
#include <librealsense2/rsutil.h>
#endif


class CameraLoop : public QThread {

    Q_OBJECT

public:

    CameraLoop();
    ~CameraLoop(){stop();}

    virtual void init() {}
    virtual void run() {}
    virtual void stop() {}
    virtual void step() {}
    virtual void toggleStream() {}
    virtual void enableStream() {}
    virtual void disableStream() {}
    virtual bool isStreamEnabled() const { return false; }

signals:
    void cameraDetected();
    void cameraLost();
    void messageOut(QString);
};


class AsusCameraLoop : public CameraLoop
{
	Q_OBJECT

    bool running;
    double lastUpdateTimestamp;
    int cameraConnectionState;
    bool streamEnabled;
    int failedFrames;

    openni::Status status;
    openni::Device device;
    openni::VideoStream depthSensor;
    openni::VideoStream colorSensor;
    openni::VideoFrameRef depthFrame;
    openni::VideoFrameRef colorFrame;

public:
    AsusCameraLoop();
    ~AsusCameraLoop();

    void init() override;
    void run() override;
    void stop() override;
    void step() override;
    void enableStream() override;
    void disableStream() override;
    void toggleStream() override;
    bool isStreamEnabled() const override;

private:
    bool connectCamera();
    void disconnectCamera();
    void processFrame();
};

#if DEPTHCAM == 2
class RealsenseCameraLoop : public CameraLoop
{
    Q_OBJECT

    bool running;
    double lastUpdateTimestamp;
    int cameraConnectionState;
    bool streamEnabled;
    int failedFrames;

    rs2_error *e;
    rs2_pipeline_profile *pipeline_profile;
    rs2_config *rs2config;
    rs2_pipeline *pipeline;
    rs2_device *dev;
    rs2_device_list *device_list;
    rs2_sensor_list *sensor_list;
    rs2_context *ctx;
    // align depth and RGB
    rs2_frame_queue *m_alignQueue;
    rs2_processing_block *m_alignProcessor;

    // temporal filter
    rs2_frame_queue *m_postprocess_temporal_Queue;
    rs2_processing_block *m_postprocess_temporal_Processor;

    // spatial filter
    rs2_frame_queue *m_postprocess_spatial_Queue;
    rs2_processing_block *m_postprocess_spatial_Processor;

    // not needed, image to local points is done by librealsense api
    //std::vector<std::vector<std::vector<double>>> unitImage;


public:
    RealsenseCameraLoop();
    ~RealsenseCameraLoop();

    void init() override;
    void run() override;
    void stop() override;
    void step() override;
    void toggleStream() override;
    void enableStream() override;
    void disableStream() override;
    bool isStreamEnabled() const override;

private:
    bool connectCamera();
    void reset();
    void processFrame();

    void rs_print_error(rs2_error *e);
    void rs_print_device_info(rs2_device *dev);
};
#endif

#endif // CAMERALOOP_H_
