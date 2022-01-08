/*
 * CameraLoop.cpp
 *
 * The CameraLoop polls images from the depth camera and writes them into the state.
 * The loop can be start()-ed and run as a thread. It will then do everything
 * automatically. It will detect a camera as soon as one is connected and start
 * reading frames from it and writing them into the point buffer in state. The
 * camera detection event is reported as a cameraDetected() signal. If the reading
 * fails too many times, it will be reported as a cameraLost() signal. You can
 * explicitly enable or disable the camera data stream for whatever reason, but
 * the thread will continue to run and monitor the camera state.
 */

#include "control/CameraLoop.h"
#include "control/State.h"
#include "globals/Config.h"
#include "utils/StopWatch.h"
#include "utils/eigenutils.h"

using namespace openni;

CameraLoop::CameraLoop() : QThread(NULL)
{

}

AsusCameraLoop::AsusCameraLoop()
{
    running = false;
    streamEnabled = true;
    lastUpdateTimestamp = 0;
    cameraConnectionState = 0;
    failedFrames = 0;
}

AsusCameraLoop::~AsusCameraLoop()
{
    stop();
    wait();
}

// Init function, called first thing after construction.
void AsusCameraLoop::init()
{
//    initializeUnitImage();
}

// Step function, called periodically to maintain communication.
void AsusCameraLoop::step()
{
    if (connectCamera())
        processFrame();
}

void AsusCameraLoop::enableStream()
{
    streamEnabled = true;
}

void AsusCameraLoop::disableStream()
{
    streamEnabled = false;
}

// Toggles the data stream between the camera and the state object.
// This will have no influence on the thread, it will keep running and monitoring the camera.
void AsusCameraLoop::toggleStream()
{
    streamEnabled = !streamEnabled;
}

// Tells you if the camera stream is enabled.
bool AsusCameraLoop::isStreamEnabled() const
{
    return streamEnabled;
}

// The "loop" of the thread. Calls the step() function and measures execution time.
void AsusCameraLoop::run()
{
    StopWatch stopWatch;

    running = true;
    while (running)
    {
        stopWatch.start();
        curState.realTime = stopWatch.time();
        curState.cameraLoopIterationTime = curState.realTime - lastUpdateTimestamp;
        lastUpdateTimestamp = curState.realTime;

        step();

        curState.cameraLoopExecutionTime = stopWatch.elapsedTime();
    }

    disconnectCamera();
}

// Stops the thread execution (blocking!).
void AsusCameraLoop::stop()
{
    running = false;
}

// Executes the camera connect and setup routine.
// This function is designed to be called periodically whether a device is connected or not.
// It will automatically detect a camera as soon as it is connected to USB. Then, the openni
// connection is set up. The function returns true when a camera is connected and successfully
// set up, false otherwise.
bool AsusCameraLoop::connectCamera()
{
    //qDebug() << "CameraLoop::connectCamera() connection state" << cameraConnectionState;

    if (cameraConnectionState > 4)
        return true;

    if (cameraConnectionState == 0)
    {
        // Initialize the openni interface.
        //qDebug() << "OpenNI Version is " << OpenNI::getVersion().major << "." << OpenNI::getVersion().minor << "." << OpenNI::getVersion().maintenance << "." << OpenNI::getVersion().build;
        //qDebug() << "CameraLoop: Initializing OpenNi.";
        status = OpenNI::initialize();
        if (status != STATUS_OK)
        {
            //emit messageOut("CameraLoop: OpenNi initialization failed!");
            qDebug() << "CameraLoop::connectCamera(): OpenNi initialization failed!";
            qDebug() << "ERROR: #" << status << ", " << OpenNI::getExtendedError();
            return false;
        }

        qDebug() << "CameraLoop: Scanning devices.";
        cameraConnectionState++;
    }

    if (cameraConnectionState == 1)
    {
        // Open any device.
        Array<DeviceInfo> deviceList;
        OpenNI::enumerateDevices(&deviceList);
        for (int i = 0; i < deviceList.getSize(); ++i){
            qDebug() << "   " << i << ". Name: " << deviceList[i].getName() << ", URI: " << deviceList[i].getUri() << "\n";
        }
        status = device.open(ANY_DEVICE);
        if (status != STATUS_OK)
        {
//            emit messageOut("CameraLoop: Failed to open device.");
//            qDebug() << "CameraLoop::connectCamera(): Failed to open device.";
//            qDebug() << "ERROR: #" << status << ", " << OpenNI::getExtendedError();
            return false;
        }

        cameraConnectionState++;
    }

    if (cameraConnectionState == 2)
    {
        qDebug() << "CameraLoop: Creating depth data stream.";

        // Create the data stream for the depth sensor.
        if (device.getSensorInfo(SENSOR_DEPTH) == NULL)
        {
//            emit messageOut("CameraLoop: No depth sensor found on device.");
            qDebug() << "CameraLoop: No depth sensor found on device.";
            return false;
        }

        if (depthSensor.isValid())
        {
            depthSensor.stop();
            depthSensor.destroy();
        }

        status = depthSensor.create(device, SENSOR_DEPTH);
        if (status != STATUS_OK)
        {
//            emit messageOut("CameraLoop: Could not create depth data stream.");
            qDebug() << "CameraLoop: Could not create depth data stream.";
            qDebug() << "ERROR: #" << status << ", " << OpenNI::getExtendedError();
            disconnectCamera();
            return false;
        }

        qDebug() << "CameraLoop: Setting video mode for depth camera.";

        VideoMode videoMode;
        videoMode.setFps(CAMERA_FPS);
        videoMode.setResolution(IMAGE_WIDTH, IMAGE_HEIGHT);
        videoMode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
        depthSensor.setMirroringEnabled(false);
        status = depthSensor.setVideoMode(videoMode);
        if (status != STATUS_OK)
        {
//            emit messageOut("CameraLoop: Could not set video mode for depth sensor.");
            qDebug() << "CameraLoop: Could not set video mode for depth sensor.";
            qDebug() << "ERROR: #" << status << ", " << OpenNI::getExtendedError();
            disconnectCamera();
            return false;
        }

        status = depthSensor.start();
        if (status != STATUS_OK)
        {
//            emit messageOut("CameraLoop: Could not start depth sensor stream.");
            qDebug() << "CameraLoop: Could not start depth sensor stream.";
            qDebug() << "ERROR: #" << status << ", " << OpenNI::getExtendedError();
            disconnectCamera();
            return false;
        }

        cameraConnectionState++;
    }

    if (cameraConnectionState == 3)
    {
        qDebug() << "CameraLoop: Creating color data stream.";

        // Create the data stream for the color sensor.
        if (device.getSensorInfo(SENSOR_COLOR) == NULL)
        {
//            emit messageOut("CameraLoop: No color sensor found on device.");
            qDebug() << "CameraLoop: No color sensor found on device.";
            return false;
        }

        if (colorSensor.isValid())
        {
            colorSensor.stop();
            colorSensor.destroy();
        }

        status = colorSensor.create(device, SENSOR_COLOR);
        if (status != STATUS_OK)
        {
//            emit messageOut("CameraLoop: Could not create color data stream.");
            qDebug() << "CameraLoop: Could not create color data stream.";
            qDebug() << "ERROR: #" << status << ", " << OpenNI::getExtendedError();
            disconnectCamera();
            return false;
        }

        qDebug() << "CameraLoop: Setting video mode for color camera.";

        VideoMode videoMode;
        videoMode.setFps(CAMERA_FPS);
        videoMode.setResolution(IMAGE_WIDTH, IMAGE_HEIGHT);
        videoMode.setPixelFormat(PIXEL_FORMAT_RGB888);
        colorSensor.setMirroringEnabled(false);
        status = colorSensor.setVideoMode(videoMode);
        if (status != STATUS_OK)
        {
//            emit messageOut("CameraLoop: Could not set video mode for color sensor.");
            qDebug() << "CameraLoop: Could not set video mode for color sensor.";
            qDebug() << "ERROR: #" << status << ", " << OpenNI::getExtendedError();
            return false;
        }

        status = colorSensor.start();
        if (status != STATUS_OK)
        {
//            emit messageOut("CameraLoop: Could not start color sensor stream.");
            qDebug() << "CameraLoop: Could not start color sensor stream.";
            return false;
        }

        cameraConnectionState++;
    }

    if (cameraConnectionState == 4)
    {
        // Set up device parameters.
        qDebug() << "CameraLoop: Setting up device parameters.";

        // This synchronizes the image recording between rgb and depth.
        status = device.setDepthColorSyncEnabled(true);
        if (status != STATUS_OK)
        {
//            emit messageOut("CameraLoop: Failed to open device.");
            qDebug() << "CameraLoop::connectCamera(): Failed to set depth color sync.";
            qDebug() << "ERROR: #" << status << ", " << OpenNI::getExtendedError();
            return false;
        }

        // Necessary in order to overlap rgb and depth data, but not in general.
        status = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
        if (status != STATUS_OK)
        {
            qDebug() << "CameraLoop::connectCamera(): Failed to set image registration.";
            qDebug() << "ERROR: #" << status << ", " << OpenNI::getExtendedError();
            return false;
        }

        cameraConnectionState++;
    }

    qDebug() << "Camera connect success.";
    emit messageOut("Camera connected.");
    emit cameraDetected();
    return true;
}

// Disconnects the camera.
void AsusCameraLoop::disconnectCamera()
{
    qDebug() << "CameraLoop::disconnectCamera()";

    depthSensor.stop();
    depthSensor.destroy();
    colorSensor.stop();
    colorSensor.destroy();
    device.close();
    OpenNI::shutdown();
    failedFrames = 0;
    cameraConnectionState = 0;
}

// Reads a new frame from the camera (blocking), converts the depth data to 3D points in world coordinates,
// and writes them into the point buffer in state along with a color buffer of the RGB values.
void AsusCameraLoop::processFrame()
{
    int streamReadyIndex;
    VideoStream* pStream = &depthSensor;

    status = OpenNI::waitForAnyStream(&pStream, 1, &streamReadyIndex, CAMERA_SAMPLE_READ_WAIT_TIMEOUT);
    if (status != STATUS_OK)
    {
        qDebug() << "CameraLoop::processFrame(): timeout reached (" << CAMERA_SAMPLE_READ_WAIT_TIMEOUT << " ms): " << OpenNI::getExtendedError();
        failedFrames++;

        if (failedFrames > 3)
        {
            qDebug() << "Camera lost.";
            emit cameraLost();
            emit messageOut("Camera lost.");
            disconnectCamera();
        }

        return;
    }

    status = depthSensor.readFrame(&depthFrame);
    if (!(status == STATUS_OK && depthFrame.isValid()))
    {
        qDebug() << "CameraLoop::processFrame(): reading depth frame failed: " << OpenNI::getExtendedError();
        return;
    }

    status = colorSensor.readFrame(&colorFrame);
    if (!(status == STATUS_OK && colorFrame.isValid()))
    {
        qDebug() << "CameraLoop::processFrame(): reading color frame failed: " << OpenNI::getExtendedError();
        return;
    }

    if (!streamEnabled)
        return;

    int frameHeight = depthFrame.getHeight();
    int frameWidth = depthFrame.getWidth();
    int frameDepthStride = depthFrame.getStrideInBytes();
    int frameColorStride = colorFrame.getStrideInBytes();
    char* frameDepthData = (char*)depthFrame.getData();
    char* frameColorData = (char*)colorFrame.getData();

    // Mutex against the robot control loop.
    QMutexLocker locker(&gMutex);

    uint bufferIndex = 0;
    for (int y = 0; y < frameHeight; ++y)
    {
        DepthPixel* depthPixel = (DepthPixel*)(frameDepthData + (y * frameDepthStride));
        RGB888Pixel* colorPixel = (RGB888Pixel*)(frameColorData + (y * frameColorStride));

        for (int x = 0; x < frameWidth; ++x, ++depthPixel, ++colorPixel, ++bufferIndex)
        {
            if (*depthPixel == 0)
            {
                curState.pointBuffer[bufferIndex].fill(0);

                curState.colorBuffer[bufferIndex].x() = 0;
                curState.colorBuffer[bufferIndex].y() = 0;
                curState.colorBuffer[bufferIndex].z() = 0;

                curState.depthBuffer[bufferIndex] = 0;
            }
            else
            {
                // Transform from depth pixels to local points.
                Vec3 p(curState.unitImage[bufferIndex].x() * (*depthPixel),
                       curState.unitImage[bufferIndex].y() * (*depthPixel),  // x to -y
                       curState.unitImage[bufferIndex].z() * (*depthPixel)); // y to z

                if (config.debugLevel > 0)
                {
                    qDebug() << p;
                }

                // Write the point into the point buffer.
                curState.pointBuffer[bufferIndex] = p;
            }

            // Stream into color buffer.
            curState.colorBuffer[bufferIndex].x() = colorPixel->r;
            curState.colorBuffer[bufferIndex].y() = colorPixel->g;
            curState.colorBuffer[bufferIndex].z() = colorPixel->b;

            // Stream into depth buffer.
            curState.depthBuffer[bufferIndex] = *depthPixel;

            if (config.debugLevel > 0)
            {
                qDebug() << curState.colorBuffer[bufferIndex].x() << "," << curState.colorBuffer[bufferIndex].y() << "," << curState.colorBuffer[bufferIndex].z();
            }
        }
    }
}

#if DEPTHCAM == 2
RealsenseCameraLoop::RealsenseCameraLoop()
{
    running = false;
    streamEnabled = true;
    lastUpdateTimestamp = 0;
    cameraConnectionState = 0;
    failedFrames = 0;

    e = 0;
    pipeline_profile = 0;
    rs2config = 0;
    pipeline = 0;
    dev = 0;
    device_list = 0;
    ctx = 0;
    m_alignQueue = 0;
    m_alignProcessor = 0;
    m_postprocess_temporal_Processor = 0;
}

RealsenseCameraLoop::~RealsenseCameraLoop()
{
    //disconnectCamera();
    stop();
}

// Init function, called first thing after construction.
void RealsenseCameraLoop::init()
{
    qDebug() << "CameraRealsense: INIT.";
}

// The "loop" of the thread. Calls the step() function and measures execution time.
void RealsenseCameraLoop::run()
{
    qDebug() << "CameraRealsense: START.";
    StopWatch stopWatch;

    running = true;
    while (running)
    {
        stopWatch.start();
        state.realTime = stopWatch.time();
        state.cameraLoopIterationTime = state.realTime - lastUpdateTimestamp;
        lastUpdateTimestamp = state.realTime;

        step();

        state.cameraLoopExecutionTime = stopWatch.elapsedTime();
    }

    qDebug() << "CameraRealsense: STOP.";
}

// Step function, called periodically after start() was called.
// It attempts to connect the camera until success, and then calls
// processFrame() until failure.
void RealsenseCameraLoop::step()
{
    if (connectCamera())
        processFrame();
}

// Toggles the data stream between the camera and the state object.
// This will have no influence on the thread, it will keep running
// and reading the camera.
void RealsenseCameraLoop::toggleStream()
{
    streamEnabled = !streamEnabled;
}

void RealsenseCameraLoop::enableStream()
{
    streamEnabled = true;
}

void RealsenseCameraLoop::disableStream()
{
    streamEnabled = false;
}

bool RealsenseCameraLoop::isStreamEnabled() const
{
    return streamEnabled;
}

// Stops the thread execution (blocking!).
void RealsenseCameraLoop::stop()
{
    running = false;
    wait();
}


// Executes the camera connect and setup routine.
// This function is designed to be called periodically whether a device is connected or not.
// It will automatically detect a camera as soon as it is connected to USB. Then, the RS2
// connection is set up. The function returns true when a camera is connected and
// successfully set up, false otherwise. Keep calling it until it connects.
bool RealsenseCameraLoop::connectCamera()
{
//    qDebug() << "CameraRealsense::connectCamera()";

    if (cameraConnectionState > 0)
    {
        //qDebug() << "still connected!" << cameraConnectionState;
        return true;
    }

    // Create a context object. This object owns the handles to all connected realsense devices.
    // The returned object should be released with rs2_delete_context(...)
    ctx = rs2_create_context(RS2_API_VERSION, &e);
    rs_print_error(e);


    // Get a list of all the connected devices.
    // The returned object should be released with rs2_delete_device_list(...)
    device_list = rs2_query_devices(ctx, &e);
    rs_print_error(e);

    int dev_count = rs2_get_device_count(device_list, &e);
    rs_print_error(e);

    if (dev_count == 0)
    {
        qDebug() << "Searching for RealSense devices...";
        QThread::sleep(5);
        return false;
    }


    dev = rs2_create_device(device_list, 0, &e);
    rs_print_error(e);
    rs_print_device_info(dev);

/*
    sensor_list = rs2_query_sensors(dev, &e);

    rs2_sensor* sensor = rs2_create_sensor(sensor_list, 0, &e);
    int is_supported = rs2_supports_option((rs2_options*)sensor, RS2_OPTION_EXPOSURE, &e);

    qDebug() << "is_supported "<< is_supported;

*/

    rs2_sensor_list* slist = rs2_query_sensors(dev, &e);
    rs_print_error(e);

    int sensor_list_count = rs2_get_sensors_count(slist, &e);
    rs_print_error(e);

    rs2_sensor* sensor = rs2_create_sensor(slist, 0, &e);
    rs_print_error(e);

    int is_supported = rs2_supports_option((rs2_options*)sensor, RS2_OPTION_LASER_POWER, &e);
    rs_print_error(e);

    //float min,max,step,def;
    //rs2_get_option_range((rs2_options*)sensor, RS2_OPTION_LASER_POWER, &min, &max, &step, &def, &e);


    rs2_set_option((rs2_options*)sensor, RS2_OPTION_LASER_POWER, 360, &e);
    rs2_set_option((rs2_options*)sensor, RS2_OPTION_EMITTER_ENABLED, 1, &e);
    rs2_set_option((rs2_options*)sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0, &e);

    if (e)
    {
        reset();
        return false;
    }


    // Create a pipeline to configure, start and stop camera streaming
    // The returned object should be released with rs2_delete_pipeline(...)
    pipeline = rs2_create_pipeline(ctx, &e);
    rs_print_error(e);

    // Create a config instance, used to specify hardware configuration
    // The returned object should be released with rs2_delete_config(...)
    rs2config = rs2_create_config(&e);
    rs_print_error(e);

    if (e)
    {
        reset();
        return false;
    }

    const int CAMERA_FPS = 6;


    //qDebug() << "Configuring data streams.";

    // Request a specific configuration
    rs2_config_enable_stream(rs2config, RS2_STREAM_DEPTH, 0, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_Z16, CAMERA_FPS, &e);
    if (e)
    {
        qDebug() << "CameraLoop::connect() : Depth stream could not be configured!";
        rs_print_error(e);
        return false;
    }

    rs2_config_enable_stream(rs2config, RS2_STREAM_COLOR, 0, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_RGB8, CAMERA_FPS, &e);
    if (e)
    {
        qDebug() << "CameraLoop::connect() : Color stream could not be configured!";
        rs_print_error(e);
        return false;
    }


    //
    // Depth & Color alignment
    //
    // init alignment process
    m_alignQueue = rs2_create_frame_queue(1, &e);
    m_alignProcessor = rs2_create_align(RS2_STREAM_COLOR, &e);


    //
    // Post-Processing
    //
    // Establishing a frame_queue object for each processing block that will receive the processed frames
    // Source: https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md#temporal-filter
    m_postprocess_temporal_Queue = rs2_create_frame_queue(1, &e);
    rs_print_error(e);
    // Creating processing blocks/ filters
    m_postprocess_temporal_Processor = rs2_create_temporal_filter_block(&e);
    rs_print_error(e);
    // Control filter options
    rs2_set_option((rs2_options*)m_postprocess_temporal_Processor, RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.1, NULL); // default 0.4, range [0-1]
    rs_print_error(e);
    rs2_set_option((rs2_options*)m_postprocess_temporal_Processor, RS2_OPTION_FILTER_SMOOTH_DELTA, 20, NULL); // default 20, discrete range [1-100]
    rs_print_error(e);

    m_postprocess_spatial_Queue = rs2_create_frame_queue(1, &e);
    rs_print_error(e);
    // Creating processing blocks/ filters
    m_postprocess_spatial_Processor = rs2_create_spatial_filter_block(&e);
    rs_print_error(e);
    // Control filter options
    rs2_set_option((rs2_options*)m_postprocess_spatial_Processor, RS2_OPTION_FILTER_MAGNITUDE, 0, NULL); // default 2, range [1-5]
    rs_print_error(e);
    rs2_set_option((rs2_options*)m_postprocess_spatial_Processor, RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.9, NULL); // default 0.5, range [0.25-1]
    rs_print_error(e);
    rs2_set_option((rs2_options*)m_postprocess_spatial_Processor, RS2_OPTION_FILTER_SMOOTH_DELTA, 10, NULL); // default 20, discrete range [1-50]
    rs_print_error(e);
    rs2_set_option((rs2_options*)m_postprocess_spatial_Processor, RS2_OPTION_HOLES_FILL, 0, NULL); // default 0, [0-5] range mapped to [none,2,4,8,16,unlimited] pixels.
    rs_print_error(e);


    qDebug() << "Starting data streams.";

    // Start the pipeline streaming
    // The returned object should be released with rs2_delete_pipeline_profile(...)
    pipeline_profile = rs2_pipeline_start_with_config(pipeline, rs2config, &e);
    if (e)
    {
        qDebug() << "CameraLoop::connect() : Pipeline streaming could not be started!";
        rs_print_error(e);
        reset();
        return false;
    }

    rs2_start_processing_queue(m_alignProcessor, m_alignQueue, &e);
    if (e)
    {
        qDebug() << "CameraLoop::connect() : Align processor processing queue could not be started!";
        rs_print_error(e);
        reset();
        return false;
    }


    // Direct the output of the filters to a dedicated queue
    rs2_start_processing_queue(m_postprocess_temporal_Processor, m_postprocess_temporal_Queue, &e);
    if (e)
    {
        qDebug() << "CameraLoop::connect() : Temporal postprocess processing queue could not be started!";
        rs_print_error(e);
        reset();
        return false;
    }

    rs2_start_processing_queue(m_postprocess_spatial_Processor, m_postprocess_spatial_Queue, &e);
    if (e)
    {
        qDebug() << "CameraLoop::connect() : Spatial postprocess processing queue could not be started!";
        rs_print_error(e);
        reset();
        return false;
    }

    cameraConnectionState = 1;

    qDebug() << "Camera connect success.";
    emit messageOut("Camera connected.");
    emit cameraDetected();

    return true;
}

// Resets the camera loop to a fresh start by destroying all involved
// data structures and resetting internal states so that the connectCamera()
// function can be called again.
void RealsenseCameraLoop::reset()
{
    //qDebug() << "CameraRealsense: disconnectCamera()";

    // Stop the pipeline streaming
    if (pipeline)
    {
        rs2_pipeline_stop(pipeline, &e);
        rs_print_error(e);
        rs2_delete_pipeline(pipeline);
        pipeline = nullptr;
    }

    // Release resources

    if (pipeline_profile)
    {
        rs2_delete_pipeline_profile(pipeline_profile);
        pipeline_profile = nullptr;
    }

    if (rs2config)
    {
        rs2_delete_config(rs2config);
        rs2config = nullptr;
    }


    if (dev)
    {
        rs2_delete_device(dev);
        dev = nullptr;
    }

    if (device_list)
    {
        rs2_delete_device_list(device_list);
        device_list = nullptr;
    }

/*
    if (sensor_list)
    {
        rs2_delete_sensor_list(sensor_list);
        sensor_list = nullptr;
    }
*/

    if (ctx)
    {
        rs2_delete_context(ctx);
        ctx = nullptr;
    }

    if (m_alignQueue)
    {
        rs2_delete_frame_queue(m_alignQueue);
        m_alignQueue = nullptr;
    }

    if (m_alignProcessor)
    {
        rs2_delete_processing_block(m_alignProcessor);
        m_alignProcessor = nullptr;
    }


    if (m_postprocess_temporal_Processor)
    {
        rs2_delete_processing_block(m_postprocess_temporal_Processor);
        m_postprocess_temporal_Processor = nullptr;
    }
    if (m_postprocess_temporal_Queue)
    {
        rs2_delete_frame_queue(m_postprocess_temporal_Queue);
        m_postprocess_temporal_Queue = nullptr;
    }

    if (m_postprocess_spatial_Processor)
    {
        rs2_delete_processing_block(m_postprocess_spatial_Processor);
        m_postprocess_spatial_Processor = nullptr;
    }
    if (m_postprocess_spatial_Queue)
    {
        rs2_delete_frame_queue(m_postprocess_spatial_Queue);
        m_postprocess_spatial_Queue = nullptr;
    }

    cameraConnectionState = 0;
}

// Reads a new frame from the camera (blocking), applies post processing filters
// converts the depth data to 3D points in local coordinates and writes them
// into the point buffer in state along with the RGB values into the color buffer.
void RealsenseCameraLoop::processFrame()
{
    //qDebug() << "CameraRealsense: processFrame()";

    // This call waits until a new composite_frame is available
    // composite_frame holds a set of frames. It is used to prevent frame drops
    // The retunred object should be released with rs2_release_frame(...)
    rs2_frame* frames;


    // Blocking call to fetch a frame.
    frames = rs2_pipeline_wait_for_frames(pipeline, 2000, &e);

    // Returns the number of frames embedded within the composite frame
    int num_of_frames = rs2_embedded_frames_count(frames, &e);
    rs_print_error(e);
    //qDebug() << "CameraRealsense: num_of_frames: " << num_of_frames;


    rs2_frame *depth_final_frame = nullptr;

    // align depth and color data
    rs2_frame_add_ref(frames, &e);
    rs_print_error(e);

    rs2_process_frame(m_alignProcessor, frames, &e);
    rs_print_error(e);
    if (e)
        qDebug() << "rs2_process_frame failed!";


    rs2_frame *frames_aligned = rs2_wait_for_frame(m_alignQueue, 100, &e);
    rs_print_error(e);
    if (e)
        qDebug() << "rs2_wait_for_frame frames_aligned failed!";


    // Get depth and color frames
    rs2_frame *frame_depth = rs2_extract_frame(frames_aligned, 0, &e);
    rs_print_error(e);
    rs2_frame *frame_color = rs2_extract_frame(frames_aligned, 1, &e);
    rs_print_error(e);



    //
    // Apply post-process filter
    //
    if (false)
    {
        // spatial filter
        rs2_process_frame(m_postprocess_spatial_Processor, frame_depth, NULL);
        rs2_frame* depth_spatial_frame = rs2_wait_for_frame(m_postprocess_spatial_Queue, 5000, NULL);

        // temporal filter
        rs2_process_frame(m_postprocess_temporal_Processor, depth_spatial_frame, NULL);
        rs2_frame* depth_temporal_frame = rs2_wait_for_frame(m_postprocess_temporal_Queue, 5000, NULL);


        depth_final_frame = depth_temporal_frame;
        //delete[] depth_filtered_frame;
        depth_temporal_frame = nullptr;
        depth_spatial_frame = nullptr;

    }
    else
    {
        depth_final_frame = frame_depth;
    }


    // get camera intrinsics
    const rs2_stream_profile *profile = rs2_get_frame_stream_profile(depth_final_frame, &e);
    rs_print_error(e);
    rs2_intrinsics camera_intrin;
    rs2_get_video_stream_intrinsics(profile, &camera_intrin, &e);
    rs_print_error(e);

    // Retrieve depth data, configured as 16-bit depth values
    const uint16_t *depth_frame_data = (const uint16_t *)(rs2_get_frame_data(depth_final_frame, &e));
    rs_print_error(e);
    const uint8_t *rgb_frame_data = (const uint8_t *)(rs2_get_frame_data(frame_color, &e));
    rs_print_error(e);

    // Mutex against rc step, camera step, and draw.
    QMutexLocker locker(&state.gMutex);

    // Loop over image
    if (streamEnabled)
    {
        uint bufferIndex = 0;
        for (int y = 0; y < IMAGE_HEIGHT; ++y)
        {
            for (int x = 0; x < IMAGE_WIDTH; ++x)
            {
                uint16_t depth = *depth_frame_data++;
                uint8_t r = *rgb_frame_data++;
                uint8_t g = *rgb_frame_data++;
                uint8_t b = *rgb_frame_data++;

                // Transform the pixel coordinates to local points using the depth data.
                float point[3];
                float pixel[2] = {float(x), float(y)};
                rs2_deproject_pixel_to_point(point, &camera_intrin, pixel, depth);

                Vec3 p(point[2] *  0.001,
                       point[0] * -0.001,  // 90 degree clockwise (x,y) -> (y,-x)
                       point[1] * -0.001);

                // Copy the point into the point buffer in state.
                state.pointBuffer[bufferIndex] = p;

                // Copy into the organized point cloud
                state.opc.setPx(x, y, p.x);
                state.opc.setPy(x, y, p.y);
                state.opc.setPz(x, y, p.z);
                state.opc.setDepth(x, y, depth);

                // Copy the rgb values into the color buffer.
                state.colorBuffer[bufferIndex].r = r;
                state.colorBuffer[bufferIndex].g = g;
                state.colorBuffer[bufferIndex].b = b;

                // copy into the depth buffer
                state.depthBuffer[bufferIndex] = depth;

                // increment buffer index
                ++bufferIndex;
            }
        }
    }

    rs2_release_frame(frame_depth);
    rs2_release_frame(depth_final_frame);

    rs2_release_frame(frame_color);
    rs2_release_frame(frames_aligned);
    rs2_release_frame(frames);

    //qDebug() << "CameraRealsense: processFrame() finished";
}

void RealsenseCameraLoop::rs_print_error(rs2_error *e)
{
    if (e)
    {
        qDebug() << "rs_error was raised when calling "
        << rs2_get_failed_function(e) << "(" << rs2_get_failed_args(e) << "):";
        qDebug() << "    " << rs2_get_error_message(e);
    }
}

void RealsenseCameraLoop::rs_print_device_info(rs2_device *dev)
{
    rs2_error *e = 0;

    qDebug() << "Using device 0, an " << rs2_get_device_info(dev, RS2_CAMERA_INFO_NAME, &e);
    rs_print_error(e);

    qDebug() << "    Serial number: " << rs2_get_device_info(dev, RS2_CAMERA_INFO_SERIAL_NUMBER, &e);
    rs_print_error(e);

    qDebug() << "    Firmware version: " << rs2_get_device_info(dev, RS2_CAMERA_INFO_FIRMWARE_VERSION, &e);
    rs_print_error(e);
}
#endif
