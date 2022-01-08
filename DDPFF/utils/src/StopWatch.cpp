#include "utils/StopWatch.h"

StopWatch::StopWatch()
{

}

// Resets the elapsed time to 0.
void StopWatch::start()
{
    _stopWatch.start();
}

// Returns a time stamp expressed in seconds since the last start.
double StopWatch::elapsedTime()
{
    return _stopWatch.elapsedTime();
}

// Returns a time stamp expressed in milliseconds since the last start.
double StopWatch::elapsedTimeMs()
{
    return _stopWatch.elapsedTime()*1000;
}

// Returns a time stamp expressed in seconds since program start.
double StopWatch::programTime()
{
    return _stopWatch.programTime();
}

// Returns a system timestamp expressed in seconds since I don't know when.
double StopWatch::systemTime()
{
    return _stopWatch.systemTime();
}

// Returns a time stamp expressed in seconds since program start.
double StopWatch::time()
{
    return _stopWatch.programTime();
}
