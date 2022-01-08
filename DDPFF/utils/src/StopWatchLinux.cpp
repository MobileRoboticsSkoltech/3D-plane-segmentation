#include "utils/StopWatchLinux.h"

SubStopWatch::SubStopWatch()
{
    clock_gettime(CLOCK_MONOTONIC, &programStartTime);
    clock_gettime(CLOCK_MONOTONIC, &lastStartTime);
}

// Resets the elapsed time to 0.
void SubStopWatch::start()
{
    clock_gettime(CLOCK_MONOTONIC, &lastStartTime);
}

// Returns a time stamp expressed in seconds since the last restart.
double SubStopWatch::elapsedTime()
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    long diffInSecs = now.tv_sec - lastStartTime.tv_sec;
    long diffInNanos = now.tv_nsec - lastStartTime.tv_nsec;
    return (double)diffInSecs + (double)diffInNanos/1000000000;
}

// Returns a time stamp expressed in seconds since program start.
double SubStopWatch::programTime()
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    long diffInSecs = now.tv_sec - programStartTime.tv_sec;
    long diffInNanos = now.tv_nsec - programStartTime.tv_nsec;
    return (double)diffInSecs + (double)diffInNanos/1000000000;
}

// Returns a system timestamp expressed in seconds since I don't know when.
double SubStopWatch::systemTime()
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    long diffInSecs = now.tv_sec;
    long diffInNanos = now.tv_nsec;
    return (double)diffInSecs + (double)diffInNanos/1000000000;
}
