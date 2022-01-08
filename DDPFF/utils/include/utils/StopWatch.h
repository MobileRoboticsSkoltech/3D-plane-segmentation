#ifndef STOPWATCH_H_
#define STOPWATCH_H_

#ifdef _WIN32
#include "StopWatchWindows.h"
#elif __linux__
#include "StopWatchLinux.h"
#endif

class StopWatch
{
    SubStopWatch _stopWatch;

public:
	StopWatch();
    ~StopWatch(){}
    void start();
	double elapsedTime();
    double elapsedTimeMs();
	double programTime();
	double systemTime();
    double stateTime();
	double time();
};

#endif /* STOPWATCH_H_ */
