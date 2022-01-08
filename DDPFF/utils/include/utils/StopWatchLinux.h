#ifndef STOPWATCHLINUX_H_
#define STOPWATCHLINUX_H_
#include <time.h>

class SubStopWatch
{
    struct timespec lastStartTime;
    struct timespec programStartTime;

public:
    SubStopWatch();
    ~SubStopWatch(){}
    void start();
	double elapsedTime();
	double programTime();
	double systemTime();
};

#endif /* STOPWATCHLINUX_H_ */
