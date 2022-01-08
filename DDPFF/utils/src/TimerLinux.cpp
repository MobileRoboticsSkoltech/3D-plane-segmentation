#include "utils/TimerLinux.h"
#include "utils/StopWatch.h"
#include <QDebug>
#include <unistd.h>

SubTimer::SubTimer(QObject *parent) : QThread (parent)
{
    running = false;
}

SubTimer::~SubTimer()
{
	stop();
    wait();
}

void SubTimer::startTimer(int milliSeconds)
{
    running = true;

    period.tv_sec = milliSeconds/1000;
    period.tv_nsec = (milliSeconds % 1000) * 1000000;
    //qDebug() << "starting timer with" << milliSeconds << "ms" << period.tv_sec << period.tv_nsec << QThread::currentThreadId();

    start(QThread::TimeCriticalPriority);
}

void SubTimer::stop()
{
    running = false;
}

bool SubTimer::isActive()
{
    return running;
}

void SubTimer::run()
{
    while(running)
    {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &period, NULL);
        emit timeOut();
    }
}
