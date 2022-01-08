#include "utils/Timer.h"

Timer::Timer(QObject *parent) : QObject (parent)
{
    connect(&_timer, SIGNAL(timeOut()), this, SIGNAL(timeout()), Qt::DirectConnection);
}

Timer::~Timer()
{
    stop();
}

void Timer::start(int milliSeconds)
{
    _timer.startTimer(milliSeconds);
}

void Timer::stop()
{
    _timer.stop();
}

bool Timer::isActive()
{
    return _timer.isActive();
}


