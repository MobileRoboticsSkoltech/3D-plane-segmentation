#ifndef TIMER_H
#define TIMER_H

#include <QObject>

#ifdef __linux__
#include "TimerLinux.h"
#elif _WIN32
#include "util/TimerWindows.h"
#endif

class Timer : public QObject
{
    Q_OBJECT

    SubTimer _timer;

public:
    Timer(QObject *parent = 0);
    ~Timer();
    bool isActive();

public slots:
	void start(int milliSeconds = 0);
	void stop();

signals:
    void timeout();
};

#endif
