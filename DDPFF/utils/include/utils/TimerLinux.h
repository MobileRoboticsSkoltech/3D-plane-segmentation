#ifndef LINUXTIMER_H
#define LINUXTIMER_H

#include <QObject>
#include <QThread>
#include <time.h>

class SubTimer : public QThread
{
    Q_OBJECT

    bool running;
    struct timespec period;

public:
    SubTimer(QObject *parent = 0);
    ~SubTimer();
    bool isActive();

public slots:
    void startTimer(int milliSeconds);
	void stop();

signals:
	void timeOut();

private:
    void run();
};

#endif
