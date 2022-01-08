#ifndef MESSAGEQUEUE_H_
#define MESSAGEQUEUE_H_

#include <GL/glew.h>
#include <QtGui>

class MessageQueue : public QObject
{
	Q_OBJECT

private:
	QTimer timer;
	double totalMessageTime;
	double updateInterval;
	QList<double> timeStamps;

private slots:
	void updateMessages();

public:

	QList<QString> messages;
	QList<double> fadeFactors;

	MessageQueue();
	~MessageQueue(){};

	void draw(QPainter* p, int r=85, int g=85, int b=0);

public slots:
	void messageIn(QString m);

signals:
	void updated();
};

#endif // MESSAGEQUEUE_H_
