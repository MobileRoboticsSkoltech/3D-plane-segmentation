#include "gui/MessageQueue.h"

// Provides a simple, most recent on top message queue.
// Use the messageIn(QString) slot to add a message to the queue.
// The messages are faded out and deleted after a short time.
// To implement message animation, the message queue periodically
// fires an updated() signal as long as it has something to say and
// stops signaling when the queue is empty.

MessageQueue::MessageQueue()
{
	totalMessageTime = 1.8; // messages appear for how long in seconds?
	updateInterval = 0.05; // seconds to update the message queue and send the signal

	timer.setInterval((int)(updateInterval*1000));
	connect(&timer, SIGNAL(timeout()), this, SLOT(updateMessages()));
//	timer.start();
}

// Prepends the new message to the message queue.
void MessageQueue::messageIn(QString m)
{
	// Restart the timer if there are no messages and the queue was resting.
	if (messages.isEmpty())
		timer.start();

	messages.prepend(m);
	timeStamps.prepend(totalMessageTime);
	fadeFactors.prepend(1.0);
}

// Updates the message queue by subtracting the update interval from the timestamps,
// recalculating the fade factors and erasing messages that have timed out.
void MessageQueue::updateMessages()
{
	int i = 0;
	while (i < timeStamps.size())
	{
		timeStamps[i] -= updateInterval;

		if (timeStamps[i] <= 0)
		{
			timeStamps.erase(timeStamps.begin()+i, timeStamps.end());
			messages.erase(messages.begin()+i, messages.end());
			fadeFactors.erase(fadeFactors.begin()+i, fadeFactors.end());

			if (timeStamps.isEmpty())
				timer.stop();

			break;
		}

		fadeFactors[i] = qBound(0.0, 20.0*timeStamps[i]/totalMessageTime, 1.0);

		i++;
	}

	emit updated();
}

// Draws the messages on a QPainter canvas.
void MessageQueue::draw(QPainter* p, int r, int g, int b)
{
	p->save();
	p->setFont(QFont("Helvetica", 18));

	for (int i = 0; i < messages.size(); i++)
	{
		p->setPen(QColor(r, g, b, fadeFactors[i]*255));
		p->drawText(QPoint(0, i*28), messages[i]);
	}

	p->restore();
}
