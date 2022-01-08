#ifndef CONFIGWIDGET_H_
#define CONFIGWIDGET_H_

#include <GL/glew.h>
#include <QtGui>
#include <QTreeWidget>
#include <QLabel>

class ConfigWidget: public QTreeWidget
{
	Q_OBJECT

	QSignalMapper configSliderMapper;
	QList<QSlider*> configSliders;
	QList<QLabel*> configLabels;
	QList<QTreeWidgetItem*> treeItems;
	int selectedSliderIndex;

public:
	ConfigWidget(QWidget *parent = 0);
	~ConfigWidget();

	void init();

public slots:
	void configSliderMoved(int);
	void configChangedIn();
	void selectNextSlider();
	void selectPrevSlider();
	void increaseSelectedSlider();
	void decreaseSelectedSlider();
	void scrollOnExpand(QTreeWidgetItem*);

signals:
	void configChangedOut();

private:
	void saveState(QTreeWidgetItem*);

};

#endif // CONFIGWIDGET_H_
