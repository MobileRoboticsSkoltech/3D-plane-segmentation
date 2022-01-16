#ifndef _MAINWINDOW_H
#define _MAINWINDOW_H

#include <QMainWindow>
#include "ui_mainwindow.h"
#include <CGAL/Qt/DemosMainWindow.h>

class QDragEnterEvent;
class QDropEvent;
class Surface;
class QDoubleSpinBox;
class QCloseEvent;

namespace CGAL{
class QGLViewer;
}

class MainWindow : public CGAL::Qt::DemosMainWindow, public Ui::MainWindow
{
  Q_OBJECT
public:
  MainWindow(MainWindow* other_window = 0);
  void dragEnterEvent(QDragEnterEvent *);
  void dropEvent(QDropEvent *event);

public Q_SLOTS:
  void show_only(QString);
  void surface_open(const QString& filename);

private Q_SLOTS:
  void on_action_Open_triggered();
  void on_action_OpenDirectory_triggered();
  void on_action_Quit_triggered();
  void on_action_Clone_triggered();
  
private:
  void closeEvent(QCloseEvent *event);
  Surface* surface;
};


#endif // _MAINWINDOW_H
