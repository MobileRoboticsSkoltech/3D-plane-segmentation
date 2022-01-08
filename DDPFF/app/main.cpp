#include "app.h"
#include "globals/constants.h"

#include <QtGui>
#include <QApplication>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <QDebug>

int main(int argc, char *argv[])
{
    // Initialize a global random number generator
    srand((unsigned)time(NULL));

    // Instantiate the QApplication and the main window.
    QApplication a(argc, argv);
    App w;

    // Install the main window as an event filter.
    a.installEventFilter(&w);

    // Apply a stylesheet to the application.
    QFile file("styles.css");
    int ctr = 0;
    while (!file.open(QFile::ReadOnly) && ctr < 3)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        ctr++;
    }

    QString styleSheet = QLatin1String(file.readAll());
    w.setStyleSheet(styleSheet);

    // Show the window and start the main event loop.
    w.show();
    return a.exec();
}
