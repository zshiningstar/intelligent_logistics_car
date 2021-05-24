
#include <QtGui>
#include <QApplication>
#include "../include/main_window.hpp"

int main(int argc, char **argv)
{

    QApplication app(argc, argv);
    av_console::MainWindow w(argc,argv);
    w.showFullScreen();
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();


    //app->exit(777);
    if(result == 777) //restart
        QProcess::startDetached(qApp->applicationFilePath(), QStringList());

	return result;
}
