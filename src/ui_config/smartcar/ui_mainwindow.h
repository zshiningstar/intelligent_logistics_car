/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *plot;
    QLabel *describe;
    QPushButton *gps;
    QPushButton *record;
    QPushButton *track;
    QPushButton *livox;
    QPushButton *leishen;
    QPushButton *cluster;
    QPushButton *back;
    QPushButton *serial;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(619, 341);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        plot = new QPushButton(centralWidget);
        plot->setObjectName(QString::fromUtf8("plot"));
        plot->setEnabled(true);
        plot->setGeometry(QRect(430, 40, 161, 61));
        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(20);
        sizePolicy.setVerticalStretch(20);
        sizePolicy.setHeightForWidth(plot->sizePolicy().hasHeightForWidth());
        plot->setSizePolicy(sizePolicy);
        describe = new QLabel(centralWidget);
        describe->setObjectName(QString::fromUtf8("describe"));
        describe->setGeometry(QRect(30, 0, 141, 18));
        gps = new QPushButton(centralWidget);
        gps->setObjectName(QString::fromUtf8("gps"));
        gps->setEnabled(true);
        gps->setGeometry(QRect(30, 40, 161, 61));
        sizePolicy.setHeightForWidth(gps->sizePolicy().hasHeightForWidth());
        gps->setSizePolicy(sizePolicy);
        record = new QPushButton(centralWidget);
        record->setObjectName(QString::fromUtf8("record"));
        record->setEnabled(true);
        record->setGeometry(QRect(230, 40, 161, 61));
        sizePolicy.setHeightForWidth(record->sizePolicy().hasHeightForWidth());
        record->setSizePolicy(sizePolicy);
        track = new QPushButton(centralWidget);
        track->setObjectName(QString::fromUtf8("track"));
        track->setEnabled(true);
        track->setGeometry(QRect(30, 120, 161, 61));
        sizePolicy.setHeightForWidth(track->sizePolicy().hasHeightForWidth());
        track->setSizePolicy(sizePolicy);
        livox = new QPushButton(centralWidget);
        livox->setObjectName(QString::fromUtf8("livox"));
        livox->setEnabled(true);
        livox->setGeometry(QRect(30, 200, 161, 61));
        sizePolicy.setHeightForWidth(livox->sizePolicy().hasHeightForWidth());
        livox->setSizePolicy(sizePolicy);
        leishen = new QPushButton(centralWidget);
        leishen->setObjectName(QString::fromUtf8("leishen"));
        leishen->setEnabled(true);
        leishen->setGeometry(QRect(430, 200, 161, 61));
        sizePolicy.setHeightForWidth(leishen->sizePolicy().hasHeightForWidth());
        leishen->setSizePolicy(sizePolicy);
        cluster = new QPushButton(centralWidget);
        cluster->setObjectName(QString::fromUtf8("cluster"));
        cluster->setEnabled(true);
        cluster->setGeometry(QRect(230, 200, 161, 61));
        sizePolicy.setHeightForWidth(cluster->sizePolicy().hasHeightForWidth());
        cluster->setSizePolicy(sizePolicy);
        back = new QPushButton(centralWidget);
        back->setObjectName(QString::fromUtf8("back"));
        back->setEnabled(true);
        back->setGeometry(QRect(230, 120, 161, 61));
        sizePolicy.setHeightForWidth(back->sizePolicy().hasHeightForWidth());
        back->setSizePolicy(sizePolicy);
        serial = new QPushButton(centralWidget);
        serial->setObjectName(QString::fromUtf8("serial"));
        serial->setEnabled(true);
        serial->setGeometry(QRect(430, 120, 161, 61));
        sizePolicy.setHeightForWidth(serial->sizePolicy().hasHeightForWidth());
        serial->setSizePolicy(sizePolicy);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 619, 27));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        plot->setText(QApplication::translate("MainWindow", "plot", 0, QApplication::UnicodeUTF8));
        describe->setText(QApplication::translate("MainWindow", "SEU-smartcar", 0, QApplication::UnicodeUTF8));
        gps->setText(QApplication::translate("MainWindow", "gps", 0, QApplication::UnicodeUTF8));
        record->setText(QApplication::translate("MainWindow", "record", 0, QApplication::UnicodeUTF8));
        track->setText(QApplication::translate("MainWindow", "track", 0, QApplication::UnicodeUTF8));
        livox->setText(QApplication::translate("MainWindow", "livox", 0, QApplication::UnicodeUTF8));
        leishen->setText(QApplication::translate("MainWindow", "leishen", 0, QApplication::UnicodeUTF8));
        cluster->setText(QApplication::translate("MainWindow", "cluster", 0, QApplication::UnicodeUTF8));
        back->setText(QApplication::translate("MainWindow", "back", 0, QApplication::UnicodeUTF8));
        serial->setText(QApplication::translate("MainWindow", "serial", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
