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
    QPushButton *gps_topic;
    QPushButton *gps_odom;
    QPushButton *topic_list;
    QPushButton *record_room;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(640, 643);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        plot = new QPushButton(centralWidget);
        plot->setObjectName(QString::fromUtf8("plot"));
        plot->setEnabled(true);
        plot->setGeometry(QRect(30, 500, 561, 61));
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
        gps->setGeometry(QRect(30, 60, 161, 61));
        sizePolicy.setHeightForWidth(gps->sizePolicy().hasHeightForWidth());
        gps->setSizePolicy(sizePolicy);
        record = new QPushButton(centralWidget);
        record->setObjectName(QString::fromUtf8("record"));
        record->setEnabled(true);
        record->setGeometry(QRect(30, 280, 161, 61));
        sizePolicy.setHeightForWidth(record->sizePolicy().hasHeightForWidth());
        record->setSizePolicy(sizePolicy);
        track = new QPushButton(centralWidget);
        track->setObjectName(QString::fromUtf8("track"));
        track->setEnabled(true);
        track->setGeometry(QRect(230, 280, 161, 61));
        sizePolicy.setHeightForWidth(track->sizePolicy().hasHeightForWidth());
        track->setSizePolicy(sizePolicy);
        livox = new QPushButton(centralWidget);
        livox->setObjectName(QString::fromUtf8("livox"));
        livox->setEnabled(true);
        livox->setGeometry(QRect(30, 390, 161, 61));
        sizePolicy.setHeightForWidth(livox->sizePolicy().hasHeightForWidth());
        livox->setSizePolicy(sizePolicy);
        leishen = new QPushButton(centralWidget);
        leishen->setObjectName(QString::fromUtf8("leishen"));
        leishen->setEnabled(true);
        leishen->setGeometry(QRect(430, 390, 161, 61));
        sizePolicy.setHeightForWidth(leishen->sizePolicy().hasHeightForWidth());
        leishen->setSizePolicy(sizePolicy);
        cluster = new QPushButton(centralWidget);
        cluster->setObjectName(QString::fromUtf8("cluster"));
        cluster->setEnabled(true);
        cluster->setGeometry(QRect(230, 390, 161, 61));
        sizePolicy.setHeightForWidth(cluster->sizePolicy().hasHeightForWidth());
        cluster->setSizePolicy(sizePolicy);
        back = new QPushButton(centralWidget);
        back->setObjectName(QString::fromUtf8("back"));
        back->setEnabled(true);
        back->setGeometry(QRect(430, 280, 161, 61));
        sizePolicy.setHeightForWidth(back->sizePolicy().hasHeightForWidth());
        back->setSizePolicy(sizePolicy);
        serial = new QPushButton(centralWidget);
        serial->setObjectName(QString::fromUtf8("serial"));
        serial->setEnabled(true);
        serial->setGeometry(QRect(430, 170, 161, 61));
        sizePolicy.setHeightForWidth(serial->sizePolicy().hasHeightForWidth());
        serial->setSizePolicy(sizePolicy);
        gps_topic = new QPushButton(centralWidget);
        gps_topic->setObjectName(QString::fromUtf8("gps_topic"));
        gps_topic->setEnabled(true);
        gps_topic->setGeometry(QRect(230, 60, 161, 61));
        sizePolicy.setHeightForWidth(gps_topic->sizePolicy().hasHeightForWidth());
        gps_topic->setSizePolicy(sizePolicy);
        gps_odom = new QPushButton(centralWidget);
        gps_odom->setObjectName(QString::fromUtf8("gps_odom"));
        gps_odom->setEnabled(true);
        gps_odom->setGeometry(QRect(430, 60, 161, 61));
        sizePolicy.setHeightForWidth(gps_odom->sizePolicy().hasHeightForWidth());
        gps_odom->setSizePolicy(sizePolicy);
        topic_list = new QPushButton(centralWidget);
        topic_list->setObjectName(QString::fromUtf8("topic_list"));
        topic_list->setEnabled(true);
        topic_list->setGeometry(QRect(230, 170, 161, 61));
        sizePolicy.setHeightForWidth(topic_list->sizePolicy().hasHeightForWidth());
        topic_list->setSizePolicy(sizePolicy);
        record_room = new QPushButton(centralWidget);
        record_room->setObjectName(QString::fromUtf8("record_room"));
        record_room->setEnabled(true);
        record_room->setGeometry(QRect(30, 170, 161, 61));
        sizePolicy.setHeightForWidth(record_room->sizePolicy().hasHeightForWidth());
        record_room->setSizePolicy(sizePolicy);
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(30, 20, 63, 22));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(30, 130, 121, 22));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(30, 240, 191, 22));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(30, 350, 191, 22));
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(30, 460, 191, 22));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 640, 27));
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
        gps->setText(QApplication::translate("MainWindow", "daoyuan", 0, QApplication::UnicodeUTF8));
        record->setText(QApplication::translate("MainWindow", "record", 0, QApplication::UnicodeUTF8));
        track->setText(QApplication::translate("MainWindow", "forward", 0, QApplication::UnicodeUTF8));
        livox->setText(QApplication::translate("MainWindow", "livox", 0, QApplication::UnicodeUTF8));
        leishen->setText(QApplication::translate("MainWindow", "leishen", 0, QApplication::UnicodeUTF8));
        cluster->setText(QApplication::translate("MainWindow", "cluster", 0, QApplication::UnicodeUTF8));
        back->setText(QApplication::translate("MainWindow", "backward", 0, QApplication::UnicodeUTF8));
        serial->setText(QApplication::translate("MainWindow", "backward_room", 0, QApplication::UnicodeUTF8));
        gps_topic->setText(QApplication::translate("MainWindow", "/gps", 0, QApplication::UnicodeUTF8));
        gps_odom->setText(QApplication::translate("MainWindow", "/odom", 0, QApplication::UnicodeUTF8));
        topic_list->setText(QApplication::translate("MainWindow", "forward_room", 0, QApplication::UnicodeUTF8));
        record_room->setText(QApplication::translate("MainWindow", "record_room", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "GPS", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "IN ROOM", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "OUTSIDE ROOM", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "SENSOR", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "TOOLS", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
