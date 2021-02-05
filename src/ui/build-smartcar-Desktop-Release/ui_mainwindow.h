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
#include <QtGui/QListWidget>
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
    QListWidget *listWidget_gps;
    QListWidget *listWidget_track;
    QPushButton *master;
    QLabel *describe;
    QPushButton *gps;
    QPushButton *record;
    QPushButton *track;
    QPushButton *livox;
    QPushButton *leishen;
    QLabel *label_gps;
    QLabel *label_track;
    QListWidget *listWidget_record;
    QLabel *label_record;
    QPushButton *cluster;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(642, 686);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        listWidget_gps = new QListWidget(centralWidget);
        listWidget_gps->setObjectName(QString::fromUtf8("listWidget_gps"));
        listWidget_gps->setGeometry(QRect(220, 50, 391, 141));
        listWidget_track = new QListWidget(centralWidget);
        listWidget_track->setObjectName(QString::fromUtf8("listWidget_track"));
        listWidget_track->setGeometry(QRect(220, 440, 391, 151));
        master = new QPushButton(centralWidget);
        master->setObjectName(QString::fromUtf8("master"));
        master->setEnabled(true);
        master->setGeometry(QRect(20, 30, 161, 61));
        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(20);
        sizePolicy.setVerticalStretch(20);
        sizePolicy.setHeightForWidth(master->sizePolicy().hasHeightForWidth());
        master->setSizePolicy(sizePolicy);
        describe = new QLabel(centralWidget);
        describe->setObjectName(QString::fromUtf8("describe"));
        describe->setGeometry(QRect(30, 0, 141, 18));
        gps = new QPushButton(centralWidget);
        gps->setObjectName(QString::fromUtf8("gps"));
        gps->setEnabled(true);
        gps->setGeometry(QRect(20, 130, 161, 61));
        sizePolicy.setHeightForWidth(gps->sizePolicy().hasHeightForWidth());
        gps->setSizePolicy(sizePolicy);
        record = new QPushButton(centralWidget);
        record->setObjectName(QString::fromUtf8("record"));
        record->setEnabled(true);
        record->setGeometry(QRect(20, 230, 161, 61));
        sizePolicy.setHeightForWidth(record->sizePolicy().hasHeightForWidth());
        record->setSizePolicy(sizePolicy);
        track = new QPushButton(centralWidget);
        track->setObjectName(QString::fromUtf8("track"));
        track->setEnabled(true);
        track->setGeometry(QRect(20, 330, 161, 61));
        sizePolicy.setHeightForWidth(track->sizePolicy().hasHeightForWidth());
        track->setSizePolicy(sizePolicy);
        livox = new QPushButton(centralWidget);
        livox->setObjectName(QString::fromUtf8("livox"));
        livox->setEnabled(true);
        livox->setGeometry(QRect(20, 430, 71, 61));
        sizePolicy.setHeightForWidth(livox->sizePolicy().hasHeightForWidth());
        livox->setSizePolicy(sizePolicy);
        leishen = new QPushButton(centralWidget);
        leishen->setObjectName(QString::fromUtf8("leishen"));
        leishen->setEnabled(true);
        leishen->setGeometry(QRect(20, 530, 161, 61));
        sizePolicy.setHeightForWidth(leishen->sizePolicy().hasHeightForWidth());
        leishen->setSizePolicy(sizePolicy);
        label_gps = new QLabel(centralWidget);
        label_gps->setObjectName(QString::fromUtf8("label_gps"));
        label_gps->setGeometry(QRect(220, 30, 391, 18));
        label_track = new QLabel(centralWidget);
        label_track->setObjectName(QString::fromUtf8("label_track"));
        label_track->setGeometry(QRect(220, 420, 54, 18));
        listWidget_record = new QListWidget(centralWidget);
        listWidget_record->setObjectName(QString::fromUtf8("listWidget_record"));
        listWidget_record->setGeometry(QRect(220, 240, 391, 141));
        label_record = new QLabel(centralWidget);
        label_record->setObjectName(QString::fromUtf8("label_record"));
        label_record->setGeometry(QRect(220, 220, 54, 18));
        cluster = new QPushButton(centralWidget);
        cluster->setObjectName(QString::fromUtf8("cluster"));
        cluster->setEnabled(true);
        cluster->setGeometry(QRect(110, 430, 71, 61));
        sizePolicy.setHeightForWidth(cluster->sizePolicy().hasHeightForWidth());
        cluster->setSizePolicy(sizePolicy);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 642, 23));
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
        master->setText(QApplication::translate("MainWindow", "master", 0, QApplication::UnicodeUTF8));
        describe->setText(QApplication::translate("MainWindow", "SEU-smartcar", 0, QApplication::UnicodeUTF8));
        gps->setText(QApplication::translate("MainWindow", "gps", 0, QApplication::UnicodeUTF8));
        record->setText(QApplication::translate("MainWindow", "record", 0, QApplication::UnicodeUTF8));
        track->setText(QApplication::translate("MainWindow", "track", 0, QApplication::UnicodeUTF8));
        livox->setText(QApplication::translate("MainWindow", "livox", 0, QApplication::UnicodeUTF8));
        leishen->setText(QApplication::translate("MainWindow", "leishen", 0, QApplication::UnicodeUTF8));
        label_gps->setText(QApplication::translate("MainWindow", "gps", 0, QApplication::UnicodeUTF8));
        label_track->setText(QApplication::translate("MainWindow", "track", 0, QApplication::UnicodeUTF8));
        label_record->setText(QApplication::translate("MainWindow", "record", 0, QApplication::UnicodeUTF8));
        cluster->setText(QApplication::translate("MainWindow", "cluster", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
