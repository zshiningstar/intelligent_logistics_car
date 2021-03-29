/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *master;
    QLabel *describe;
    QPushButton *gps;
    QPushButton *record;
    QPushButton *track;
    QPushButton *livox;
    QPushButton *leishen;
    QPushButton *cluster;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(565, 426);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        master = new QPushButton(centralWidget);
        master->setObjectName(QStringLiteral("master"));
        master->setEnabled(true);
        master->setGeometry(QRect(70, 50, 161, 61));
        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(20);
        sizePolicy.setVerticalStretch(20);
        sizePolicy.setHeightForWidth(master->sizePolicy().hasHeightForWidth());
        master->setSizePolicy(sizePolicy);
        describe = new QLabel(centralWidget);
        describe->setObjectName(QStringLiteral("describe"));
        describe->setGeometry(QRect(30, 0, 141, 18));
        gps = new QPushButton(centralWidget);
        gps->setObjectName(QStringLiteral("gps"));
        gps->setEnabled(true);
        gps->setGeometry(QRect(320, 50, 161, 61));
        sizePolicy.setHeightForWidth(gps->sizePolicy().hasHeightForWidth());
        gps->setSizePolicy(sizePolicy);
        record = new QPushButton(centralWidget);
        record->setObjectName(QStringLiteral("record"));
        record->setEnabled(true);
        record->setGeometry(QRect(70, 150, 161, 61));
        sizePolicy.setHeightForWidth(record->sizePolicy().hasHeightForWidth());
        record->setSizePolicy(sizePolicy);
        track = new QPushButton(centralWidget);
        track->setObjectName(QStringLiteral("track"));
        track->setEnabled(true);
        track->setGeometry(QRect(320, 150, 161, 61));
        sizePolicy.setHeightForWidth(track->sizePolicy().hasHeightForWidth());
        track->setSizePolicy(sizePolicy);
        livox = new QPushButton(centralWidget);
        livox->setObjectName(QStringLiteral("livox"));
        livox->setEnabled(true);
        livox->setGeometry(QRect(70, 250, 71, 61));
        sizePolicy.setHeightForWidth(livox->sizePolicy().hasHeightForWidth());
        livox->setSizePolicy(sizePolicy);
        leishen = new QPushButton(centralWidget);
        leishen->setObjectName(QStringLiteral("leishen"));
        leishen->setEnabled(true);
        leishen->setGeometry(QRect(320, 250, 161, 61));
        sizePolicy.setHeightForWidth(leishen->sizePolicy().hasHeightForWidth());
        leishen->setSizePolicy(sizePolicy);
        cluster = new QPushButton(centralWidget);
        cluster->setObjectName(QStringLiteral("cluster"));
        cluster->setEnabled(true);
        cluster->setGeometry(QRect(160, 250, 71, 61));
        sizePolicy.setHeightForWidth(cluster->sizePolicy().hasHeightForWidth());
        cluster->setSizePolicy(sizePolicy);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 565, 27));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        master->setText(QApplication::translate("MainWindow", "plot", 0));
        describe->setText(QApplication::translate("MainWindow", "SEU-smartcar", 0));
        gps->setText(QApplication::translate("MainWindow", "gps", 0));
        record->setText(QApplication::translate("MainWindow", "record", 0));
        track->setText(QApplication::translate("MainWindow", "track", 0));
        livox->setText(QApplication::translate("MainWindow", "livox", 0));
        leishen->setText(QApplication::translate("MainWindow", "leishen", 0));
        cluster->setText(QApplication::translate("MainWindow", "cluster", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
