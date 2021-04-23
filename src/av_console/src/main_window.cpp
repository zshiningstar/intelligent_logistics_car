
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/av_console/main_window.hpp"
#include <QFileDialog>
#include "unistd.h"
#include "cstdio"
#include "QDir"

namespace av_console {
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent):
  QMainWindow(parent),
  qnode(argc,argv),
  m_nodeInited(false)
{
  ui.setupUi(this);

  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
  ReadSettings();
  setWindowIcon(QIcon(":/images/car.jpg"));
  ui.tabWidget->setCurrentIndex(0);

  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
  QObject::connect(&qnode, SIGNAL(taskStateChanged(int)), this, SLOT(onTaskStateChanged(int)));
  QObject::connect(&qnode, SIGNAL(rosmasterOffline()), this, SLOT(onRosmasterOffline()));
  QObject::connect(&mTimer, SIGNAL(timeout()), this, SLOT(onTimeout()));

  this->initSensorStatusWidget();

    }
}

MainWindow::~MainWindow() {}

void av_console::MainWindow::on_pushButton_driverlessStart_clicked(bool checked)
{
    if(checked)
    {
        //确保qnode已经初始化
        if(!qnode.initialed())
        {
            onTaskStateChanged(qnode.Idle);
            QMessageBox msgBox(this);
            msgBox.setIcon(QMessageBox::Warning);
            msgBox.setText("Please Connect Firstly.");
            msgBox.exec();
            return;
        }

        if(!qnode.serverConnected())
        {
            this->showMessgeInStatusBar("driverless server is not connected! restarting driverless program...", true);
            qnode.stampedLog(qnode.Info, "driverless server is not connected! restarting driverless program...");
            launchRosNodes("path_track");
            onTaskStateChanged(qnode.Idle);
            return;
        }

        bool ok;
        float speed = ui.comboBox_driverSpeed->currentText().toFloat(&ok);
        QString roadnet_file = ui.lineEdit_roadNet->text();
        if(roadnet_file.isEmpty())
        {
            QMessageBox msgBox(this);
            msgBox.setText("No Roadnet File.");
            msgBox.exec();
            onTaskStateChanged(qnode.Idle);
            return;
        }

        //询问是否保存日志文件
        QFileInfo roadnetFileInfo(roadnet_file);
        QDir roadnetDir(roadnetFileInfo.absolutePath());//文件所在目录

        QString question = tr("Save log file in ") + roadnetFileInfo.absolutePath() + tr(" ?");
        QMessageBox msgBox(QMessageBox::Question, tr("Start driverless"), question,
                           QMessageBox::YesAll|QMessageBox::Yes|QMessageBox::Cancel, this);

        msgBox.button(QMessageBox::YesAll)->setText(tr("Run and save"));
        msgBox.button(QMessageBox::Yes)->setText(tr("Run without save"));
        msgBox.button(QMessageBox::Cancel)->setText(tr("Cancel"));
        msgBox.setDefaultButton(QMessageBox::Yes);
        int button = msgBox.exec();
        //若点击了叉号，则放弃操作 Cancel
        //std::cout  << std::hex << button << std::endl;
        if(button == QMessageBox::Cancel)
        {
            onTaskStateChanged(qnode.Idle);
            return;
        }
        else if(button == QMessageBox::YesAll) // saveLog
        {
        }

        driverless::DoDriverlessTaskGoal goal;
        goal.roadnet_file = roadnet_file.toStdString();
        goal.expect_speed = speed;

        //目标类型
        if(this->ui.comboBox_goalType->currentText() == "Path")  //路径文件
            goal.type = goal.FILE_TYPE;
        else if(this->ui.comboBox_goalType->currentText() == "Goal") //目标点
        {
            goal.type = goal.POSE_TYPE;
            PoseArray path;
            if(!loadPathPoints(roadnet_file.toStdString(), path) || path.poses.size() < 1)
            {
                QMessageBox msgBox;
                msgBox.setText(QString("Load goal pose from %1 failed!").arg(roadnet_file));
                msgBox.exec();
                onTaskStateChanged(qnode.Idle);
                return;
            }
            const Pose goal_pose = path.poses[0];
            goal.target_pose.x = goal_pose.x;
            goal.target_pose.y = goal_pose.y;
            goal.target_pose.theta = goal_pose.yaw;
        }
        else
        {
            QMessageBox::warning(this,"Unknown Road Type!", "Unknown Road Type!");
            onTaskStateChanged(qnode.Idle);
            return;
        }
        //任务类型
        if(ui.comboBox_taskType->currentText() == "Drive") //前进
            goal.task  = goal.DRIVE_TASK;
        else if(ui.comboBox_taskType->currentText() == "Reverse")
        {
            goal.task  = goal.REVERSE_TASK;
            if(this->ui.checkBox_pathFilp->checkState() == Qt::Checked)
                goal.path_filp = true;
            else
                goal.path_filp = false;
        }

        qnode.requestDriverlessTask(goal);
        onTaskStateChanged(qnode.Driverless_Starting);
    }
    else
    {
        qnode.cancleAllGoals();
        onTaskStateChanged(qnode.Idle);
    }
}
