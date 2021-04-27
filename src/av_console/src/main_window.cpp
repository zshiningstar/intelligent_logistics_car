
#include <QtGui>
#include <QMessageBox>
#include <QFileDialog>
#include <iostream>
#include "../include/main_window.hpp"
#include "unistd.h"
#include "iostream"
#include "cstdio"
#include "QDir"

namespace av_console {
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent):
    QMainWindow(parent),
    qnode(argc,argv),
    m_nodeInited(false),
    m_pathRecorder(nullptr),
    m_dataRecorder(nullptr)
{
    ui.setupUi(this);

    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tabWidget->setCurrentIndex(0);

    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(&qnode, SIGNAL(taskStateChanged(int)), this, SLOT(onTaskStateChanged(int)));
    QObject::connect(&qnode, SIGNAL(rosmasterOffline()), this, SLOT(onRosmasterOffline()));
    QObject::connect(&mTimer, SIGNAL(timeout()), this, SLOT(onTimeout()));

    this->initSensorStatusWidget();

    //launchDrivelessNode();
}

MainWindow::~MainWindow()
{
    if(m_dataRecorder != nullptr)
        delete m_dataRecorder;


}

/*初始化传感器状态显示控件*/
void MainWindow::initSensorStatusWidget()
{
    //ui.widget_rtkStatus->setChecked(false);
    //ui.widget_rtkStatus->setButtonStyle(ImageSwitch::ButtonStyle_4);
    //ui.widget_rtkStatus->setClickedDisable();

    ui.widget_camera1Status->setChecked(false);
    ui.widget_camera1Status->setButtonStyle(ImageSwitch::ButtonStyle_4);
    ui.widget_camera1Status->setClickedDisable();

    ui.widget_livoxStatus->setChecked(false);
    ui.widget_livoxStatus->setButtonStyle(ImageSwitch::ButtonStyle_4);
    ui.widget_livoxStatus->setClickedDisable();

    ui.widget_gpsStatus->setChecked(false);
    ui.widget_gpsStatus->setButtonStyle(ImageSwitch::ButtonStyle_4);
    ui.widget_gpsStatus->setClickedDisable();

    ui.widget_lidarStatus->setChecked(false);
    ui.widget_lidarStatus->setButtonStyle(ImageSwitch::ButtonStyle_4);
    ui.widget_lidarStatus->setClickedDisable();

    connect(&qnode,SIGNAL(sensorStatusChanged(int,bool)),this,SLOT(sensorStatusChanged(int,bool)));
}

//start driverless
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
            launchRosNodes("driverless");
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
        //launchRosNodes("kill_driverless");
    }
}

//connect
void MainWindow::on_pushButton_connect_clicked()
{
    if(ui.checkbox_use_environment->isChecked())
    {
        if (!qnode.init())
        {
            showMessgeInStatusBar("roscore is not running. please wait a moment and reconnect.", true);
            qnode.log("roscore is not running. please wait a moment and reconnect.");
            return;
        }
        showMessgeInStatusBar("connect to rosmaster ok.");
        qnode.stampedLog(qnode.Info, "connect to rosmaster ok.");
        qnode.start();
        ui.pushButton_connect->setEnabled(false);

    }
    else
    {
        if(!qnode.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString()))
        {
            showNoMasterMessage();
            return;
        }
        else
        {
            ui.pushButton_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
		}
	}
    m_nodeInited = true;

    //实例化数据记录器
    m_dataRecorder = new RecordData();
    //载入RosNodesArray信息
    m_rosNodesArrayInvalid = loadRosNodesArrayInfo();

}


void MainWindow::on_checkbox_use_environment_stateChanged(int state)
{
	bool enabled;
    if ( state == 0 )
		enabled = true;
    else
		enabled = false;

	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}


void MainWindow::sensorStatusChanged(int sensor_id, bool status)
{
    qDebug() <<"sensorStatusChanged  " <<  sensor_id << "\t " << status;
    if(Sensor_Camera1 == sensor_id)
        ui.widget_camera1Status->setChecked(status);
    else if(Sensor_Lidar == sensor_id)
        ui.widget_lidarStatus->setChecked(status);
    else if(Sensor_Gps == sensor_id)
        ui.widget_gpsStatus->setChecked(status);
    else if(Sensor_Livox == sensor_id)
        ui.widget_livoxStatus->setChecked(status);
}

void MainWindow::showNoMasterMessage()
{
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
}

void MainWindow::showMessgeInStatusBar(const QString& msg, bool warnning)
{
    ui.statusbar->showMessage(msg, 3000);
    if(warnning)
        ui.statusbar->setStyleSheet("color: red");
    else
        ui.statusbar->setStyleSheet("color: black");
}

void MainWindow::updateLoggingView()
{
   ui.view_logging->scrollToBottom();
}

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("About ..."),tr("SEU Automatic Vehicle Console."));
}


void MainWindow::ReadSettings() {
    //目录,文件名,linux保存在~/.config
    QSettings settings("av_console", "av_console");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://127.0.0.1:11311/")).toString();
    QString host_url = settings.value("host_url", QString("127.0.0.1")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);

    bool checked = settings.value("use_environment_variables", true).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
    m_roadNetFileDir = settings.value("roadNetFileDir","").toString();
    ui.lineEdit_roadNet->setText(m_roadNetFileDir);

    int speedIndex = settings.value("speedIndex","0").toInt();
    ui.comboBox_driverSpeed->setCurrentIndex(speedIndex);

    m_recordFileDir = settings.value("recordDataFileDir","").toString();
    ui.lineEdit_recordFileName->setText(m_recordFileDir);
}

void MainWindow::WriteSettings() {
    QSettings settings("av_console", "av_console");

    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry()); //保存各窗口尺寸
    settings.setValue("windowState", saveState()); //保存各窗口位置
    settings.setValue("roadNetFileDir",m_roadNetFileDir);
    settings.setValue("speedIndex",QString::number(ui.comboBox_driverSpeed->currentIndex()));

    settings.setValue("recordDataFileDir",m_recordFileDir);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
    int button = QMessageBox::question(this,"Confirm Exit",
                                       "Are you sure to exit?",
                                       QMessageBox::Yes|QMessageBox::No,
                                       QMessageBox::No);
    if(button == QMessageBox::Yes)
    {
        launchRosNodes("kill_driverless");
        event->accept();
        QMainWindow::closeEvent(event);
    }
    else
        event->ignore();
}

}  // namespace av_console


void av_console::MainWindow::on_pushButton_gps_clicked(bool checked)
{
    if(checked)
    {
      changeToCmdDir();
      system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch daoyuan daoyuan.launch'&");
      ui.lineEdit_gps->setText("launch sucessfully!");
    }
    else
    {
      changeToCmdDir();
      system("gnome-terminal -x rosnode kill daoyuan");
      ui.lineEdit_gps->setText("close sucessfully!");
    }
}

void av_console::MainWindow::on_pushButton_cluster_clicked(bool checked)
{
    if(checked)
    {
      changeToCmdDir();
      system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch euclidean_cluster euclidean_cluster.launch'&");
      ui.lineEdit_cluster->setText("launch sucessfully!");
    }
    else
    {
      ui.lineEdit_cluster->setText("close sucessfully!");
      system("gnome-terminal -x rosnode kill euclidean_cluster");
    }
}
void av_console::MainWindow::on_pushButton_livox_clicked(bool checked)
{
    if(checked)
    {
      changeToCmdDir();
      system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch livox_ros_driver livox_lidar_rviz.launch'&");
      ui.lineEdit_livox->setText("launch sucessfully!");
    }
    else
    {
      changeToCmdDir();
      system("gnome-terminal -x rosnode kill livox_lidar_publisher rviz");
      ui.lineEdit_livox->setText("close sucessfully!");
    }
}

void av_console::MainWindow::on_pushButton_lsRadar_clicked(bool checked)
{
    if(checked)
    {
      changeToCmdDir();
      system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch lslidar_c16_decoder lslidar_c16.launch'&");
      ui.lineEdit_lsRadar->setText("launch sucessfully!");
    }
    else
    {
      changeToCmdDir();
      system("gnome-terminal -e rosnode kill lslidar_c16_driver_node lslidar_c16_decoder_node");
      ui.lineEdit_lsRadar->setText("close sucessfully!");
    }
}

//mode=true  ros 工作空间目录
//mode=false 应用程序所在目录 default
bool av_console::MainWindow::changeToCmdDir(bool mode)
{
  static bool parsed = false;
  static QDir cmdDir;
  if(parsed)
  {
    QDir::setCurrent(cmdDir.absolutePath()); //切换目录
    return true;
  }

  QString cmdPath;
  if(mode)
  {
      char buf[50] ;
      FILE * fp =  popen("rospack find av_console", "r");
      fscanf(fp,"%s",buf);
      pclose(fp);
      if(std::string(buf).find("home") == std::string::npos)
      {
          //qnode.stampedLog(qnode.Error, std::string(buf));
          qnode.stampedLog(qnode.Error,"change to cmd directory failed!");
          return false;
      }
      cmdPath = tr(buf);
  }
  else
  {
      //可执行程序所在目录
      //std::cout << QCoreApplication::applicationDirPath().toStdString() << std::endl;
      cmdPath = QCoreApplication::applicationDirPath();
  }

  cmdDir = QDir::current();//获取当前工作目录
  cmdDir.cd(cmdPath);      //修改目录，仅修改了目录名，未切换
  cmdDir.cd("../cmd");
  QDir::setCurrent(cmdDir.absolutePath()); //切换目录
  qnode.stampedLog(qnode.Info,cmdDir.absolutePath().toStdString());

  parsed = true;
  return true;
}

void av_console::MainWindow::on_pushButton_pathPlanning_clicked(bool checked)
{
    if(checked)
    {
        m_pathRecorder = new RecordPath();
        ui.listView_pathPlanning->setModel(m_pathRecorder->loggingModel());
        connect(m_pathRecorder, SIGNAL(loggingUpdated()), this, SLOT(updatePathPlanningLoggingView()));
        if(!m_pathRecorder->start())
        {
            changeToCmdDir();
            system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch daoyuan daoyuan.launch'&");
            m_pathRecorder->log("INFO","No Location Message Published, Starting GPS Automatically.");
        }
        ui.pushButton_pathPlanning->setText("Stop And Save");
    }
    else
    {
        m_pathRecorder->stop();
        if(m_pathRecorder->pathPointsSize() == 0)
        {
          m_pathRecorder->log("WARN","path points is too few !");
          ui.pushButton_pathPlanning->setText("Start");
          return ;
        }

        if(m_roadNetFileDir.isEmpty())
            m_roadNetFileDir = "./";

        while(true)
        {
            QString fileName = QFileDialog::getSaveFileName(this,
                                        "save path points", m_roadNetFileDir, "TXT(*txt)");
            if(fileName.isEmpty())
            {
                int Abandon =
                QMessageBox::question(this,"question","Abandon this record?",
                                      QMessageBox::Yes | QMessageBox::No,QMessageBox::No);
                if(Abandon == QMessageBox::Yes)
                {
                    delete m_pathRecorder;
                    m_pathRecorder = NULL;
                    break;
                }
                else
                    continue;

            }
            std::string pathInfoFile =
            fileName.toStdString().substr(0,fileName.toStdString().find_last_of(".")) + "_info.xml";
            m_pathRecorder->generatePathInfoFile(pathInfoFile);
            m_pathRecorder->savePathPoints(fileName.toStdString());
            delete m_pathRecorder;
            m_pathRecorder = NULL;
            m_roadNetFileDir = fileName;
            break;
        }
        ui.pushButton_pathPlanning->setText("Start");
    }
}

void av_console::MainWindow::updatePathPlanningLoggingView()
{
  ui.listView_pathPlanning->scrollToBottom();
}

void av_console::MainWindow::on_pushButton_openRoadNet_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                "open roadnet file", m_roadNetFileDir, "TXT(*txt)");
    if(fileName.isEmpty())
        return;
/*
    QStringList list = fileName.split('/');
    QString name = *(list.end()-1);
    qDebug() << fileName << "\t" << name;
    ui.lineEdit_roadNet->setText(name);
*/
    ui.lineEdit_roadNet->setText(fileName);
    m_roadNetFileDir = fileName;
}



void av_console::MainWindow::on_tabWidget_currentChanged(int index)
{
    static int last_index = 0;
    if(!qnode.initialed())
    {
        ui.tabWidget->setCurrentIndex(stackWidgetIndex_driverless);
        showMessgeInStatusBar("please connect to master firstly!", true);
        //qnode.log("please connect to master firstly!");
        return;
    }
    if(index == stackWidgetIndex_recorder)
    {
        bool ok;
        QString text = QInputDialog::getText(this, "Infomation", "This feature is under development",
                                                     QLineEdit::Normal,
                                                     "",&ok);
        if(text == "seucar")
            m_dataRecorder->setDisable(false);
        else
            m_dataRecorder->setDisable(true);
    }

    last_index = index;
}

void av_console::MainWindow::onTaskStateChanged(int state)
{
    if(state == qnode.Idle)
    {
        ui.pushButton_driverlessStart->setChecked(false);
        ui.pushButton_driverlessStart->setText("Start");
    }
    else if(state == qnode.Driverless_Starting)
    {
        ui.pushButton_driverlessStart->setChecked(true);
        ui.pushButton_driverlessStart->setText("Starting");
    }
    else if(state == qnode.Driverless_Running)
    {
        ui.pushButton_driverlessStart->setChecked(true);
        ui.pushButton_driverlessStart->setText("Stop");
    }
    else if(state == qnode.Driverless_Complete)
    {
        ui.pushButton_driverlessStart->setChecked(false);
        ui.pushButton_driverlessStart->setText("Start");
    }
}

void av_console::MainWindow::on_comboBox_taskType_activated(const QString &arg1)
{
    if(arg1 == "Custom") //自定义任务形式
    {
        //if(m_customDialog == nullptr)
         //   m_customDialog = new CustomTaskDialog(this);

    }
}

//ros master shutdown , reEnable the connect button
void av_console::MainWindow::onRosmasterOffline()
{
    ui.pushButton_connect->setEnabled(true);
}

void av_console::MainWindow::onTimeout()
{

}

void av_console::MainWindow::showEvent(QShowEvent* event)
{
   this->setPushButtonStylesheet(QString("font: 14pt \"Sans Serif\";"));
}

/*@brief 配置所有pushButton的stylesheet
 */
void av_console::MainWindow::setPushButtonStylesheet(const QString& style)
{
    const QSize BUTTON_SIZE = QSize(130, 40);
    const QSize BUTTON_SIZE1 = QSize(200, 40);
    ui.pushButton_gps->setFixedSize(BUTTON_SIZE);
    ui.pushButton_livox->setFixedSize(BUTTON_SIZE);
    ui.pushButton_lsRadar->setFixedSize(BUTTON_SIZE);
    ui.pushButton_cluster->setFixedSize(BUTTON_SIZE);
    ui.pushButton_camera->setFixedSize(BUTTON_SIZE);

ui.lineEdit_cluster->setFixedSize(BUTTON_SIZE1);
ui.lineEdit_gps->setFixedSize(BUTTON_SIZE1);
ui.lineEdit_lsRadar->setFixedSize(BUTTON_SIZE1);
ui.lineEdit_livox->setFixedSize(BUTTON_SIZE1);
ui.lineEdit_camera->setFixedSize(BUTTON_SIZE1);

    ui.pushButton_camera->setStyleSheet(style);
    ui.pushButton_connect->setStyleSheet(style);
    ui.pushButton_driverlessStart->setStyleSheet(style);
    ui.pushButton_lsRadar->setStyleSheet(style);
    ui.pushButton_gps->setStyleSheet(style);
    ui.pushButton_livox->setStyleSheet(style);
    ui.pushButton_openRoadNet->setStyleSheet(style);
    ui.pushButton_pathPlanning->setStyleSheet(style);
    ui.pushButton_cluster->setStyleSheet(style);
    ui.pushButton_selectRecordFile->setStyleSheet(style);
    ui.pushButton_startRecordData->setStyleSheet(style);

    //设置等高
    ui.pushButton_quit->setFixedHeight(ui.groupBox_sensorStatus->height());
    ui.pushButton_startRecordData->setCheckable(true);
}

QObjectList av_console::MainWindow::getAllLeafChilds(QObject* object)
{
    QObjectList result;
    std::queue<QObject *> queue;
    queue.push(object);
    while(!queue.empty())
    {
        QObject * node = queue.front(); queue.pop();
        //qDebug() << "pop " << node->objectName();

        QObjectList childs = node->children();
        if(childs.size() == 0) //无子节点，当且节点为叶子
        {
            result.push_back(node);
            continue;
        }

        //将其所有子节点放入队列
        for(QObject* child : childs)
        {
            queue.push(child);
            //qDebug() << "push " << child->objectName();
        }
    }
    return result;
}

void av_console::MainWindow::disableRecordDataConfigure(bool flag)
{
    QObjectList childs = getAllLeafChilds(ui.widget_recorderConfig);
    for(QObject* child : childs)
    {
        QCheckBox* checkBox = qobject_cast<QCheckBox*>(child);
        QPushButton* botton = qobject_cast<QPushButton*>(child);
        QLineEdit *lineEdit = qobject_cast<QLineEdit*>(child);
        if(checkBox)
            checkBox->setDisabled(flag);
        else if(botton)
            botton->setDisabled(flag);
        else if(lineEdit)
            lineEdit->setReadOnly(flag);
    }
}

void av_console::MainWindow::on_pushButton_startRecordData_clicked(bool checked)
{
    if(checked)
    {
        static bool listview_inited  = false;

        if(!listview_inited)
        {
            ui.listView_dataRecorder->setModel(m_dataRecorder->loggingModel());
            connect(m_dataRecorder, SIGNAL(loggingUpdated()), this, SLOT(updateDataRecorderLoggingView()));
            listview_inited = true;
        }

        m_dataRecorder->clearLog();
        m_dataRecorder->setDataFile(ui.lineEdit_recordFileName->text());
        m_dataRecorder->setRecordFrequency(ui.lineEdit_recordFrequency->text().toInt());
        m_dataRecorder->setLaunchSensorWaitTime(ui.lineEdit_recorderWaitTime->text().toInt());

        std::string vehicle_state_topic = g_rosNodesArray["base_control"].topics["vehicle_state"];
        std::string gps_topic = g_rosNodesArray["gps"].topics["inspvax"];
        std::string utm_topic = g_rosNodesArray["gps"].topics["odom"];
        //std::string imu_topic = g_rosNodesArray["imu"].topics["corr_imu"];

        m_dataRecorder->setRecordVehicleState(vehicle_state_topic, ui.checkBox_recordRoadwheelAngle->isChecked(),
                                              ui.checkBox_recordSpeed->isChecked());
        m_dataRecorder->setRecordGps(gps_topic, ui.checkBox_recordYaw->isChecked(),
                                     ui.checkBox_recordWGS84->isChecked());
        m_dataRecorder->setRecordUtm(utm_topic, ui.checkBox_recordYaw->isChecked(),
                                     ui.checkBox_recordUTM->isChecked());
        //m_dataRecorder->setRecordImu(imu_topic, ui.checkBox_recordAnglularVel->isChecked(),
         //                            ui.checkBox_recordAccel->isChecked());

        m_dataRecorder->setRecordStamp(ui.checkBox_recordDataStamp->isChecked());
        bool ok = m_dataRecorder->start();

        if(!ok)
        {
            ui.pushButton_startRecordData->setChecked(false);
            return;
        }

        disableRecordDataConfigure(true);
        ui.pushButton_startRecordData->setText(QString("Stop"));
    }
    else
    {
        m_dataRecorder->stop();
        disableRecordDataConfigure(false);
        ui.pushButton_startRecordData->setText(QString("Start"));
    }
}

void av_console::MainWindow::updateDataRecorderLoggingView()
{
  ui.listView_dataRecorder->scrollToBottom();
}

void av_console::MainWindow::on_pushButton_selectRecordFile_clicked()
{
    if(m_recordFileDir.isEmpty())
        m_recordFileDir = "/home/projects";

    QString fileName = QFileDialog::getSaveFileName(this, QString("New File"), m_recordFileDir, "TXT(*txt)");
    if(fileName.isEmpty())
        return;

    ui.lineEdit_recordFileName->setText(fileName);
    m_recordFileDir = fileName;
}

bool av_console::MainWindow::loadRosNodesArrayInfo()
{

    //std::string file = QDir::currentPath().toStdString() + "/src/av_console/cmd/cmd.xml";
    QString path;
    QDir dir;
    path=dir.currentPath();
    std::string file = path.toStdString() + "/src/av_console/cmd/cmd.xml";
    //std::cout << QCoreApplication::applicationDirPath().toStdString() << std::endl;
    //qnode.stampedLog(QNode::Info ,QDir::currentPath().toStdString());
    //qnode.stampedLog(QNode::Info ,QCoreApplication::applicationDirPath().toStdString());
    qnode.stampedLog(QNode::Info ,file);
    tinyxml2::XMLDocument Doc;   //定义xml文件对象
    tinyxml2::XMLError res = Doc.LoadFile(file.c_str());

    if(tinyxml2::XML_ERROR_FILE_NOT_FOUND == res)
    {
        qnode.stampedLog(QNode::Error ,std::string("load ") + file + " failed");
        return false;
    }
    else if(tinyxml2::XML_SUCCESS != res)
    {
        qnode.stampedLog(QNode::Error ,std::string("parse ") + file + " failed");
        return false;
    }
    else
    {
        qnode.stampedLog(QNode::Info ,std::string("parse ") + "cmds successfully");
        return true;
    }
    tinyxml2::XMLElement *pRoot = Doc.RootElement();

    //第一个子节点 RosNodes
    const tinyxml2::XMLElement *pRosNodes = pRoot->FirstChildElement();
    while(pRosNodes)
    {
        std::string ros_nodes_name(pRosNodes->Attribute("name"));
        RosNodes & ros_nodes = g_rosNodesArray[ros_nodes_name];

        const tinyxml2::XMLElement *pChild = pRosNodes->FirstChildElement();
        while(pChild)
        {
            std::string child_name(pChild->Name());
            if(child_name == "LaunchCommand")
            {
                std::string launch_cmd(pChild->GetText());
                ros_nodes.launch_cmd = launch_cmd;
            }
            else if(child_name == "Topic")
            {
                std::string topic_name(pChild->Attribute("name"));
                std::string topic_val(pChild->GetText());

                ros_nodes.topics[topic_name] = topic_val;
            }
            pChild = pChild->NextSiblingElement();//转到下一子节点
        }
        pRosNodes = pRosNodes->NextSiblingElement();//转到下一子节点，链表结构
    }

    //this->displayRosNodesArrayInfo();
    return true;
}

void av_console::MainWindow::displayRosNodesArrayInfo()
{
    for(RosNodesArray::iterator iter=g_rosNodesArray.begin(); iter!=g_rosNodesArray.end(); ++iter)
    {
        std::string name = iter->first;
        RosNodes& rosNodes = iter->second;
        std::cout << name << " cmd: " << rosNodes.launch_cmd << std::endl;

        for(std::unordered_map<std::string, std::string>::iterator it=rosNodes.topics.begin();
            it != rosNodes.topics.end(); ++it)
            std::cout << "topic name: " << it->first << "\ttopic value: " << it->second << std::endl;
        std::cout << "--------------------\r\n";
    }
}
