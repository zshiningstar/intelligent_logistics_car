#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}



void MainWindow::on_gps_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch daoyuan ceshi.launch'&");
}

void MainWindow::on_record_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch driver gps_path_record.launch'&");
}

void MainWindow::on_track_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch driver path_track_forward.launch'&");
}

void MainWindow::on_livox_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch livox_ros_driver livox_lidar_rviz.launch'&");
}

void MainWindow::on_leishen_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch lslidar_c16_decoder lslidar_c16.launch'&");
}

void MainWindow::on_cluster_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch euclidean_cluster euclidean_cluster.launch'&");
}

void MainWindow::on_back_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch driver path_track_back.launch'&");
}

void MainWindow::on_serial_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch driver temp_go.launch'&");
}

void MainWindow::on_plot_clicked()
{
    system("gnome-terminal -x bash -c 'cd /home/robot/logistics_ws/src/recorder/data; python plot.py path.txt'&");
}

void MainWindow::on_gps_topic_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; rostopic echo /gps'&");
}

void MainWindow::on_gps_odom_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; rostopic echo /odom'&");
}

void MainWindow::on_topic_list_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch driver temp_back.launch'&");
}

void MainWindow::on_record_room_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch driver temp_record.launch'&");
}
