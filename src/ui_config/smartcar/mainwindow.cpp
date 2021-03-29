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
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch nuogeng driver.launch'&");
}

void MainWindow::on_record_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch driver path_recorder.launch'&");
}

void MainWindow::on_track_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/logistics_ws/devel/setup.bash; roslaunch driver path_track.launch'&");
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

void MainWindow::on_master_clicked()
{
    system("gnome-terminal -x bash -c 'cd /home/nvidia/logistics_ws/src/recorder/data; python plot.py path.txt'&");
}
