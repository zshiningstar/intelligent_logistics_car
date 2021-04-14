#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_gps_clicked();

    void on_record_clicked();

    void on_track_clicked();

    void on_livox_clicked();

    void on_leishen_clicked();

    void on_cluster_clicked();

    void on_back_clicked();

    void on_serial_clicked();

    void on_plot_clicked();

    void on_gps_topic_clicked();

    void on_gps_odom_clicked();

    void on_topic_list_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
