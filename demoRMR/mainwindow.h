#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <vector>

#include "robot.h"

#ifndef DISABLE_JOYSTICK
#include <QJoysticks.h>
#endif

namespace Ui {
class MainWindow;
}

///
/// Trieda hlavného okna aplikácie – obsahuje všetky gombíky a spúšťanie robota.
///
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
#ifndef DISABLE_OPENCV
    bool useCamera1;
    int actIndex;
    cv::Mat frame[3];
#endif

#ifndef DISABLE_SKELETON
    int updateSkeletonPicture;
    skeleton skeleJoints;
#endif

    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // štart
    void on_pushButton_9_clicked();
    // manuálne ovládanie
    void on_pushButton_2_clicked(); // dopredu
    void on_pushButton_3_clicked(); // dozadu
    void on_pushButton_6_clicked(); // doľava
    void on_pushButton_5_clicked(); // doprava
    void on_pushButton_4_clicked(); // stop
    void on_pushButton_clicked();   // prepínanie kamera/laser
    void on_pushButton_10_clicked(); // ➕ GO TO TARGET (zadanie cieľa X, Y)

    // kreslenie dát
    int paintThisLidar(const std::vector<LaserData> &laserData);
#ifndef DISABLE_OPENCV
    int paintThisCamera(const cv::Mat &cameraData);
#endif
#ifndef DISABLE_SKELETON
    int paintThisSkeleton(const skeleton &skeledata);
#endif

private:
    robot _robot;
    Ui::MainWindow *ui;

    void paintEvent(QPaintEvent *event) override;

    int updateLaserPicture;
    std::vector<LaserData> copyOfLaserData;
    int datacounter;
    std::string ipaddress;

    QTimer *timer;

#ifndef DISABLE_JOYSTICK
    QJoysticks *instance;
#endif

public slots:
    void setUiValues(double robotX, double robotY, double robotFi);
};

#endif // MAINWINDOW_H
