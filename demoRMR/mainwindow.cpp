#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ipaddress = "127.0.0.1";
    ui->setupUi(this);

    datacounter = 0;

#ifndef DISABLE_OPENCV
    actIndex = -1;
    useCamera1 = false;
#endif
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::black);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);

    QRect rect = ui->widget->geometry();
    rect.translate(0, 15);
    painter.drawRect(rect);

#ifndef DISABLE_OPENCV
    if (useCamera1 == true && actIndex > -1)
    {
        QImage image = QImage((uchar*)frame[actIndex].data,
                              frame[actIndex].cols,
                              frame[actIndex].rows,
                              frame[actIndex].step,
                              QImage::Format_RGB888);
        painter.drawImage(rect, image.rgbSwapped());
    }
    else
#endif
    {
        if (updateLaserPicture == 1)
        {
            updateLaserPicture = 0;

            pero.setColor(Qt::red);
            painter.setPen(pero);
            painter.drawEllipse(QPoint(rect.width()/2 + rect.topLeft().x(),
                                       rect.height()/2 + rect.topLeft().y()),
                                15, 15);
            painter.drawLine(QPoint(rect.width()/2 + rect.topLeft().x(),
                                    rect.height()/2 + rect.topLeft().y()),
                             QPoint(rect.width()/2 + rect.topLeft().x(),
                                    rect.height()/2 + rect.topLeft().y() - 15));

            pero.setColor(Qt::green);
            painter.setPen(pero);

            for (const auto &k : copyOfLaserData)
            {
                int dist = k.scanDistance / 20;
                int xp = rect.width() - (rect.width()/2 + dist*2*sin((360.0 - k.scanAngle)*3.14159/180.0)) + rect.topLeft().x();
                int yp = rect.height() - (rect.height()/2 + dist*2*cos((360.0 - k.scanAngle)*3.14159/180.0)) + rect.topLeft().y();
                if (rect.contains(xp, yp))
                    painter.drawEllipse(QPoint(xp, yp), 2, 2);
            }
        }
    }

#ifndef DISABLE_SKELETON
    if (updateSkeletonPicture == 1)
    {
        painter.setPen(Qt::red);
        for (int i = 0; i < 75; i++)
        {
            int xp = rect.width() - rect.width() * skeleJoints.joints[i].x + rect.topLeft().x();
            int yp = rect.height() * skeleJoints.joints[i].y + rect.topLeft().y();
            if (rect.contains(xp, yp))
                painter.drawEllipse(QPoint(xp, yp), 2, 2);
        }
    }
#endif
}

void MainWindow::setUiValues(double robotX, double robotY, double robotFi)
{
    ui->lineEdit_2->setText(QString::number(robotX));
    ui->lineEdit_3->setText(QString::number(robotY));
    ui->lineEdit_4->setText(QString::number(robotFi));
}

void MainWindow::on_pushButton_9_clicked() // START
{
    connect(&_robot, SIGNAL(publishPosition(double,double,double)),
            this, SLOT(setUiValues(double,double,double)));
    connect(&_robot, SIGNAL(publishLidar(const std::vector<LaserData> &)),
            this, SLOT(paintThisLidar(const std::vector<LaserData> &)));

#ifndef DISABLE_OPENCV
    connect(&_robot, SIGNAL(publishCamera(const cv::Mat &)),
            this, SLOT(paintThisCamera(const cv::Mat &)));
#endif
#ifndef DISABLE_SKELETON
    connect(&_robot, SIGNAL(publishSkeleton(const skeleton &)),
            this, SLOT(paintThisSkeleton(const skeleton &)));
#endif

    _robot.initAndStartRobot(ipaddress);

#ifndef DISABLE_JOYSTICK
    instance = QJoysticks::getInstance();
    connect(instance, &QJoysticks::axisChanged,
            [this](const int js, const int axis, const qreal value) {
                double forw = 0, rot = 0;
                if (axis == 1) { forw = -value * 300; }
                if (axis == 0) { rot = -value * (3.14159 / 2.0); }
                this->_robot.setSpeedVal(forw, rot);
            });
#endif
}

void MainWindow::on_pushButton_2_clicked() { _robot.setSpeed(500, 0); }
void MainWindow::on_pushButton_3_clicked() { _robot.setSpeed(-250, 0); }
void MainWindow::on_pushButton_6_clicked() { _robot.setSpeed(0, 3.14159/2); }
void MainWindow::on_pushButton_5_clicked() { _robot.setSpeed(0, -3.14159/2); }
void MainWindow::on_pushButton_4_clicked() { _robot.setSpeed(0, 0); }

void MainWindow::on_pushButton_10_clicked()
{
    double targetX = ui->lineEdit_5->text().toDouble();
    double targetY = ui->lineEdit_6->text().toDouble();

    _robot.setTarget(targetX, targetY);
}
// ---------------------------

void MainWindow::on_pushButton_clicked()
{
#ifndef DISABLE_OPENCV
    if (useCamera1 == true)
    {
        useCamera1 = false;
        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1 = true;
        ui->pushButton->setText("use laser");
    }
#endif
}

int MainWindow::paintThisLidar(const std::vector<LaserData> &laserData)
{
    copyOfLaserData = laserData;
    updateLaserPicture = 1;
    update();
    return 0;
}

#ifndef DISABLE_OPENCV
int MainWindow::paintThisCamera(const cv::Mat &cameraData)
{
    cameraData.copyTo(frame[(actIndex + 1) % 3]);
    actIndex = (actIndex + 1) % 3;
    updateLaserPicture = 1;
    return 0;
}
#endif

#ifndef DISABLE_SKELETON
int MainWindow::paintThisSkeleton(const skeleton &skeledata)
{
    memcpy(&skeleJoints, &skeledata, sizeof(skeleton));
    updateSkeletonPicture = 1;
    return 0;
}
#endif
