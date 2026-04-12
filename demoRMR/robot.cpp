#include "robot.h"
#include <cmath>
// #include <iostream>
#include <string>
#include <vector>

robot::robot(QObject *parent) : QObject(parent)
{
    x = 0.0;
    y = 0.0;
    fi = 0.0;

    targetX = 0.0;
    targetY = 0.0;
    hasTarget = false;
    v_actual = 0;
    w_actual = 0.0;

    datacounter = 0;
    rotateMode = false;
    gyroOffset = 0.0;
    gyroInitialized = false;

    //mapping
    resolution = 0.05;
    gridWidth = 14/resolution;
    gridHeight = 14/resolution;

    grid.resize(gridWidth);
    for(int i = 0; i < gridWidth; i++)
    {
        grid[i].resize(gridHeight);
    }

    tempGrid.resize(gridWidth);
    for(int i = 0; i < gridWidth; i++)
    {
        tempGrid[i].resize(gridHeight);

        for(int j = 0; j < gridHeight; j++)
        {
            tempGrid[i][j] = 0;
        }
    }
    minDist = 0.23;
    maxDist = 3.0;

    //navigacia
    sectorCount = 36;
    sectorWidthDeg = 360.0 / sectorCount;
    sectors.resize(sectorCount, 0);
    obstacleMaxDist = 1.5;
    bestDirectionDeg = 0.0;
    qRegisterMetaType<LaserMeasurement>("LaserMeasurement");
#ifndef DISABLE_OPENCV
    qRegisterMetaType<cv::Mat>("cv::Mat");
#endif
#ifndef DISABLE_SKELETON
    qRegisterMetaType<skeleton>("skeleton");
#endif
}

void robot::initAndStartRobot(std::string ipaddress)
{

    forwardspeed=0;
    rotationspeed=0;
    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robotCom.setLaserParameters([this](const std::vector<LaserData>& dat)->int{return processThisLidar(dat);},ipaddress);
    robotCom.setRobotParameters([this](const TKobukiData& dat)->int{return processThisRobot(dat);},ipaddress);
#ifndef DISABLE_OPENCV
    robotCom.setCameraParameters(std::bind(&robot::processThisCamera,this,std::placeholders::_1),"http://"+ipaddress+":8000/stream.mjpg");
#endif
#ifndef DISABLE_SKELETON
    robotCom.setSkeletonParameters(std::bind(&robot::processThisSkeleton,this,std::placeholders::_1));
#endif
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robotCom.robotStart();


}

void robot::setTarget(double x, double y)
{
    targetX = x;
    targetY = y;
    hasTarget = true;
    rotateMode = true;
    w_actual = 0.0;
    v_actual = 0.0;
}

void robot::setSpeedVal(double forw, double rots)
{
    forwardspeed=forw;
    rotationspeed=rots;
    useDirectCommands=0;
}

void robot::setSpeed(double forw, double rots)
{
    if(forw==0 && rots!=0)
        robotCom.setRotationSpeed(rots);
    else if(forw!=0 && rots==0)
        robotCom.setTranslationSpeed(forw);
    else if((forw!=0 && rots!=0))
        robotCom.setArcSpeed(forw,forw/rots);
    else
        robotCom.setTranslationSpeed(0);
    useDirectCommands=1;
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int robot::processThisRobot(const TKobukiData &robotdata)
{
    if(datacounter == 0)
    {
        prevEncoderLeft = robotdata.EncoderLeft;
        prevEncoderRight = robotdata.EncoderRight;
        gyroOffset = robotdata.GyroAngle;
        datacounter++;
        return 0;
    }

    int deltaL = robotdata.EncoderLeft - prevEncoderLeft;
    int deltaR = robotdata.EncoderRight - prevEncoderRight;

    if(deltaL > 32768) deltaL -= 65536;
    if(deltaL < -32768) deltaL += 65536;

    if(deltaR > 32768) deltaR -= 65536;
    if(deltaR < -32768) deltaR += 65536;

    prevEncoderLeft = robotdata.EncoderLeft;
    prevEncoderRight = robotdata.EncoderRight;

    double tickToMeter = 0.000085292090497737556558;
    double wheelBase = 0.23;

    double dL = tickToMeter * deltaL;
    double dR = tickToMeter * deltaR;

    fi = ((robotdata.GyroAngle - gyroOffset) / 100.0) * M_PI / 180.0;

    double dS = (dR + dL) / 2.0;
    x += dS * cos(fi);
    y += dS * sin(fi);


    if(datacounter % 5 == 0)
    {
        emit publishPosition(x, y, fi);
    }

    if(hasTarget)
    {
        double dx = targetX - x;
        double dy = targetY - y;

        double distance = sqrt(dx*dx + dy*dy);
        double targetAngle = atan2(dy, dx);
        double angleError = targetAngle - fi;

        if(angleError > M_PI)  angleError -= 2.0 * M_PI;
        if(angleError < -M_PI) angleError += 2.0 * M_PI;

        double v_target = 0.0;
        double w_target = 0.0;


        if(distance < 0.01)
        {
            hasTarget = false;
            rotateMode = false;
            v_target = 0.0;
            w_target = 0.0;
            v_actual = 0.0;
            w_actual = 0.0;
        }
        else
        {
            double enterRot = 0.20;
            double exitRot  = 0.10;

            if(rotateMode)
            {
                if(fabs(angleError) < exitRot)
                    rotateMode = false;
            }
            else
            {
                if(fabs(angleError) > enterRot)
                    rotateMode = true;
            }

            if(rotateMode)
            {

                v_target = 0.0;
                w_target = 0.8 * angleError;

                if(w_target > 0.5)  w_target = 0.5;
                if(w_target < 0.5) w_target = 0.5;
            }
            else
            {

                double v_max = 250.0;
                double v_min = 20.0;
                double slowDownDist = 0.2;

                if(distance > slowDownDist)
                {
                    v_target = v_max;
                }
                else
                {
                    v_target = v_min + (v_max - v_min) * (distance / slowDownDist);
                }

                if(v_target < v_min) v_target = v_min;
                if(v_target > v_max) v_target = v_max;


                /*double angleScale = cos(angleError);
                if(angleScale < 0.0) angleScale = 0.0;
                v_target *= angleScale;*/


                w_target = 0.8 * angleError;
                if(w_target > 0.5)  w_target = 0.5;
                if(w_target < -0.5) w_target = -0.5;
            }
        }

        double acc_v = 8.0;
        double acc_v2 = 15.0;
        if(v_actual < v_target)
        {
            v_actual += acc_v;
            if(v_actual > v_target) v_actual = v_target;
        }
        else if(v_actual > v_target)
        {
            v_actual -= acc_v2;
            if(v_actual < v_target) v_actual = v_target;
        }

        double acc_w = 0.05;
        if(w_actual < w_target)
        {
            w_actual += acc_w;
            if(w_actual > w_target) w_actual = w_target;
        }
        else if(w_actual > w_target)
        {
            w_actual -= acc_w;
            if(w_actual < w_target) w_actual = w_target;
        }


        if(!hasTarget && fabs(v_actual) < 0.01 && fabs(w_actual) < 0.01)
        {
            setSpeed(0, 0);
        }
        else
        {
            setSpeed(v_actual, w_actual);
        }
    }

    if(useDirectCommands == 0)
    {
        if(forwardspeed == 0 && rotationspeed != 0)
            robotCom.setRotationSpeed(rotationspeed);
        else if(forwardspeed != 0 && rotationspeed == 0)
            robotCom.setTranslationSpeed(forwardspeed);
        else if((forwardspeed != 0 && rotationspeed != 0))
            robotCom.setArcSpeed(forwardspeed, forwardspeed / rotationspeed);
        else
            robotCom.setTranslationSpeed(0);
    }

    Position p;

    p.x = x;
    p.y = y;
    p.fi = fi;
    p.timestamp = robotdata.timestamp;

    positionHistory.push_back(p);
    if(positionHistory.size() > 1000)
    {
        positionHistory.erase(positionHistory.begin());
    }

    datacounter++;
    return 0;
}

robot::Position robot::interpolatePosition(uint32_t time)
{
    if(positionHistory.size() < 2)
        return positionHistory.back();

    for(int i=1; i<positionHistory.size(); i++)
    {
        if(positionHistory[i].timestamp >= time)
        {
            Position p1 = positionHistory[i-1];
            Position p2 = positionHistory[i];

            double ratio =
                double(time - p1.timestamp) /
                double(p2.timestamp - p1.timestamp);

            Position result;

            result.x =
                p1.x + ratio * (p2.x - p1.x);

            result.y =
                p1.y + ratio * (p2.y - p1.y);

            result.fi =
                p1.fi + ratio * (p2.fi - p1.fi);

            result.timestamp = time;

            return result;
        }
    }

    return positionHistory.back();
}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru
int robot::processThisLidar(const std::vector<LaserData>& laserData)
{

    copyOfLaserData=laserData;

    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    // updateLaserPicture=1;
    sectors.assign(sectorCount, 0);
    int robot_i = (x / resolution) + gridWidth / 2;
    int robot_j = (y / resolution) + gridHeight / 2;

    for (const auto& point : laserData)
    {
        double angleDeg = point.scanAngle;
        double distance = point.scanDistance;
        angleDeg = -angleDeg;

        double distance_m = distance / 1000.0;
        double angleRad = angleDeg * M_PI / 180.0;
        if (distance_m <= minDist || distance_m > maxDist)
            continue;

        if(positionHistory.size() < 2)
            continue;
        Position pos = interpolatePosition(point.timestamp);

        double x_glob = pos.x + distance_m * cos(pos.fi + angleRad);
        double y_glob = pos.y + distance_m * sin(pos.fi + angleRad);

        int i = (x_glob / resolution) + gridWidth / 2;
        int j = (y_glob / resolution) + gridHeight / 2;

        if(i >= 0 && i < gridWidth && j >= 0 && j < gridHeight)
        {
            drawLine(robot_i, robot_j, i, j);
            tempGrid[i][j]++;

            if(tempGrid[i][j] >= 15)
            {
                grid[i][j] = 1;
            }

            if(tempGrid[i][j] > 80)
            {
                tempGrid[i][j] = 80;
            }
        }

        while (angleDeg >= 180.0) angleDeg -= 360.0;
        while (angleDeg < -180.0) angleDeg += 360.0;

        //navigation
        int index = static_cast<int>((angleDeg + 180.0) / sectorWidthDeg);

        if (index < 0) index = 0;
        if (index >= sectorCount) index = sectorCount - 1;

        sectors[index] = 1;
    }
    emit publishLidar(copyOfLaserData);
    // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

void robot::drawLine(int x0, int y0,
                     int x1, int y1)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    while(true)
    {
        if(x0 >= 0 && x0 < gridWidth &&
            y0 >= 0 && y0 < gridHeight)
        {
            if(grid[x0][y0] != 1)
                grid[x0][y0] = 0;
        }

        if(x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;

        if(e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }

        if(e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

const std::vector<std::vector<int>>& robot::getGrid() const
{
    return grid;
}

#ifndef DISABLE_OPENCV
///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z kamery
int robot::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka

    emit publishCamera(frame[actIndex]);
    return 0;
}
#endif

#ifndef DISABLE_SKELETON
/// vola sa ked dojdu nove data z trackera
int robot::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    emit publishSkeleton(skeleJoints);
    return 0;
}
#endif
