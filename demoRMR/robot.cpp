#include "robot.h"
#include <cmath>
// #include <iostream>
#include <string>

robot::robot(QObject *parent) : QObject(parent)
{
    x = 0.0;
    y = 0.0;
    fi = 0.0;

    targetX = 0.0;
    targetY = 0.0;
    hasTarget = false;

    datacounter = 0;
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
    setTarget(0.5, 0.7);
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

void robot::setTarget(double tx, double ty)
{
    targetX = tx;
    targetY = ty;
    hasTarget = true;
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

        datacounter ++;
        return 0;
    }

    int deltaL = robotdata.EncoderLeft - prevEncoderLeft;
    int deltaR = robotdata.EncoderRight - prevEncoderRight;

    prevEncoderLeft = robotdata.EncoderLeft;
    prevEncoderRight = robotdata.EncoderRight;

    double tickToMeter = 0.000085292090497737556558;
    double wheelBase = 0.23;

    double dL = tickToMeter * deltaL;
    double dR = tickToMeter * deltaR;

    double fi_old = fi;

    fi = (robotdata.GyroAngle / 100.0) * M_PI / 180.0;

    if(fabs(dR - dL) > 0.0001)
    {
        double R = (wheelBase * (dR + dL)) / (2.0 * (dR - dL));

        x += R * (sin(fi) - sin(fi_old));
        y -= R * (cos(fi) - cos(fi_old));
    }
    else
    {
        double dS = (dR + dL) / 2.0;

        x += dS * cos(fi_old);
        y += dS * sin(fi_old);
    }

    if(datacounter % 5 == 0)
    {
        emit publishPosition(x, y, fi);
    }
    //std::cout << x << " " << y << " " << fi << std::endl;

    if(hasTarget)
    {
        double dx = targetX - x;
        double dy = targetY - y;

        double distance = sqrt(dx*dx + dy*dy);
        double targetAngle = atan2(dy, dx);
        double angleError = targetAngle - fi;

        while(angleError > M_PI)  angleError -= 2*M_PI;
        while(angleError < -M_PI) angleError += 2*M_PI;

        if(distance < 0.05)
        {
            setSpeed(0,0);
            hasTarget = false;
        }
        else
        {
            double v = 150 * distance;
            if(v > 250) v = 250;

            double w = 1.0 * angleError;

            setSpeed(v, w);
        }
    }

    ///---tu sa posielaju rychlosti do robota... vklude zakomentujte ak si chcete spravit svoje
    if(useDirectCommands==0)
    {
        if(forwardspeed==0 && rotationspeed!=0)
            robotCom.setRotationSpeed(rotationspeed);
        else if(forwardspeed!=0 && rotationspeed==0)
            robotCom.setTranslationSpeed(forwardspeed);
        else if((forwardspeed!=0 && rotationspeed!=0))
            robotCom.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
        else
            robotCom.setTranslationSpeed(0);
    }
    datacounter++;

    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru
int robot::processThisLidar(const std::vector<LaserData>& laserData)
{

    copyOfLaserData=laserData;

    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    emit publishLidar(copyOfLaserData);
   // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

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
