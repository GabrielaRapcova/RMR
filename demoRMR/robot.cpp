#include "robot.h"
#include <cmath>
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

    lidarReady = false;

    datacounter = 0;
    rotateMode = false;
    gyroOffset = 0.0;
    gyroInitialized = false;
    sectorCount = 72;
    sectorWidthDeg = 360.0 / sectorCount;
    sectors.resize(sectorCount, 0.0);
    binarySectors.resize(sectorCount, 0);
    maskedSectors.resize(sectorCount, 0);
    obstacleMaxDist = 700.0;
    bestDirectionDeg = 0.0;
    robotRadius = 0.12;
    safetyMargin = 0.0;
    wallFollowing = false;
    wallFollowDirection = 0.0;

    qRegisterMetaType<LaserMeasurement>("LaserMeasurement");
    qRegisterMetaType<std::vector<int>>("std::vector<int>");
#ifndef DISABLE_OPENCV
    qRegisterMetaType<cv::Mat>("cv::Mat");
#endif
#ifndef DISABLE_SKELETON
    qRegisterMetaType<skeleton>("skeleton");
#endif
}

void robot::initAndStartRobot(std::string ipaddress)
{
    forwardspeed = 0;
    rotationspeed = 0;

    robotCom.setLaserParameters(
        [this](const std::vector<LaserData>& dat)->int { return processThisLidar(dat); },
        ipaddress);

    robotCom.setRobotParameters(
        [this](const TKobukiData& dat)->int { return processThisRobot(dat); },
        ipaddress);

#ifndef DISABLE_OPENCV
    robotCom.setCameraParameters(
        std::bind(&robot::processThisCamera, this, std::placeholders::_1),
        "http://" + ipaddress + ":8000/stream.mjpg");
#endif

#ifndef DISABLE_SKELETON
    robotCom.setSkeletonParameters(
        std::bind(&robot::processThisSkeleton, this, std::placeholders::_1));
#endif

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
    forwardspeed = forw;
    rotationspeed = rots;
    useDirectCommands = 0;
}

void robot::setSpeed(double forw, double rots)
{
    v_actual = forw;
    w_actual = rots;

    if(forw == 0 && rots != 0)
        robotCom.setRotationSpeed(rots);
    else if(forw != 0 && rots == 0)
        robotCom.setTranslationSpeed(forw);
    else if(forw != 0 && rots != 0)
        robotCom.setArcSpeed(forw, forw / rots);
    else
        robotCom.setTranslationSpeed(0);

    useDirectCommands = 1;
}

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

    const double tickToMeter = 0.000085292090497737556558;
    double dL = tickToMeter * deltaL;
    double dR = tickToMeter * deltaR;

    fi = ((robotdata.GyroAngle - gyroOffset) / 100.0) * M_PI / 180.0;

    double dS = (dR + dL) / 2.0;
    x += dS * cos(fi);
    y += dS * sin(fi);

    if(datacounter % 5 == 0)
        emit publishPosition(x, y, fi);

    if(hasTarget)
    {
        double dx = targetX - x;
        double dy = targetY - y;
        double distance = sqrt(dx*dx + dy*dy);

        double goalAngle = atan2(dy, dx);
        double goalRelative = goalAngle - fi;

        while(goalRelative > M_PI) goalRelative -= 2*M_PI;
        while(goalRelative < -M_PI) goalRelative += 2*M_PI;

        double goalDeg = goalRelative * 180.0 / M_PI;

        if(distance < 0.05)
        {
            hasTarget = false;
            setSpeed(0, 0);
            datacounter++;
            return 0;
        }

        if(!lidarReady)
        {
            // natoc na ciel
            if(fabs(goalRelative) > 0.2)
                setSpeed(0, 0.8 * goalRelative);

            // jazda rovno
            else
                setSpeed(100,0);

            datacounter++;
            return 0;
        }
        //prevod uhla na sector
        auto angleToSector = [this](double angleDeg)->int
        {
            while(angleDeg >= 180.0) angleDeg -= 360.0;
            while(angleDeg <  -180.0) angleDeg += 360.0;

            int idx = static_cast<int>((angleDeg + 180.0) / sectorWidthDeg);
            return (idx % sectorCount + sectorCount) % sectorCount;
        };

        // je gol blokovany
        int goalIdx = angleToSector(goalDeg);
        bool goalBlocked = maskedSectors[goalIdx] == 1;

        std::vector<double> candidates = candidateDirections;

        candidates.push_back(goalDeg);

        //ignorovanie masky
        bool ignoreMask = (fabs(v_actual) < 20);
        std::vector<double> feasible;

        for(double cand : candidates)
        {
            int idx = angleToSector(cand);
            if(ignoreMask || maskedSectors[idx] == 0)
                feasible.push_back(cand);
        }

        //obzeranie volneho smeru
        if(feasible.empty())
        {
            setSpeed(0, 0.4);
            return 0;
        }

        double bestCost = 1e9;
        double chosenDeg = feasible.front();

        for(double cand : feasible)
        {
            auto angleDiff = [](double a, double b)
            {
                double d = a - b;
                while(d > 180) d -= 360;
                while(d < -180) d += 360;
                return fabs(d);
            };

            //nastavovanie váh
            double goalDiff  = angleDiff(cand, goalDeg);
            double robotDiff = angleDiff(cand, 0.0);
            double prevDiff  = angleDiff(cand, bestDirectionDeg);
            double obstaclePenalty = sectors[angleToSector(cand)];

            if(goalBlocked && !wallFollowing)
            {
                wallFollowing = true;
                wallFollowDirection = bestDirectionDeg;
            }

            if(!goalBlocked && fabs(goalDeg) < 20)
            {
                wallFollowing = false;
            }


            double cost;

            if(wallFollowing)
            {
                double wallDiff = angleDiff(cand, wallFollowDirection);

                cost =
                    1.5 * goalDiff +
                    1.0 * robotDiff +
                    2.5 * wallDiff +
                    0.5 * obstaclePenalty;
            }
            else
            {
                cost =
                    4.3 * goalDiff +
                    2.0 * robotDiff +
                    4.0 * prevDiff +
                    0.5 * obstaclePenalty;
            }

            if(cost < bestCost)
            {
                bestCost = cost;
                chosenDeg = cand;
            }
        }

        bestDirectionDeg = chosenDeg;

        double angleError = chosenDeg * M_PI / 180.0;

        if(fabs(angleError) > 0.6)
            setSpeed(0, 0.9 * angleError);
        else if(fabs(angleError) > 0.25)
            setSpeed(80, 0.8 * angleError);
        else
            setSpeed(150, 0.5 * angleError);
    }

    datacounter++;
    return 0;
}

int robot::processThisLidar(const std::vector<LaserData>& laserData)
{
    copyOfLaserData = laserData;

    sectors.assign(sectorCount, 0.0);
    binarySectors.assign(sectorCount, 0);
    maskedSectors.assign(sectorCount, 0);
    freeGaps.clear();
    candidateDirections.clear();

    //polarny histogram
    for(const auto& point : laserData)
    {
        double angleDeg = -point.scanAngle;
        double distance = point.scanDistance;

        if(distance <= 0.0 || distance > obstacleMaxDist)
            continue;

        while(angleDeg >= 180.0) angleDeg -= 360.0;
        while(angleDeg <  -180.0) angleDeg += 360.0;

        int idx = (angleDeg + 180.0) / sectorWidthDeg;
        idx = std::max(0, std::min(sectorCount - 1, idx));

        double a = obstacleMaxDist;
        double b = 1.0;
        double c = 1.0;

        double mi = c * c * (a - b * distance);

        if(mi < 0)
            mi = 0;

        sectors[idx] += mi;
    }

    double thresholdHigh = 40.0;
    double thresholdLow  = 20.0;

    std::vector<int> previousBinary = binarySectors;

    for(int i = 0; i < sectorCount; i++)
    {
        //obsadeny
        if(sectors[i] > thresholdHigh)
        {
            binarySectors[i] = 1;
        }
        //volny
        else if(sectors[i] < thresholdLow)
        {
            binarySectors[i] = 0;
        }
        //mozno
        else
        {
            binarySectors[i] = previousBinary[i];
        }
    }

    //zvacsenie neprechodnosti + maskovanie
    for(int i = 0; i < sectorCount; i++)
    {
        if(binarySectors[i] == 1)
        {
            double approxDist = obstacleMaxDist - sectors[i];
            approxDist = std::max(approxDist, 1.0);

            double effectiveRadius = robotRadius + safetyMargin;

            double ratio = effectiveRadius / approxDist;

            ratio = std::min(1.0, ratio);

            double angleExpandRad = asin(ratio);
            double angleExpandDeg = angleExpandRad * 180.0 / M_PI;

            int expand = std::max(2, (int)(angleExpandDeg / sectorWidthDeg));

            for(int j = -expand; j <= expand; j++)
            {
                int idx = i + j;
                if(idx >= 0 && idx < sectorCount)
                    maskedSectors[idx] = 1;
            }
        }
    }

    bool inGap = false;
    int start = 0;

    //zaplnanie gap
    for(int i = 0; i < sectorCount; i++)
    {
        if(maskedSectors[i] == 0 && !inGap)
        {
            inGap = true;
            start = i;
        }
        else if(maskedSectors[i] == 1 && inGap)
        {
            inGap = false;
            freeGaps.push_back({start, i - 1});
        }
    }

    if(inGap)
        freeGaps.push_back({start, sectorCount - 1});

    int wideThreshold = 8;

    //vyber smeru
    for(auto& g : freeGaps)
    {
        int width = g.second - g.first + 1;  //gap

        if(width < wideThreshold)
        {
            int mid = (g.first + g.second) / 2;  //stred
            candidateDirections.push_back(mid * sectorWidthDeg - 180.0);
        }
        else
        {
            int left = g.first + width * 0.25;
            int right = g.second - width * 0.25;

            candidateDirections.push_back(left * sectorWidthDeg - 180.0);
            candidateDirections.push_back(right * sectorWidthDeg - 180.0);
        }
    }

    emit publishLidar(copyOfLaserData);
    lidarReady = true;
    return 0;
}

#ifndef DISABLE_OPENCV
int robot::processThisCamera(cv::Mat cameraData)
{
    cameraData.copyTo(frame[(actIndex+1)%3]);
    actIndex=(actIndex+1)%3;
    emit publishCamera(frame[actIndex]);
    return 0;
}
#endif

#ifndef DISABLE_SKELETON
int robot::processThisSkeleton(skeleton skeledata)
{
    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));
    emit publishSkeleton(skeleJoints);
    return 0;
}
#endif

