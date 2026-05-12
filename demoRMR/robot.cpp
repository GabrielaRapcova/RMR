#include "robot.h"
#include <cmath>
// #include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <fstream>

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

    //planning
    goal_X = 0.0;
    goal_Y = 0.0;
    currentMainPoint = 0;
    followingPath = false;

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

    distanceField.resize(gridWidth);

    for(int i = 0; i < gridWidth; i++)
    {
        distanceField[i].resize(gridHeight);
        for(int j = 0; j < gridHeight; j++)
        {
            distanceField[i][j] = 0.0;  // ADD THIS LINE
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
    mappingEnabled = true;

    //monte carlo
    localizationEnabled = false;

    particleCount = 500;
    a1 = 0.05;
    a2 = 0.05;
    a3 = 0.1;
    a4 = 0.05;
    prevOdomX = x;
    prevOdomY = y;
    prevOdomFi = fi;
    estimatedX = 0.0;
    estimatedY = 0.0;
    estimatedFi = 0.0;
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

void robot::setGoal(double X, double Y)
{
    goal_X = X;
    goal_Y = Y;
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

void robot::initializeParticles()
{
    particles.clear();

    for(int k = 0; k < particleCount; k++)
    {
        Particle p;

        double noise_x = (((double)rand() / RAND_MAX) - 0.5) * 1.0;
        double noise_y = (((double)rand() / RAND_MAX) - 0.5) * 1.0;
        double noise_fi = (((double)rand() / RAND_MAX) - 0.5) * M_PI / 2;

        p.x = x + noise_x;
        p.y = y + noise_y;
        p.fi = fi + noise_fi;

        p.weight = 1.0 / particleCount;

        particles.push_back(p);
    }
}

const std::vector<robot::Particle>& robot::getParticles() const
{
    return particles;
}
double robot::getEstimatedX() const
{
    return estimatedX;
}

double robot::getEstimatedY() const
{
    return estimatedY;
}

double robot::getEstimatedFi() const
{
    return estimatedFi;
}

double robot::sampleNormal(double variance)
{
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 1.0);
    double u2 = ((double)rand() + 1.0) / ((double)RAND_MAX + 1.0);

    double z =
        sqrt(-2.0 * log(u1)) *
        cos(2.0 * M_PI * u2);

    return z * sqrt(variance);
}

void robot::motionUpdate()
{
    double dx = x - prevOdomX;
    double dy = y - prevOdomY;

    double trans =
        sqrt(dx*dx + dy*dy);

    double rot1 =
        atan2(dy, dx) - prevOdomFi;

    double rot2 =
        fi - prevOdomFi - rot1;

    rot1 = normalizeAngle(rot1);
    rot2 = normalizeAngle(rot2);

    for(auto &p : particles)
    {
        double rot1_hat =
            rot1 -
            sampleNormal(
                a1 * rot1 * rot1 +
                a2 * trans * trans);

        double trans_hat =
            trans -
            sampleNormal(
                a3 * trans * trans +
                a4 * rot1 * rot1 +
                a4 * rot2 * rot2);

        double rot2_hat =
            rot2 -
            sampleNormal(
                a1 * rot2 * rot2 +
                a2 * trans * trans);

        p.x +=
            trans_hat *
            cos(p.fi + rot1_hat);

        p.y +=
            trans_hat *
            sin(p.fi + rot1_hat);

        p.fi += rot1_hat + rot2_hat;

        p.fi = normalizeAngle(p.fi);
    }

    prevOdomX = x;
    prevOdomY = y;
    prevOdomFi = fi;
}

void robot::computeDistanceField()
{
    bool obstacleFound = false;

    for(int i = 0; i < gridWidth; i++)
    {
        for(int j = 0; j < gridHeight; j++)
        {
            if(grid[i][j] == 1)
            {
                obstacleFound = true;
                break;
            }
        }

        if(obstacleFound)
            break;
    }

    if(!obstacleFound)
        return;

    int searchRadius = 20;

    for(int i = 0; i < gridWidth; i++)
    {
        for(int j = 0; j < gridHeight; j++)
        {
            if(grid[i][j] == 1)
            {
                distanceField[i][j] = 0.0;
                continue;
            }

            double minDistSq = 1e9;

            for(int x = std::max(0, i - searchRadius);
                 x < std::min(gridWidth, i + searchRadius);
                 x++)
            {
                for(int y = std::max(0, j - searchRadius);
                     y < std::min(gridHeight, j + searchRadius);
                     y++)
                {
                    if(grid[x][y] == 1)
                    {
                        double dx = i - x;
                        double dy = j - y;

                        double distSq =
                            dx * dx + dy * dy;

                        if(distSq < minDistSq)
                        {
                            minDistSq = distSq;
                        }
                    }
                }
            }

            distanceField[i][j] =
                sqrt(minDistSq) * resolution;
        }
    }
}

const std::vector<std::vector<double>>& robot::getDistanceField() const
{
    return distanceField;
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
            if (followingPath)
            {
                currentMainPoint++;

                if (currentMainPoint < mainpoints.size())
                {
                    Cell c = mainpoints[currentMainPoint];

                    if (!isInsideGrid(c.i, c.j)) {
                        followingPath = false;
                        return 0;
                    }

                    double wx = (c.i - gridWidth / 2 + 0.5) * resolution;
                    double wy = (c.j - gridHeight / 2 + 0.5) * resolution;

                    setTarget(wx, wy);
                    return 0;
                }
                else
                {
                    followingPath = false;
                }
            }

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
                if(w_target < -0.5) w_target = -0.5;
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


        if(!hasTarget ||( fabs(v_actual) < 0.01 && fabs(w_actual) < 0.01))
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
    p.timestamp = robotdata.synctimestamp;

    positionHistory.push_back(p);
    if(positionHistory.size() > 1000)
    {
        positionHistory.erase(positionHistory.begin());
    }

    static int counter = 0;

    counter++;

    if(counter % 20 == 0)
    {
        computeDistanceField();
    }

    datacounter++;
    return 0;
}

double robot::normalizeAngle(double a) const
{
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

bool robot::isInsideGrid(int i, int j) const
{
    return (i >= 0 && i < gridWidth &&
            j >= 0 && j < gridHeight);
}

void robot::planToGoal()
{
    followingPath = false;
    currentMainPoint = 0;

    int start_i = int(round(x / resolution)) + gridWidth / 2;
    int start_j = int(round(y / resolution)) + gridHeight / 2;

    int goal_i = int(round(goal_X / resolution)) + gridWidth / 2;
    int goal_j = int(round(goal_Y / resolution)) + gridHeight / 2;

    if (!isInsideGrid(goal_i, goal_j))
        return;

    if (grid[goal_i][goal_j] == 1)
        return;

    floodFill(goal_i, goal_j);
    path = extractPath(start_i, start_j);
    qDebug() << "PATH size:" << path.size();

    if (path.empty())
        return;

    mainpoints.clear();
    mainpoints.push_back(path[0]);
    qDebug() << "MAINPOINTS size:" << mainpoints.size();

    for (int i = 1; i < path.size() - 1; i++)
    {
        int dx1 = path[i].i - path[i-1].i;
        int dy1 = path[i].j - path[i-1].j;

        int dx2 = path[i+1].i - path[i].i;
        int dy2 = path[i+1].j - path[i].j;

        if (dx1 != dx2 || dy1 != dy2)
        {
            mainpoints.push_back(path[i]);
        }
    }

    mainpoints.push_back(path.back());

    currentMainPoint = 0;
    followingPath = true;

    Cell c = mainpoints[currentMainPoint];
    double wx = (c.i - gridWidth / 2 + 0.5) * resolution;
    double wy = (c.j - gridHeight / 2 + 0.5) * resolution;

    setTarget(wx, wy);
}

void robot::inflateObstacles()
{
    std::vector<std::vector<int>> inflated = planGrid;

    int r = int((0.23) / resolution);

    for (int i = 0; i < gridWidth; i++)
    {
        for (int j = 0; j < gridHeight; j++)
        {
            if (planGrid[i][j] == 1)
            {
                for (int dx = -r; dx <= r; dx++)
                {
                    for (int dy = -r; dy <= r; dy++)
                    {
                        int ni = i + dx;
                        int nj = j + dy;

                        if (!isInsideGrid(ni, nj))
                            continue;

                        double dist = sqrt(dx*dx + dy*dy);
                        if (dist <= r)
                        {
                            inflated[ni][nj] = 1;
                        }
                    }
                }
            }
        }
    }

    planGrid = inflated;
}


void robot::floodFill(int goal_i, int goal_j)
{

    if (!isInsideGrid(goal_i, goal_j))
        return;

    planGrid = grid;
    inflateObstacles();

    if (planGrid[goal_i][goal_j] == 1)
        return;

    std::queue<Cell> q;

    planGrid[goal_i][goal_j] = 2;
    q.push({goal_i, goal_j});

    while (!q.empty())
    {
        Cell c = q.front();
        q.pop();

        for (int k = 0; k < 8; k++)
        {
            int ni = c.i + di8[k];
            int nj = c.j + dj8[k];

            if (!isInsideGrid(ni, nj))
                continue;

            if (abs(di8[k]) == 1 && abs(dj8[k]) == 1)
            {
                if (planGrid[c.i + di8[k]][c.j] == 1 ||
                    planGrid[c.i][c.j + dj8[k]] == 1)
                    continue;
            }

            if (planGrid[ni][nj] == 0)
            {
                planGrid[ni][nj] = planGrid[c.i][c.j] + 1;
                q.push({ni, nj});
            }
        }
    }
}

std::vector<robot::Cell> robot::extractPath(int start_i, int start_j)
{
    std::vector<Cell> result;

    if (planGrid[start_i][start_j] < 2)
        return result;

    if (!isInsideGrid(start_i, start_j))
        return result;

    int ci = start_i;
    int cj = start_j;

    result.push_back({ci, cj});

    while (planGrid[ci][cj] > 2)
    {
        int best_i = ci;
        int best_j = cj;
        int best_val = planGrid[ci][cj];

        if (!isInsideGrid(ci, cj))
            break;

        for (int k = 0; k < 8; k++)
        {
            int ni = ci + di8[k];
            int nj = cj + dj8[k];

            if (!isInsideGrid(ni, nj))
                continue;

            if (planGrid[ni][nj] >= 2 &&
                planGrid[ni][nj] < best_val)
            {
                best_val = planGrid[ni][nj];
                best_i = ni;
                best_j = nj;
            }
        }

        if (best_i == ci && best_j == cj)
            break;

        ci = best_i;
        cj = best_j;
        result.push_back({ci, cj});
    }

    return result;
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

            double ratio = double(time - p1.timestamp) / double(p2.timestamp - p1.timestamp);

            Position result;

            result.x = p1.x + ratio * (p2.x - p1.x);
            result.y = p1.y + ratio * (p2.y - p1.y);

            double dfi = normalizeAngle(p2.fi - p1.fi);
            result.fi = normalizeAngle(p1.fi + ratio * dfi);

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

    int robot_i = (x / resolution) + gridWidth / 2;
    int robot_j = (y / resolution) + gridHeight / 2;

    for (const auto& point : laserData)
    {
        if(positionHistory.size() < 2)
            continue;
        Position pos = interpolatePosition(point.timestamp);

        double angleDeg = point.scanAngle;
        double distance = point.scanDistance;
        angleDeg = -angleDeg;

        double distance_m = distance / 1000.0;
        double angleRad = angleDeg * M_PI / 180.0;

        if (distance_m <= minDist || distance_m > maxDist ||
            (distance_m >= 0.6 && distance_m <= 0.7))
        {
            continue;
        }

        double x_glob = pos.x + distance_m * cos(pos.fi + angleRad);
        double y_glob = pos.y + distance_m * sin(pos.fi + angleRad);

        int i = (x_glob / resolution) + gridWidth / 2;
        int j = (y_glob / resolution) + gridHeight / 2;

        if(mappingEnabled)
        {
            if(i >= 0 && i < gridWidth && j >= 0 && j < gridHeight)
            {
                drawLine(robot_i, robot_j, i, j);

                tempGrid[i][j]++;

                if(tempGrid[i][j] >= 25)
                {
                    grid[i][j] = 1;
                }
            }
        }

        while (angleDeg >= 180.0) angleDeg -= 360.0;
        while (angleDeg < -180.0) angleDeg += 360.0;

    }


    if(localizationEnabled)
    {
        computeDistanceField();
        measurementUpdate();
        resampleParticles();
        motionUpdate();
        estimatePose();
    }


    emit publishLidar(copyOfLaserData);
    // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    return 0;
}

void robot::setLocalizationEnabled(bool enabled)
{
    localizationEnabled = enabled;

    if(enabled)
    {
        initializeParticles();
    }
}

void robot::estimatePose()
{
    double sumX = 0.0;
    double sumY = 0.0;

    double sumSin = 0.0;
    double sumCos = 0.0;

    for(const auto &p : particles)
    {
        sumX += p.weight * p.x;
        sumY += p.weight * p.y;

        sumSin += p.weight * sin(p.fi);
        sumCos += p.weight * cos(p.fi);
    }

    if(!particles.empty())
    {
        estimatedX = sumX;
        estimatedY = sumY;
    }

    estimatedFi = atan2(sumSin, sumCos);
}

void robot::resampleParticles()
{
    std::vector<Particle> newParticles;
    std::vector<double> cumulative;

    double sum = 0.0;

    for(const auto &p : particles)
    {
        sum += p.weight;
        cumulative.push_back(sum);
    }

    double random_ratio = 0.05;

    for(int i = 0; i < particleCount; i++)
    {
        if(((double)rand() / RAND_MAX) < random_ratio)
        {
            Particle p;

            while(true)
            {
                int ii = rand() % gridWidth;
                int jj = rand() % gridHeight;

                if(grid[ii][jj] == 0)
                {
                    p.x = (ii - gridWidth / 2) * resolution;
                    p.y = (jj - gridHeight / 2) * resolution;
                    p.fi = ((double)rand() / RAND_MAX) * 2.0 * M_PI;
                    p.weight = 1.0 / particleCount;
                    break;
                }
            }

            newParticles.push_back(p);
        }
        else
        {
            double r = ((double)rand() / RAND_MAX) * sum;

            for(int j = 0; j < particles.size(); j++)
            {
                if(r <= cumulative[j])
                {
                    newParticles.push_back(particles[j]);
                    break;
                }
            }
        }
    }

    particles = newParticles;

    for(auto &p : particles)
    {
        p.weight = 1.0 / particleCount;
    }
}

void robot::measurementUpdate()
{

    if(distanceField.empty() || distanceField[0].empty())
    {
        return;
    }

    double weightSum = 0.0;

    for(auto &p : particles)
    {
        double error = 0.0;

        for(int idx = 0; idx < copyOfLaserData.size(); idx += 5)
        {
            const auto &point = copyOfLaserData[idx];

            double distance = point.scanDistance / 1000.0;

            if(distance <= minDist ||
                distance > maxDist ||
                (distance >= 0.6 && distance <= 0.7))
            {
                continue;
            }

            double angle = -point.scanAngle * M_PI / 180.0;

            double hitX = p.x + distance * cos(p.fi + angle);
            double hitY = p.y + distance * sin(p.fi + angle);

            int i = hitX / resolution + gridWidth / 2;
            int j = hitY / resolution + gridHeight / 2;

            if(i < 0 || i >= gridWidth ||
                j < 0 || j >= gridHeight)
            {
                error += 25.0;
                continue;
            }

            double dist = distanceField[i][j];

            error += dist * dist;
        }

        if(error > 1000) error = 1000;
        p.weight = 1.0 / (1.0 + error);

        weightSum += p.weight;
    }

    if(weightSum == 0.0)
    {
        for(auto &p : particles)
            p.weight = 1.0 / particleCount;
    }
    else
    {
        for(auto &p : particles)
            p.weight /= weightSum;
    }
}
void robot::drawLine(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    while (true)
    {
        if (x0 >= 0 && x0 < gridWidth &&
            y0 >= 0 && y0 < gridHeight)
        {

            if (grid[x0][y0] == 1)
            {
                if (tempGrid[x0][y0] < 10)
                {
                    tempGrid[x0][y0]--;

                    if (tempGrid[x0][y0] < 3)
                    {
                        grid[x0][y0] = 0;
                    }
                }
            }
        }

        if (x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx)  { err += dx; y0 += sy; }
    }
}

void robot::saveMap(const std::string& filename)
{
    std::ofstream file(filename);

    if(!file.is_open())
        return;

    file << gridWidth << " " << gridHeight << std::endl;

    for(int i = 0; i < gridWidth; i++)
    {
        for(int j = 0; j < gridHeight; j++)
        {
            file << grid[i][j] << " ";
        }
        file << std::endl;
    }

    file.close();

    qDebug() << "MAP SAVED";
}

void robot::loadMap(const std::string& filename)
{
    std::ifstream file(filename);

    if(!file.is_open())
        return;

    int w,h;

    file >> w >> h;

    if(w != gridWidth || h != gridHeight)
    {
        qDebug() << "WRONG MAP SIZE";
        return;
    }

    for(int i = 0; i < gridWidth; i++)
    {
        for(int j = 0; j < gridHeight; j++)
        {
            file >> grid[i][j];
        }
    }

    file.close();

    mappingEnabled = false;
    computeDistanceField();
    qDebug() << "MAP LOADED";
}

const std::vector<std::vector<int>>& robot::getGrid() const
{
    return grid;
}

const std::vector<robot::Cell>& robot::getPath() const
{
    return path;
}

const std::vector<robot::Cell>& robot::getMainPoints() const
{
    return mainpoints;
}
double robot::getX() const { return x; }
double robot::getY() const { return y; }
double robot::getFi() const { return fi; }

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
