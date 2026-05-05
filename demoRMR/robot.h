#ifndef ROBOT_H
#define ROBOT_H

#include "librobot/librobot.h"
#include <QObject>
#include <QWidget>
#include <vector>
#include <string>

#ifndef DISABLE_OPENCV
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
Q_DECLARE_METATYPE(cv::Mat)
#endif

#ifndef DISABLE_SKELETON
Q_DECLARE_METATYPE(skeleton)
#endif

Q_DECLARE_METATYPE(std::vector<LaserData>)

class robot : public QObject {
    Q_OBJECT

public:
    explicit robot(QObject *parent = nullptr);

    void initAndStartRobot(std::string ipaddress);
    void setTarget(double x, double y);

    // mapping
    const std::vector<std::vector<int>>& getGrid() const;
    struct Position {
        double x;
        double y;
        double fi;
        uint32_t timestamp;
    };
    Position interpolatePosition(uint32_t time);

    // nastavuje rýchlosti
    void setSpeedVal(double forw, double rots);
    // posiela rýchlosti do robota
    void setSpeed(double forw, double rots);

signals:
    void publishPosition(double x, double y, double z);
    void publishLidar(const std::vector<LaserData> &lidata);
#ifndef DISABLE_OPENCV
    void publishCamera(const cv::Mat &camframe);
#endif
#ifndef DISABLE_SKELETON
    void publishSkeleton(const skeleton &skeledata);
#endif

private:
    /// --- Odometry ---
    double x;
    double y;
    double fi;
    double fi_old;
    int prevEncoderLeft;
    int prevEncoderRight;
    double v_actual;
    double w_actual;

    double gyroOffset;
    bool gyroInitialized;

    bool rotateMode;
    bool hasTarget;
    double targetX;
    double targetY;

    double forwardspeed;   // mm/s
    double rotationspeed;  // omega/s

    /// --- Lidar spracovanie ---
    bool lidarReady;
    int sectorCount;
    double sectorWidthDeg;
    double obstacleMaxDist;

    std::vector<double> lastDirections;
    int memorySize = 5;
    double currentObstacleDist;
    double prevFi; // Stores global orientation (fi) from the previous robot telemetry update
    std::vector<double> prevSectors;
    std::vector<double> sectors;
    std::vector<int> binarySectors;
    std::vector<int> maskedSectors;
    std::vector<std::pair<int, int>> freeGaps;
    std::vector<double> candidateDirections;
    double bestDirectionDeg;
    double robotRadius;
    double safetyMargin;

    double prevX = 0, prevY = 0;
    int stuckCounter = 0;
    /// --- Mapping ---
    std::vector<std::vector<int>> tempGrid;
    std::vector<std::vector<int>> grid;
    std::vector<Position> positionHistory;
    int gridWidth;
    int gridHeight;
    double resolution;
    double minDist;
    double maxDist;

    void drawLine(int x0, int y0, int x1, int y1);
    double normalizeAngle(double a) const;

    /// --- Callbacky ---
    int processThisLidar(const std::vector<LaserData> &laserData);
    int processThisRobot(const TKobukiData &robotdata);
#ifndef DISABLE_OPENCV
    int processThisCamera(cv::Mat cameraData);
#endif
#ifndef DISABLE_SKELETON
    int processThisSkeleton(skeleton skeledata);
#endif

    /// --- Pomocné štruktúry ---
    std::vector<LaserData> copyOfLaserData;

#ifndef DISABLE_OPENCV
    cv::Mat frame[3];
    bool useCamera1;
    int actIndex;
#endif

#ifndef DISABLE_SKELETON
    int updateSkeletonPicture;
    skeleton skeleJoints;
#endif

    libRobot robotCom;
    int datacounter;
    int useDirectCommands;
};

#endif // ROBOT_H
