#ifndef ROBOT_H
#define ROBOT_H
#include "librobot/librobot.h"
#include <QObject>
#include <QWidget>
#include <vector>

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

  //mapping
  const std::vector<std::vector<int>>& getGrid() const;
  struct Position
  {
      double x;
      double y;
      double fi;
      uint32_t timestamp;
  };
  Position interpolatePosition(uint32_t time);

  // tato funkcia len nastavuje hodnoty.. posielaju sa v callbacku(dobre, kvoli
  // asynchronnosti a zabezpeceniu,ze sa poslu len raz pri viacero prepisoch
  // vramci callu)
  void setSpeedVal(double forw, double rots);
  // tato funkcia fyzicky posiela hodnoty do robota
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
  std::vector<int> sectors;
  int sectorCount;
  double sectorWidthDeg;
  double obstacleMaxDist;

  // odometry
  double x;
  double y;
  double fi;
  double fi_old;
  int prevEncoderLeft;
  int prevEncoderRight;
  double v_actual;

  double gyroOffset;
  bool gyroInitialized;
  bool rotateMode;
  double targetX;
  double targetY;
  bool hasTarget;
  double w_actual;
  ///-----------------------------
  /// toto su rychlosti ktore sa nastavuju setSpeedVal a posielaju v
  /// processThisRobot
  double forwardspeed;  // mm/s
  double rotationspeed; // omega/s

  // grid for mapping
  std::vector<std::vector<int>> tempGrid;
  std::vector<std::vector<int>> grid;
  std::vector<Position> positionHistory;

  int gridWidth;
  int gridHeight;
  double resolution;

  double minDist;
  double maxDist;
  void drawLine(int x0,int y0, int x1,int y1);

  /// toto su callbacky co sa sa volaju s novymi datami
  int processThisLidar(const std::vector<LaserData> &laserData);
  std::vector<std::pair<int,int>> freeGaps;
  std::vector<double> candidateDirections;
  double bestDirectionDeg;
  int processThisRobot(const TKobukiData &robotdata);
#ifndef DISABLE_OPENCV
  int processThisCamera(cv::Mat cameraData);
#endif

  /// pomocne strukutry aby ste si trosku nerobili race conditions
  std::vector<LaserData> copyOfLaserData;
#ifndef DISABLE_OPENCV
  cv::Mat frame[3];
#endif
  /// classa ktora riesi komunikaciu s robotom
  libRobot robotCom;

  /// pomocne premenne... moc nerieste naco su
  int datacounter;
#ifndef DISABLE_OPENCV
  bool useCamera1;
  int actIndex;
#endif

#ifndef DISABLE_SKELETON
  int processThisSkeleton(skeleton skeledata);
  int updateSkeletonPicture;
  skeleton skeleJoints;
#endif
  int useDirectCommands;
};

#endif // ROBOT_H
