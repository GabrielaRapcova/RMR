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

  //planning
  struct Cell
  {
      int i;
      int j;
  };
  void setGoal(double X, double Y);
  void planToGoal();
  void saveMap(const std::string& filename);
  void loadMap(const std::string& filename);

  const std::vector<Cell>& getPath() const;
  const std::vector<Cell>& getMainPoints() const;
  int getCurrentMainPoint() const { return currentMainPoint;}

  double getResolution() const { return resolution; }
  int getGridWidth() const { return gridWidth; }
  int getGridHeight() const { return gridHeight; }

  double getX() const;
  double getY() const;
  double getFi() const;

  Position interpolatePosition(uint32_t time);

  //monte carlo
  void setLocalizationEnabled(bool enabled);

  struct Particle
  {
      double x;
      double y;
      double fi;

      double weight;
  };
  const std::vector<Particle>& getParticles() const;
  const std::vector<std::vector<double>>& getDistanceField() const;
  double getEstimatedX() const;
  double getEstimatedY() const;
  double getEstimatedFi() const;

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
  double forwardSpeed;
  double rotationSpeed;

  // grid for mapping
  std::vector<std::vector<int>> tempGrid;
  std::vector<std::vector<int>> grid;
  std::vector<Position> positionHistory;

  // grid for planning
  std::vector<std::vector<int>> planGrid;
  std::vector<Cell> path;
  std::vector<Cell> mainpoints;

  int gridWidth;
  int gridHeight;
  double resolution;

  bool wallFollowing;
  bool lidarReady;
  int sectorCount;
  double sectorWidthDeg;
  double obstacleMaxDist;

  double wallFollowDirection;
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


  double minDist;
  double maxDist;
  void drawLine(int x0,int y0, int x1,int y1);
  double normalizeAngle(double a) const;


  //planning
  double goal_X;
  double goal_Y;
  int currentMainPoint;
  bool followingPath;
  bool mappingEnabled;

  void floodFill(int goal_i, int goal_j);
  std::vector<Cell> extractPath(int start_i, int start_j);
  void inflateObstacles();

  const int di8[8] = { 1, -1,  0,  0,  1,  1, -1, -1 };
  const int dj8[8] = { 0,  0,  1, -1,  1, -1,  1, -1 };
  bool isInsideGrid(int i, int j) const;

  //monte carlo
  bool localizationEnabled;
  double a1;
  double a2;
  double a3;
  double a4;
  std::vector<Particle> particles;
  int particleCount;
  void initializeParticles();
  double prevOdomX;
  double prevOdomY;
  double prevOdomFi;
  void motionUpdate();
  double sampleNormal(double variance);
  std::vector<std::vector<double>> distanceField;
  void computeDistanceField();
  void measurementUpdate();
  void resampleParticles();
  double estimatedX;
  double estimatedY;
  double estimatedFi;
  void estimatePose();
  /// toto su callbacky co sa sa volaju s novymi datami
  int processThisLidar(const std::vector<LaserData> &laserData);

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
