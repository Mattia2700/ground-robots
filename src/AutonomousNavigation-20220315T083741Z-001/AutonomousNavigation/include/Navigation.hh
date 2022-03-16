#ifndef REPLANNING_HH
#define REPLANNING_HH

#include "MapManager.hh"
#include "utils.hpp"
#include "hardwareglobalinterface.h"
#include <mutex>
#include <atomic>
#include <pathpoint.h>
#include <path.h>

#include "task.h"
#include "timeperf.hpp"
#include "timer.hpp"
#include "ClothoidList.hh"
#include "personitem.h"
#include "voronoi.h"
#include "fm_planner/planner.hpp"
#include "Publisher.hpp"
#include "Subscriber.hpp"
#include "Prediction.hpp"
class Navigation: public Task
{
public:
  Navigation(std::string map, State2d const & goal);
  Navigation(std::string map, std::vector<State2d> const & waypoints);
  Navigation(PosedMap const & map, State2d const & goal);
  Navigation(PosedMap const & map, std::vector<State2d> const & waypoints);
  virtual ~Navigation();

  virtual void start() override;
  virtual void stop() override;
  virtual void pause() override;
  virtual void resume() override;
  virtual cv::Mat getImage() override;

  
//  struct People {
//     int id;
//     std::vector<double> x;
//     std::vector<double> y;
//     std::vector<double> vx;
//     std::vector<double> vy;
//     std::vector<std::vector<double>> Xpred;
//     bool isupdated = false;
//  };

  std::vector<People> people;


  //void start(bool const & terminating);
  //void stop();
  //void setPath(std::vector<State2d> const & refpath);
  //void setGoal(State2d const & goal);


//  void setRunning(bool running);
//  bool isRunning();
  bool hasFailed();
  void setSpray(bool spray);

private:
  void lidarCallback();
  void computeControl();
  void computeControl_pathfollowing();
   void publishImg();

  void init();

  std::unique_ptr<MapManager> mm;

  std::mutex mmMutex;
  std::unique_ptr<Timer> lidarTimer;
  std::unique_ptr<Timer> controlTimer;
  std::unique_ptr<Timer> imgPubTimer;
  std::unique_ptr<Common::Publisher> imgPublisher;
  std::unique_ptr<ZMQCommon::Subscriber> neighboursSubscriber1, neighboursSubscriber2;
  std::unique_ptr<Common::Publisher> predictionPublisher;
  //std::unique_ptr<Common::Publisher> peopleNeighPublisher;

private:
  std::vector<State2d> refpath;

  std::mutex pathMutex;
  std::atomic<bool> running { false };
  std::atomic<bool> done { false };
  std::atomic<bool> controlRunning { false };
  std::atomic<bool> eop { false };
  std::atomic<bool> emergencyBreak { false };
  std::atomic<bool> failed { false };

  bool paused = false;

  bool newPath = true;
  PathPoint frenetPoint;
  DynamicFrenetPoint dfp;
  bool validGoal = false;
  State2d goal;
  Path currPath;

  std::vector<State2d> waypoints;

  std::mutex neighboursMtx;
  std::unordered_map<int, Neighbours> neighbours;

  G2lib::ClothoidList currPathCL;
  TimePerf controlTime;
  int i_last = 0;
  VoronoiParams vparams;
  ControlParams cparams;
  
  bool cadmap = true;
  std::string mapfile;
  PosedMap map;

  double past_controls[2] = {0,0};
  //std::vector<PersonItem> peopleVector;

   void setImage(cv::Mat const & img);
  void setPath(const std::vector<DPSmoothing::Curve> &path);
  bool generateReferencePath();
  bool generateLocalPath();
  bool safePath();
  bool safePath(G2lib::ClothoidList const & path);
  std::vector<Neighbours> getNeighbours();
  std::mutex mtx;
  cv::Mat img;
  bool spray = false;

  TimePerf tp;
  double t0 = 0;
  bool spraying = false;


};

#endif
