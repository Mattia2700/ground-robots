#ifndef LIDARTRACKING_H
#define LIDARTRACKING_H

#include "MapManager.hh"
#include "Publisher.hpp"
#include "json.hpp"
//#include "utils.h"
#include <string>
#include <vector>
#include <fstream>

#include <unordered_map>
#include <fstream>

#include "timer.hpp"
#include <chrono>


class LidarTracking
{
public:
  LidarTracking();
  ~LidarTracking();

//  struct Category{
//    cv::Vec4i leader;
//    std::vector<cv::Vec4i> lines;
//    double m;
//    double q;
//  };

  struct ObjectItem{
    int id;
    double x;
    double y;
    double theta = 0.0;
    int probability;
    std::string type = "N.D.";
    
    void getJson(nlohmann::json & j){
      j["id"] = id;
      j["x"] = x;
      j["y"] = y;
      j["theta"] = theta;
      j["type"] = type;
      j["probability"] = probability;
    }

    void fromJson(const nlohmann::json & j){
      id = j.at("id");
      x = j.at("x");
      y = j.at("y");
      theta = j.at("theta");
      probability = j.at("probability");
      type = j.at("type").get<std::string>();

    }
  };

 struct ObjectItemFilter{
    std::vector<int> id;
    std::vector<std::vector<double>> x;
    std::vector<std::vector<double>> y;
    std::vector<double> theta ;
    std::vector<double> vx ;
    std::vector<double> vy ;
    std::vector<int>    count;
    std::vector<bool>   flag ;
    std::vector<bool>   isupdated;
    std::vector<bool>   todelete;
};

  struct ObjectsFilter { 
    int id;
    std::vector<double> x;
    std::vector<double> y;
    double theta;
    double vx;
    double vy;
    int count;
    bool isupdated;
    bool todelete;
  };

  std::vector<ObjectItem> objectsTrackedVector;
  std::vector<ObjectsFilter> objFiltVec;

  struct Object
  {
    int id;
    cv::Point2f centroid;
    int cnt;
    int obj_id;
    int probability;
  };

  std::unordered_map<int, Object> tracks;

  const double MIN_DST = 10; //10px -> 10*0.04 = 40cm
  int NXT_ID = 1;
  int NXT_OBJ_ID = 1;

//  void startTracking();
//  void stopTracking();

//  bool showImage = false;
//  bool ovetti = false;

  int trackingID = -1;
//  int last_id_tracked = -1;

private:

//  void poseSubscriber(const char *topic, const char *buf, size_t size, void *data);

//  void lidarSubscriber(const char *topic, const char *buf, size_t size, void *data);
//  void realsenseSubscriber(const char *topic, const char *buf, size_t size, void *data);
//  void processMessage(const nlohmann::json & j_in, nlohmann::json & j_out);

  std::mutex mmMutex;
  std::unique_ptr<MapManager> mm;
  cv::Mat map;

  bool loc_init;
  //bool lidar_init;

//  utils::Pose pose;
//  std::mutex poseMutex;
//  std::mutex trackerMutex;
//  std::vector<std::pair<double,double>> scanValues;

  //std::vector<Category> categoryVector;

  //ZMQCommon::Subscriber subLOC, subLIDAR, subREALSENSE;
  std::unique_ptr<Common::Publisher> trackingPublisher;
  //std::unique_ptr<Common::Replier> trackingServer;

  std::unique_ptr<Timer> timerMapProcessing, timerMapUpdate;
  //RobotStatus::LidarData lastRealsense, lastLidar;

  void callback_timerMapProcessing();
  void callback_timerMapUpdate();

};

#endif // LIDARTRACKING_H
