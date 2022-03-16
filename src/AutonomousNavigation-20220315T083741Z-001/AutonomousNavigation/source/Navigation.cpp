#include "Navigation.hh" 
#include "json.hpp" 
#include <unistd.h> 
#include "voronoi.h" 
#include "Prediction.hpp"
#include "zmq_services.hpp"
#include "taskexecutor.h"

#include "personitem.h" 
#include <iostream>
#include <fstream>
#include <chrono>
bool IMG = true;
static const int REPLANNING_PERIOD_MS = 200;//500 
static const double REPLANNING_LOOKAHEAD = 4;//4 
static const double SAFETY_DISTANCE = 1.5; 
static const int MAX_FAILURES = 50;
 std::ofstream File4("/home/eecs/log/CONTROL_INPUTS.txt"); 
 std::ofstream File5("/home/eecs/log/CURR_PATH.txt"); 
 std::ofstream File6("/home/eecs/log/REF_PATH.txt");
 std::ofstream File7("/home/eecs/log/PREDICTIONS.txt");

//using utils::ScanData; 
using G2lib::ClothoidList;



inline static void drawCL(cv::Mat & img, MapManager & mm, ClothoidList const & cl, cv::Vec3b color = {255,0,0}) {
  // draw spline
  int ncuts = static_cast<int>(std::ceil(cl.length()/mm.getResolution()));
  double step = cl.length()/ncuts;
  for (int i=0; i<=ncuts; ++i)
  {
    //try {
    double x, y;
    cl.eval(i*step, x, y);

    int r, c;
    bool inside = mm.getMapCoordinatesAbs(x, y, r, c);
    if (inside)
    {
      img.at<cv::Vec3b>(r,c) = color;
    }
    //} catch (std::exception ex)
    {
     //std::cerr << ex.what() << std::endl;
     //std::cerr << "ERROR: " << cl.length() << " " << i*step << std::endl;
    }
  }
}


inline static void drawCL(cv::Mat & img, MapManager & mm, Path const & path, cv::Vec3b color = {255,0,0}) {
  vector<PathPoint> const & pts = path.points;

  for (int i=0; i<pts.size(); ++i)
  {
    //try {
    double x = pts[i].x, y = pts[i].y;

    int r, c;
    bool inside = mm.getMapCoordinatesAbs(x, y, r, c);
    if (inside)
    {
      img.at<cv::Vec3b>(r,c) = color;
    }
    //} catch (std::exception ex)
    {
     //std::cerr << ex.what() << std::endl;
     //std::cerr << "ERROR: " << cl.length() << " " << i*step << std::endl;
    }
  }
}


inline static void drawCL(cv::Mat & img, MapManager & mm, vector<State2d> const & pts, cv::Vec3b color = {255,0,0}) {
  for (int i=0; i<pts.size(); ++i)
  {
    //try {
    double x = pts[i].x, y = pts[i].y;

    int r, c;
    bool inside = mm.getMapCoordinatesAbs(x, y, r, c);
    if (inside)
    {
      img.at<cv::Vec3b>(r,c) = color;
    }
    //} catch (std::exception ex)
    {
     //std::cerr << ex.what() << std::endl;
     //std::cerr << "ERROR: " << cl.length() << " " << i*step << std::endl;
    }
  }
}


Navigation::Navigation(std::string map, std::vector<State2d> const & waypoints):
  cadmap(true),
  mapfile(map),
  waypoints(waypoints) {
  init();
}


Navigation::Navigation(std::string map, State2d const & goal):
  cadmap(true),
  mapfile(map),
  goal(goal),
  validGoal(true) {
  init();
}

Navigation::Navigation(PosedMap const & map, std::vector<State2d> const & waypoints):
  cadmap(false),
  map(map),
  waypoints(waypoints) {
  init();
}


Navigation::Navigation(PosedMap const & map, State2d const & goal):
  cadmap(false),
  map(map),
  goal(goal),
  validGoal(true) {
  init();
}



void Navigation::init() {
  if (cadmap)
    mm = std::make_unique<MapManager>(100, 0.05, MapManager::RobotGeometry{0.5}, getObstacles(mapfile));//100,0.05
  else
    mm = std::make_unique<MapManager>(100, 0.05, MapManager::RobotGeometry{0.5}, map);

  lidarTimer = std::make_unique<Timer>();
  //connect(lidarTimer.get(), SIGNAL(timeout()), this, SLOT(lidarCallback()));
  lidarTimer->registerCallback([this](){ this->lidarCallback(); });
  lidarTimer->start(100);

  controlTimer = std::make_unique<Timer>();
  //connect(controlTimer.get(), SIGNAL(timeout()), this, SLOT(computeControl()));
  controlTimer->registerCallback([this](){ this->computeControl(); });
  controlTimer->start(50);

  auto neigh_callback = [&](const char *topic, const char *buf, size_t size, void *data) {
    using nlohmann::json;
    try {
      json data = json::parse(std::string(buf, size));
      int id = data["id"];
      double x = data["x"];
      double y = data["y"];
      double theta = data["theta"];
      
      std::vector<People> peopleNeigh;
      try {
         nlohmann::json j_data = data["data"]["peopleNeigh"];
         //std::cout<<data["data"]<<std::endl;

         for (nlohmann::json::iterator it = j_data.begin(); it != j_data.end(); ++it) {
            People partypeople;
            partypeople.x = (*it).at("x").get<std::vector<double>>(); 
            partypeople.y =  (*it).at("y").get<std::vector<double>>();
            partypeople.vx = (*it).at("vx").get<std::vector<double>>(); 
            partypeople.vy = (*it).at("vy").get<std::vector<double>>();
            partypeople.id = (*it).at("id");
            peopleNeigh.push_back(partypeople);
         }
      } catch (std::exception const & ex) {
        //std::cerr << "WRONG NEIGH FORMAT" << std::endl;
      }

      std::vector<Point2d> poseH;
      std::vector<Point2d> vH;
      try {
       poseH = data["data"]["poseH"].get<std::vector<Point2d>>();
      } catch (std::exception const & ex) {
        //std::cerr << "WRONG POSEH FORMAT" << std::endl;
      }
      try {
       vH = data["data"]["vH"].get<std::vector<Point2d>>();
      } catch (std::exception const & ex) {
        //std::cerr << "WRONG VH FORMAT" << std::endl;
      }
 
      std::vector<Point2d> comm;
      try {
       comm = data["data"]["comm"].get<std::vector<Point2d>>();
      } catch (std::exception const & ex) {
        //std::cerr << "WRONG COMM FORMAT" << std::endl;
      }

      std::unordered_map<int, double> ncost;
      try {
        ncost = data["data"]["ncost"].get<std::unordered_map<int,double>>();
      } catch (std::exception const & ex) {
       // std::cerr << "WRONG NCOST FORMAT" << std::endl;
      }
      //std::cout<<peopleNeigh.size()<<std::endl;     
      Neighbours neigh;

      //neigh.time = data["time"];
      neigh.id = id;
      neigh.pose = {x, y};
      neigh.poseH = poseH;
      neigh.vH = vH;
      
      neigh.peopleNeigh = peopleNeigh;

      neigh.comm = comm;
      neigh.ncost = ncost;

      {
        std::unique_lock<std::mutex> lock(neighboursMtx);
        neighbours[id] = neigh;
      }
      //std::cout << "SEEN AGENT " << neigh.id << " " << x << " " << y << std::endl;
      //std::cout << "COMM_SIZE: " << comm.size() << std::endl;
    } catch (std::exception const & ex) {
      std::cerr << "[WARN] Exception parsing neighbour data" << std::endl;
      std::cerr << ex.what() << std::endl;
    }
  };

  neighboursSubscriber1 = std::make_unique<ZMQCommon::Subscriber>();
  neighboursSubscriber1->register_callback(neigh_callback);
  neighboursSubscriber1->start("tcp://192.168.1.1:1992", "NSTAT");

  neighboursSubscriber2 = std::make_unique<ZMQCommon::Subscriber>();
  neighboursSubscriber2->register_callback(neigh_callback);
  neighboursSubscriber2->start("tcp://192.168.1.3:1992", "NSTAT");
  
}

Navigation::~Navigation() {
  lidarTimer->stop();
  controlTimer->stop();
  imgPubTimer->stop();
  neighboursSubscriber1->stop();
  //neighboursSubscriber2->stop();
  lidarTimer.reset();
  controlTimer.reset();
  // imgPubTimer.reset();
  //imgPublisher.reset();
}

 void
 Navigation::publishImg()
 {
   cv::Mat img = getImage();
   if (img.empty()) return;
   std::vector<uint8_t> buff; // buffer for coding
   std::vector<int> param(2);
   param[0] = cv::IMWRITE_JPEG_QUALITY;
   param[1] = 95; //default(95) 0-100
   cv::imencode(".jpg", img, buff, param);
   imgPublisher->send("IMG", (char*)(buff.data()), buff.size());
 }

void Navigation::lidarCallback() {
  RobotStatus::LidarData ldata;
  HardwareGlobalInterface::getInstance().getFrontLidarData(ldata);
  RobotStatus::LocalizationData pose;
#ifdef LOC_REALSENSE
  HardwareGlobalInterface::getInstance().getLocalizationDataRealSense(pose);
#else
  HardwareGlobalInterface::getInstance().getLocalizationDataKarto(pose);
#endif
  RobotStatus::LidarData rdata;
  HardwareGlobalInterface::getInstance().getRealSenseLidarData(rdata);

  HardwareParameters params;
  rdata.setMountingPosition(params.cameraOffsetX, params.cameraOffsetY);

  ScanData scan;
  scan.data = {ldata, rdata};
  //scan.data = {rdata};
  scan.pose = {pose.x, pose.y, pose.theta};

  {
    std::unique_lock<std::mutex> lock(mmMutex);
    mm->update(scan);
  }
}



 void Navigation::setImage(const cv::Mat &img)
 {
   std::unique_lock<std::mutex> lck(mtx);
   this->img = img;
 }


bool Navigation::generateReferencePath() {
  PosedMap const & gm = mm->getGlobalMap();
  Map map;
  map.x0 = gm.pose.x;
  map.y0 = gm.pose.y;
  map.resolution = mm->getResolution();
  map.img = gm.map;



  cv::Mat storeImg;
//  cv::cvtColor(map.img, storeImg, CV_GRAY2BGR);
  //cv::imwrite("globMap.jpg", storeImg);

  std::vector<State2d> wp;

  if (validGoal) {
    RobotStatus::LocalizationData locData;
#ifdef LOC_REALSENSE
    HardwareGlobalInterface::getInstance().getLocalizationDataRealSense(locData);
#else
    HardwareGlobalInterface::getInstance().getLocalizationDataKarto(locData);
#endif
    State2d start;
    start = {locData.x, locData.y, locData.theta};
    wp.push_back(start);
    wp.push_back(goal);
  }
  else {
    if (waypoints.size()<2) {
      std::cerr << "No goal and no waypoints. Cannot generate reference path..." << std::endl;
      return false;
    }
    wp = waypoints;
  }


  i_last = 0;
  refpath = {};

  FMPlanner fmplanner;
  fmplanner.setMap(&map);


  for (int i=1; i<wp.size(); ++i) {
    State2d start = wp[i-1];
    State2d goal  = wp[i];
    std::vector<State2d> currPath;
    if (!fmplanner.planRaw(start, goal, currPath))
    {
      return false;
    }
    //std::cerr << "SIZE: " << currPath.size() << std::endl;
    refpath.insert(refpath.end(), currPath.begin(), currPath.end());
  }
if (IMG == true){

 for (int i=0; i<refpath.size(); ++i) {
   int rp, cp;
   map.world2map(refpath[i].x, refpath[i].y, rp, cp);
   cv::circle(storeImg, cv::Point(cp,rp), 5, cv::Scalar(255,0,0), -1);
 }
}  // cv::imwrite("globPlan.jpg", storeImg);

  return true;
}

bool Navigation::generateLocalPath() {
  const double WAYPOINT_MIN_DST = 4.0;

  if (refpath.empty()) {
    std::cerr << "Empty reference path. Cannot generate local path..." << std::endl;
    return false;
  }

  Map map;
  {
    std::unique_lock<std::mutex> lock(mmMutex);
    map.img = mm->generateImage();
    map.resolution = mm->getResolution();
    mm->getWorldCoordinatesAbs(0, 0, map.x0, map.y0);
    //std::cout << map.resolution <<"  "<<map.x0<<" "<<map.y0<<std::endl;
  }
  cv::cvtColor(map.img, map.img, CV_BGR2GRAY);
  // clear other robots from costmap
 std::vector<Neighbours> NeighboursPP = getNeighbours();
 
 for (Neighbours const & n : NeighboursPP) {
    double xn = n.pose.x;
    double yn = n.pose.y;
    int rn, cn;
    map.world2map(xn, yn, rn, cn);
    int radius = std::ceil(0.5/map.resolution);
    cv::circle(map.img, cv::Point(cn, rn), radius, cv::Scalar(0), -1);
  }

   RobotStatus::LocalizationData locData;
#ifdef LOC_REALSENSE
  HardwareGlobalInterface::getInstance().getLocalizationDataRealSense(locData);
#else
  HardwareGlobalInterface::getInstance().getLocalizationDataKarto(locData);
#endif
  State2d start = {locData.x, locData.y, locData.theta};
  std::vector<PersonItem> trackedPeople;
  HardwareGlobalInterface::getInstance().getTrackingData(trackedPeople);

  // clear dynamic obstacles from costmap
 
// for (auto const & p: trackedPeople) {
 
//// for (int i=0; i<people.size(); ++i) {
//// if (people[i].x.size() >= 10) {
//    int rn,cn; 
//    double xt = p.x + locData.x ;//people[i].Xpred[0][9] + locData.x;
//    double yt = p.y + locData.y ;//people[i].Xpred[1][9] + locData.y;
//    map.world2map(xt, yt, rn, cn);
//    int radius = std::ceil(0.3/map.resolution);
//    cv::circle(map.img, cv::Point(cn, rn), radius, cv::Scalar(255), -1);
//  }


  int i_closest = -1;
  double d_closest = std::numeric_limits<double>::infinity();
  for (int i=i_last; i<=std::min((int)refpath.size()-1, i_last); ++i) {
    double d_cur = std::hypot(refpath[i].x-locData.x, refpath[i].y-locData.y);
    if (d_cur<d_closest) {
      d_closest = d_cur;
      i_closest = i;
    }
  }

    //i_closest += 7;
    //i_closest = std::min(i_closest, (int)refpath.points.size()-1);




  //i_last = std::max(i_last, i_closest);
  for (; i_last<refpath.size(); ++i_last) {
    if (std::hypot(refpath[i_last].x-locData.x, refpath[i_last].y-locData.y)>=WAYPOINT_MIN_DST) break;
  }
  if (i_last==refpath.size()) --i_last;

  ClothoidList startPath;
  double s_keep = 0;
  if (currPathCL.numSegment()>0) {
    startPath = currPathCL;
    double xs, ys, s, t, dst;
    int i_curve;
    startPath.closestPointInRange_ISO(start.x, start.y, 0, currPathCL.numSegment()-1, xs, ys, s, t, dst, 
i_curve);
    s = std::min(std::max(s, 0.), startPath.length());
    if (s==startPath.length()) {
      startPath = ClothoidList();
    }
    else {
      startPath.trim(s, startPath.length());
    }
    startPath = ClothoidList(startPath);

    s_keep = std::min(startPath.length(), 1.0);

    if (s_keep==0.) {
      startPath = ClothoidList();
    }
    else {
      startPath.trim(0, s_keep);
    }
    startPath = ClothoidList(startPath);

    if (startPath.numSegment()>0 && safePath(startPath)) {
      startPath.eval(s_keep, start.x, start.y);
      start.theta = startPath.theta(s_keep);
    }
    else {
      startPath = ClothoidList();
    }
  }
   

 

    int rn,cn; 
    double xt = refpath[i_last].x ;//people[i].Xpred[0][9] + locData.x;
    double yt = refpath[i_last].y ;//people[i].Xpred[1][9] + locData.y;
    map.world2map(xt, yt, rn, cn);
    int radius = std::ceil(0.4/map.resolution);
    cv::circle(map.img, cv::Point(cn, rn), radius, cv::Scalar(0), -1);


  
  //std::cerr << "PLANNING FROM " << locData.x << " " << locData.y << " TO " << refpath[i_last].x << " " << refpath[i_last].y << std::endl;
  FMPlanner fmplanner;
  fmplanner.setMap(&map);
  
std::vector<DPSmoothing::Curve> localPath;

  std::vector<DPSmoothing::Curve> path;

  if (!fmplanner.plan(start, refpath[i_last], localPath, true))
  {
    std::vector<State2d> rawLocalPath;
    if (!fmplanner.planRaw(start, refpath[i_last], rawLocalPath)) {
      return false;
    }
    for (int i=0; i<startPath.numSegment(); ++i) {
      path.push_back(startPath.get(i));
    }
    for (int i=1; i<rawLocalPath.size(); ++i) {
      double xp = rawLocalPath[i-1].x;
      double yp = rawLocalPath[i-1].y;
      double xc = rawLocalPath[i].x;
      double yc = rawLocalPath[i].y;
      double L = std::hypot(xc-xp, yc-yp);
      double theta = std::atan2(yc-yp, xc-xp);
      G2lib::ClothoidCurve cc(xp, yp, theta, 0., 0., L);
      path.push_back(cc);
    }
  }
  else {
    if (startPath.numSegment()>0) {
      for (int i=0; i<startPath.numSegment(); ++i) {
        path.push_back(startPath.get(i));
      }
    }
    for (int i=0; i<localPath.size(); ++i) {
      path.push_back(localPath[i]);
    }
  }

  setPath(path);
  return true;
}

void Navigation::setPath(std::vector<DPSmoothing::Curve> const & path) {
  if (path.empty()) {
    return;
  }

  ClothoidList cl;
  for (int i=0; i<path.size(); ++i) {
    #ifdef USE_DUBINS
      ClothoidList pc = path[i].toClothoidList();
      for (int j=0; j<pc.numSegment(); ++j)
        cl.push_back(pc.get(j));
    #else
      cl.push_back(path[i]);
    #endif
  }

 // std::cerr << "NEW PATH LENGTH: " << cl.length() << std::endl;

  std::vector<PathPoint> pts;
  int ncuts = std::ceil(cl.length()/0.1);
  double ds = cl.length()/ncuts;
  for (int i=0; i<ncuts; ++i) {
    double s0 = i*ds;
    double x, y, theta, kappa;
    cl.evaluate(s0, theta, kappa, x, y);
    pts.emplace_back(x, y, theta, s0, kappa, cl.kappa_D(s0));
  }
  {
    std::unique_lock<std::mutex> lock(pathMutex);
    currPath = Path(pts);
    currPathCL = cl;
    newPath = true;
  }
}

bool Navigation::safePath() {
  if (!currPathCL.numSegment()) return true;

  RobotStatus::LocalizationData locData;
#ifdef LOC_REALSENSE
  HardwareGlobalInterface::getInstance().getLocalizationDataRealSense(locData);
#else
  HardwareGlobalInterface::getInstance().getLocalizationDataKarto(locData);
#endif

  DynamicFrenetPoint dfp;
  int idx;
  currPath.getClosestPoint(locData.x, locData.y, locData.theta, dfp, idx);

  cv::Mat img;
  {
    std::unique_lock<std::mutex> lock(mmMutex);
    img = mm->generateImage();
  }
  cv::cvtColor(img, img, CV_BGR2GRAY);

  double x0 = currPath.points[idx].x, y0 = currPath.points[idx].y;
  for (int i=idx; i<currPath.points.size(); ++i) {
    double x = currPath.points[i].x, y = currPath.points[i].y;
    if (std::hypot(x-x0, y-y0)>SAFETY_DISTANCE) break;
    int r, c;
    bool inside = mm->getMapCoordinatesAbs(x, y, r, c); // make thread safe!!!
    if (inside && img.at<uint8_t>(r,c)>0)
    {
      return false;
    }
  }

  return true;
}

bool Navigation::safePath(ClothoidList const & path) {
  if (!path.numSegment()) return true;

  cv::Mat img;
  double resolution;
  {
    std::unique_lock<std::mutex> lock(mmMutex);
    img = mm->generateImage();
    resolution = mm->getResolution();
  }
  cv::cvtColor(img, img, CV_BGR2GRAY);

  int kSize = std::ceil(0.4/resolution);
  cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
                                          cv::Size(2*kSize+1, 2*kSize+1),
                                          cv::Point(kSize, kSize));
  cv::dilate(img, img, element);


  int ncuts = std::ceil(path.length()/resolution);
  double ds = path.length()/ncuts;

  for (int i=0; i<=ncuts; ++i) {
    double s = std::min(path.length(), i*ds);
    double x = path.X(s), y = path.Y(s);
    int r, c;
    bool inside = mm->getMapCoordinatesAbs(x, y, r, c); // make thread safe!!!
    if (inside && img.at<uint8_t>(r,c)>0)
    {
      return false;
    }
  }

  return true;
}

void Navigation::start() {
  running.store(true);
   predictionPublisher = std::make_unique<Common::Publisher>(HardwareGlobalInterface::getInstance().getParams().predictionPublisher);
   //peopleNeighPublisher = std::make_unique<Common::Publisher>(HardwareGlobalInterface::getInstance().getParams().peopleNeighPublisher);



   imgPublisher = std::make_unique<Common::Publisher>("tcp://*:9222");
   imgPubTimer = std::make_unique<Timer>();
   imgPubTimer->registerCallback([this](){ this->publishImg(); });
   imgPubTimer->start(100);

  if (!validGoal && waypoints.empty()) {
    std::cerr << "No goal set! Aborting..." << std::endl;
    //running.store(false);
    done.store(true);
    failed.store(true);
    return;
  }

  bool mmInit = false;
  do {
    {
      std::unique_lock<std::mutex> lock(mmMutex);
      mmInit = mm->isInitialized();
    }
    if (!mmInit) {
      std::cerr << "[WARN] MapManager not initialized yet. Waiting..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  } while (!mmInit && !done.load());

  // generate global reference path
  if (!generateReferencePath()) {
    std::cerr << "Failed to generate reference path! Aborting..." << std::endl; // running.store(false);
    failed.store(true);
    done.store(true);
    return;
  }

  // generate local path reaching next waypoint
  if (!generateLocalPath()) {
    std::cerr << "Failed to generate initial local path! Aborting..." << std::endl; // 
running.store(false);
    failed.store(true);
    done.store(true);
    return;
  }


  failed.store(false);
  controlRunning.store(true);
  eop.store(false);
  HardwareGlobalInterface::getInstance().robotOnVelControl();

  int nFailures = 0;

  while (!done.load() && !eop.load())
  {
    if (!paused) {
      if (!generateLocalPath()) {
        stopSpraying();
        std::cerr << "[WARN] Failed re-planning" << std::endl;
        ++nFailures;
        if (nFailures >= MAX_FAILURES) {
          std::cerr << "[WARN] Reached maximum number of failures. Stopping..." << std::endl;
          break;
        }
      }
      else {
        nFailures = 0;
      }
    }
    else {
      stopSpraying();
    }

// if (!safePath()) { // std::cerr << "[WARN] Unsafe path." << std::endl; // emergencyBreak.store(true); 
// } // else { // emergencyBreak.store(false); // }

// cv::Mat img;
 // {
 // std::unique_lock<std::mutex> lock(mmMutex);
 // img = mm.generateImage();
 // }
 // drawCL(img, mm, refpath);
  // drawCL(img, mm, currPath);
  // setImage(img);

    // wait
    std::this_thread::sleep_for(std::chrono::milliseconds(REPLANNING_PERIOD_MS));
  }


  controlRunning.store(false);
  HardwareGlobalInterface::getInstance().robotOff();
  validGoal = false;
  //running.store(false);
  done.store(true);
  //std:: cerr << "END OF NAVIGATION: " << eop.load() << std::endl;
  failed.store(!eop.load());
}

void Navigation::stop() {
  stopSpraying();
  done.store(true);
}

void Navigation::pause() {
  paused = true;
  stopSpraying();
  HardwareGlobalInterface::getInstance().robotOff();
}

void Navigation::resume() {
  paused = false;
  HardwareGlobalInterface::getInstance().robotOnVelControl();
}

//void Navigation::setGoal(const State2d & goal)
 //{
 // this->goal = goal;
 // validGoal = true;
 //}

//void Navigation::stop()
 //{
 //
 //lidarTimer->stop();
 // try {
 // cv::destroyWindow("mm");
 // } catch  (std::exception const & ex) {}
 //}

//void Navigation::setPath(const G2lib::ClothoidList &refpath)
 //{
 // this->refpath = refpath;
 //}

cv::Mat Navigation::getImage() {
  std::unique_lock<std::mutex> lck(mtx);
  return img.clone();
}

//void Navigation::setRunning(bool running) //{ // this->running.store(running); //}

//bool Navigation::isRunning()
 //{ 
// return running.load();
 //}

bool Navigation::hasFailed() {
  return failed.load();
}

void Navigation::setSpray(bool spray) {
  this->spray = spray;
}


//bool Navigation::sendPath(ClothoidSpline const & path) //{ // static int NXT_ID = 1;

// using nlohmann::json;

//	json jObj; //	jObj["cmd"] = "send_path_json";

// const double MAX_DS = 0.1; // int nCuts = std::ceil(path.splineLength()/MAX_DS); // double ds = path.splineLength()/nCuts;

// json jPath = json::array(); // for (int i=0; i<=nCuts; ++i) { // double s0 = i*ds; // SpaceConfig sc = path.eval(s0); // json jPt; // jPt["x"] = sc.pos.x; // jPt["y"] = sc.pos.y;
 // jPt["theta"] = sc.theta; // jPt["s"] = s0; // jPt["c"] = sc.kappa; // jPt["dc"] = path.dk(s0); // jPath.push_back(jPt); 
// } // jObj["path"]["data"] = jPath;

// std::string currId = std::to_string(NXT_ID++); // jObj["path"]["name"] = currId;


// std::string ack; // Common::RequesterSimple::status_t status; 
// bool ok = false; // if (reqGuidance.request(jObj.dump(), ack, status))
 // { // json jRes = json::parse(ack);
 // ok = jRes["ack"]=="true"; // } 
// if (!ok) { // std::cerr << "Failed to send path to GUIDANCE" << std::endl; 
// return false; // } // return true; //}

void Navigation::computeControl_pathfollowing(){
  if (!controlRunning.load()) return;

  if (emergencyBreak.load()) {
    HardwareGlobalInterface::getInstance().vehicleMove(0., 0.); // stop the vehicle
    return;
  }

  static auto t_act = std::chrono::high_resolution_clock::now();
  static auto t_old = std::chrono::high_resolution_clock::now();
  static auto init_time = std::chrono::high_resolution_clock::now();

  static double delta = 0.0;
  static double diff_delta_diff_ly = 0.0;
  static double gamma = 0.0;
  static double rho = 0.0;
  static double s_dot = 0.0;
  static double xi_dot = 0.0;
  static double w_d_unicycle = 0.0;

  static double l_x_dot = 0.0;
  static double theta_tilde_dot = 0.0;
  static double xi_ddot = 0.0;
  static double s_dot_future = 0.0;


  static double relative_time = 0.0;

  static double kappa_x_p = 1.0;
  static double kappa_p = 1.5;
  static double approach_p = 1.0;
  static double t_controller = 0.0;

  static DynamicFrenetPoint dfp;

  static int path_index = 0;

  RobotStatus::LocalizationData vehiclePose;
#ifdef LOC_REALSENSE
  HardwareGlobalInterface::getInstance().getLocalizationDataRealSense(vehiclePose);
#else
  HardwareGlobalInterface::getInstance().getLocalizationDataKarto(vehiclePose);
#endif

  std::unique_lock<std::mutex> lock(pathMutex);
  if (currPath.points.empty()) return;

  if(newPath){
      currPath.getClosestPoint(vehiclePose.x,vehiclePose.y,vehiclePose.theta,dfp,path_index);
      t_controller = 0.010; //set default 10 ms as first try
      t_act = std::chrono::high_resolution_clock::now();
      init_time = std::chrono::high_resolution_clock::now();
      newPath = false;
      controlTime.start();
  }else{
      currPath.getDynamicFrenetPoint(vehiclePose.x,vehiclePose.y,vehiclePose.theta,dfp,path_index);
      t_old = t_act;
      t_act = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> tmp = (t_act - t_old)/1000.0;
      t_controller = tmp.count();
      t_controller = ((double)controlTime.getTime())/1000.0;
      controlTime.start();
  }
  lock.unlock();


  if (std::abs(currPath.getLastPoint().s-dfp.s)<0.1) {
    eop.store(true);
    return;
  }

  std::chrono::duration<double, std::milli> tmp2 = (t_act - init_time);
  relative_time = tmp2.count();

  RobotStatus::HardwareData hwData;
  HardwareGlobalInterface::getInstance().getHardwareData(hwData);
  xi_dot = cos(dfp.theta_tilde) + kappa_x_p*dfp.l_x;
  s_dot = hwData.speed*xi_dot;
  l_x_dot = -s_dot*(1-dfp.c*dfp.l_y)+hwData.speed*cos(dfp.theta_tilde);
  theta_tilde_dot = hwData.omega - dfp.c*s_dot;
  xi_ddot = -sin(dfp.theta_tilde)*theta_tilde_dot + kappa_x_p*2.0*dfp.l_x*l_x_dot;

  if(dfp.s<0.01 && s_dot<0.0){
      s_dot = 0.0;
      xi_dot = 0.0;
      xi_ddot = 0.0;
  }

  delta = -M_PI_2 * tanh(dfp.l_y * approach_p);


  diff_delta_diff_ly = -M_PI/2.0 * approach_p * (1.0 - pow(tanh(dfp.l_y * approach_p),2));
  gamma = dfp.c*xi_dot + (-dfp.c*xi_dot*dfp.l_x + sin(dfp.theta_tilde))*diff_delta_diff_ly;

  rho = gamma - kappa_p*diff_angle(dfp.theta_tilde,delta);

  w_d_unicycle = hwData.speed*rho;

  double curvatureEffect = 1.0 - (2.0/3.0)*(fabs(dfp.c)/currPath.maxCurvature);

  //Add velocity profile in order to start from 0 and reach vsp in 2 sec
 // double smoothProfile = 0.0; 
// static double settlingTime = 2000.0; //2sec 
// if(relative_time>0.0 && relative_time<=settlingTime/2.0){ 
// smoothProfile = 2.0*pow(relative_time,2.0)/pow(settlingTime,2.0); 
// }else if(relative_time>settlingTime/2.0 && relative_time<=settlingTime){
 // smoothProwfile = -2.0*pow(relative_time,2.0)/pow(settlingTime,2.0) + 4.0*relative_time/settlingTime -1;
 // }else if(relative_time > settlingTime){
 // smoothProfile = 1.0; 
// }
  double vel_sp = 0.2;
  double smoothProfile = 1.0;


  //vehicleMove(vel_old,w_d_unicycle);
  HardwareGlobalInterface::getInstance().vehicleMove(vel_sp, w_d_unicycle);


  //Update the current dfp.s
  s_dot_future = s_dot + t_controller*(xi_ddot);

  dfp.s = dfp.s + t_controller*(s_dot+s_dot_future)/2.0;

  if(dfp.s<0.0){
      dfp.s = 0.0;
  }
}


void Navigation::computeControl(){
  if (!controlRunning.load() || eop.load() || paused) {
    stopSpraying();
    spraying = false;
    return;
  }

  auto t1 = tp.getTime();
  double dt = t1-t0;
  bool changed = false;
  if (spraying && dt>400) {
    stopSpraying();
    changed = true;
  }
  else if (!spraying && dt>2000) {
    startSpraying();
    changed = true;
  }

  if (changed) {
    spraying = !spraying;
    t0 = t1;
  }

// if (emergencyBreak.load()) { // HardwareGlobalInterface::getInstance().vehicleMove(0., 0.); 
// stop the vehicle
 // return; // }


  RobotStatus::LocalizationData currentPose;
#ifdef LOC_REALSENSE
  HardwareGlobalInterface::getInstance().getLocalizationDataRealSense(currentPose);
#else
   HardwareGlobalInterface::getInstance().getLocalizationDataKarto(currentPose);
#endif

// double dist = sqrt(pow((currentPose.x - oldVehiclePose.x),2.0)+pow((currentPose.y - oldVehiclePose.y),2.0)); 
// double theta1 = atan2((currentPose.y - oldVehiclePose.y),(currentPose.x - oldVehiclePose.x)); 
// double theta2 = currentPose.theta - theta1; // double dx = dist*cos(theta2);
 // double dy = dist*sin(theta2); 
// double dtheta = currentPose.theta - oldVehiclePose.theta;

// oldVehiclePose = currentPose;

    Path currentPath;
    {
      std::unique_lock<std::mutex> lock(pathMutex);
      currentPath = currPath;
    }

    int i_closest = -1;
    double d_closest = std::numeric_limits<double>::infinity();
    for (int i=0; i<currentPath.points.size(); ++i) {
      double d_cur = std::hypot(currentPath.points[i].x-currentPose.x, 
currentPath.points[i].y-currentPose.y);
      if (d_cur<d_closest) {
        d_closest = d_cur;
        i_closest = i;
      }
    }

    i_closest += 7;
    i_closest = std::min(i_closest, (int)currentPath.points.size()-1);



    Map map;
    {
      std::unique_lock<std::mutex> lock(mmMutex);
      map.resolution = mm->getResolution();
      map.img = mm->generateImage();
      mm->getWorldCoordinatesAbs(0, 0, map.x0, map.y0);
    }


    PathPoint frenetPoint = currentPath.points[i_closest];
    cv::Mat img = map.img.clone();

    cv::cvtColor(map.img, map.img, CV_BGR2GRAY);

    int kSize = std::ceil((vparams.dA)/map.resolution);
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
                                            cv::Size(2*kSize+1, 2*kSize+1),
                                            cv::Point(kSize, kSize));
    cv::dilate(map.img, map.img, element);

    std::vector<PersonItem> trackedPeople;
    HardwareGlobalInterface::getInstance().getTrackingData(trackedPeople);
    std::vector<DynamicObstacle> dynObstacles;
    dynObstacles.reserve(trackedPeople.size());
    for (auto const & p: trackedPeople) {
      DynamicObstacle dObst;
      dObst.id = p.id;
      dObst.pose = { p.x, p.y };
      dObst.v = { p.vx, p.vy };
      dynObstacles.push_back(dObst);
    }
    RobotStatus::LocalizationData locData; 
    #ifdef LOC_REALSENSE
         HardwareGlobalInterface::getInstance().getLocalizationDataRealSense(locData);
    #else
         HardwareGlobalInterface::getInstance().getLocalizationDataKarto(locData);
    #endif

    std::vector<Neighbours> NeighboursP = getNeighbours();


    for (auto const & tp: trackedPeople) {
       // Filter other robots from people
       bool is_robot = false;
       //std::cout <<" dist: "<<  std::hypot(NeighboursP[0].pose.x - (tp.x+locData.x)  ,NeighboursP[0].pose.y-(tp.y+locData.y) )  <<std::endl;
       for (int j = 0 ; j < NeighboursP.size() ; ++j){
          if ( std::hypot(NeighboursP[j].pose.x - (tp.x+locData.x)  ,NeighboursP[j].pose.y-(tp.y+locData.y) ) < 0.5){
             is_robot = true;
             break;
          }
       }
       if (is_robot) {
          continue;
       }

        double idx = -1;
        for (auto const & p: people) {
                if (tp.id == p.id) {
                        idx = std::addressof(p) - std::addressof(people[0]);

                }
        }

     
        if (idx != -1) {
                people[idx].isupdated = true;
                if (people[idx].x.size() >= 10) {
                        people[idx].x.erase(people[idx].x.begin());
                        people[idx].y.erase(people[idx].y.begin());
                        people[idx].vx.erase(people[idx].vx.begin());
                        people[idx].vy.erase(people[idx].vy.begin());
                }
        people[idx].x.push_back(tp.x);
        people[idx].y.push_back(tp.y);
        people[idx].vx.push_back(tp.vx);
        people[idx].vy.push_back(tp.vy);

        }else{
                People partypeople;
                partypeople.x.push_back(tp.x);
                partypeople.y.push_back(tp.y);
                partypeople.vx.push_back(tp.vx);
                partypeople.vy.push_back(tp.vy);
                partypeople.id = tp.id;
                partypeople.isupdated = true;
                people.push_back(partypeople);
        }
    }

    // Clear lost people
    for ( auto iter = people.begin() ; iter != people.end(); ){
       if (iter->isupdated == false){
          iter = people.erase(iter);
       } else {
          iter->isupdated = false;
          ++iter;
       }
    }

    nlohmann::json jdata;

    // Voronoi
    Point2d centroid_voronoi = voronoiCentroid(map, {currentPose.x, currentPose.y}, vparams,dynObstacles, NeighboursP, frenetPoint, img, jdata, currentPose.theta);

    // Append neighbours
    nlohmann::json j_array = nlohmann::json::array();
    for (int p = 0 ; p < people.size() ; p++) {
       nlohmann::json j_in;
       j_in["id"] = people[p].id;
       for (int i = 0; i < people[p].x.size(); i++) {
          j_in["x"][i] = people[p].x[i];
          j_in["y"][i] = people[p].y[i];
          j_in["vx"][i] = people[p].vx[i];
          j_in["vy"][i] = people[p].vy[i];
       }
       j_array.push_back(j_in);
    }
    jdata["peopleNeigh"] = j_array;
    //std::cout << j_array <<std::endl;
    // Send data
    executor->setDataToPublish(jdata);

    centroid_voronoi.x -= currentPose.x;
    centroid_voronoi.y -= currentPose.y;

    double norm = std::hypot(centroid_voronoi.x, centroid_voronoi.y);
    double heading[2] = {cos(currentPose.theta), sin(currentPose.theta)};
    double heading_reference[2] = { centroid_voronoi.x/norm, centroid_voronoi.y/norm };
    //std::vector<QPointF> Cv;
    //ControlParams cparams;
    //Cv.push_back(QPointF(centroid_voronoi(0),centroid_voronoi(1)));

    if ( norm < cparams.beta_th + 0.15 ) {
        vparams.R_gaussian = vparams.R_gaussian - 0.05*(vparams.R_gaussian-vparams.rhoD);
        //std::cerr << "RGAUSSIAN IF" << std::endl;
    }else {
       vparams.R_gaussian = vparams.R_gaussian - 0.05*(vparams.R_gaussian - vparams.rhoD);
        //std::cerr << "RGAUSSIAN ELSE" << std::endl;
   }




// std::cerr << vparams.R_gaussian << std::endl; 
 if ( norm < 0.4  ) { 
 cparams.v_des = cparams.v_des - 0.5*(cparams.v_des-0.05);
        //std::cerr << "RGAUSSIAN IF" << std::endl; 
 }else if (std::isnan(norm) ){
 cparams.v_des = cparams.v_des - 0.5*(cparams.v_des); 
}else{
  cparams.v_des = cparams.v_des - 0.5*(cparams.v_des - cparams.v_desC);
        //std::cerr << "RGAUSSIAN ELSE" << std::endl; 
 }

   // std::cerr << "vdes = " << cparams.v_des <<std::endl;
   // std::cerr << "rgauss = " << vparams.R_gaussian <<std::endl;
   // std::cerr << "norm = " << norm << std::endl;
   //std::cout <<  norm <<std::endl;

    double control_inputs[2];
    double hdot = heading[0]*heading_reference[0] + heading[1]*heading_reference[1];
    if ( hdot < cos(cparams.theta_th)){
        control_inputs[0] = past_controls[0] - cparams.k_brake * past_controls[0] * cparams.dtC/1000.0;
        if (control_inputs[0] < 0){
            control_inputs[0] = 0 ;
        }
    }
    else{

        if( norm > cparams.beta_th || std::isnan(norm)){
         control_inputs[0] = past_controls[0] - cparams.k_i*(past_controls[0] - cparams.v_des) * cparams.dtC/1000.0;
        }   
        else{
         control_inputs[0] = past_controls[0] - cparams.k_brake*(past_controls[0] - 0.2) * cparams.dtC/1000.0;
        }
    }

   
    double powe = 1.0 - hdot;
    if (powe < 0){
        powe = 0;
    }
    control_inputs[1] = - cparams.k_angular * pow(powe,cparams.alpha) * Signum(heading_reference[0]*heading[1] - heading_reference[1]*heading[0]);
    if (control_inputs[1] > 0.75){
      control_inputs[1] = 0.75;
    }else if (control_inputs[1] < -0.75){
      control_inputs[1] = -0.75;
    }

    if (std::hypot(currentPath.points.back().x-currentPose.x, currentPath.points.back().y-currentPose.y)< 0.5){
      control_inputs[0] = past_controls[0] - cparams.k_brake * past_controls[0] * cparams.dtC/1000.0;
      control_inputs[1] = 0;
      eop.store(true);
    }


    for (auto const & dO: dynObstacles) {
      if (std::hypot(dO.pose.x, dO.pose.y)<(vparams.dA+vparams.dO)) {
        control_inputs[0] = past_controls[0] - cparams.k_brake * past_controls[0] * cparams.dtC/1000.0;
        break;
      }
    }


    past_controls[0] = control_inputs[0];
    past_controls[1] = control_inputs[1];
    
    nlohmann::json j_pub;
    nlohmann::json j_pred = nlohmann::json::array();

    // Predictive module
    for (int p = 0 ; p < people.size() ; p++) {
        nlohmann::json j_in;
        if (people[p].x.size() >= 10) {
            std::vector<std::vector<double>> Xin{people[p].x,people[p].y,people[p].vx,people[p].vy};
            people[p].Xpred = prediction(Xin);
            for (int i = 0 ; i < people[p].Xpred[0].size(); ++i){
                int r,c;
                map.world2map(people[p].Xpred[0][i]+currentPose.x, people[p].Xpred[1][i]+currentPose.y, r, c);
                j_in["xpred"][i] = c;//people[p].Xpred[0][i];
                j_in["ypred"][i] = r;//people[p].Xpred[1][i];
            }
            j_pred.push_back(j_in);
        }
     }

   // Predictive module for peopleNeigh
   for (int i = 0; i < NeighboursP.size(); i++){
  // std::cout << "peopleNeighsize: " << NeighboursP[i].peopleNeigh.size() << std::endl;
       for (int j = 0 ; j < NeighboursP[i].peopleNeigh.size(); j++) {
           nlohmann::json j_in;
               //std::cout<<"QUI1"<<std::endl;
   
        if (NeighboursP[i].peopleNeigh[j].x.size() >= 10) {
		//std::cout<<"QUI"<<std::endl;
              std::vector<std::vector<double>> Xin{NeighboursP[i].peopleNeigh[j].x,
                                                   NeighboursP[i].peopleNeigh[j].y,
                                                   NeighboursP[i].peopleNeigh[j].vx,
                                                   NeighboursP[i].peopleNeigh[j].vy};
              NeighboursP[i].peopleNeigh[j].Xpred = prediction(Xin);
              for (int p = 0 ; p < NeighboursP[i].peopleNeigh[j].Xpred[0].size(); p++) {
                   int r,c;
                   map.world2map(NeighboursP[i].peopleNeigh[j].Xpred[0][p] + NeighboursP[i].pose.x, NeighboursP[i].peopleNeigh[j].Xpred[1][p] + NeighboursP[i].pose.y, r, c);
                   j_in["xpred"][p] = c;//people[p].Xpred[0][i];
                   j_in["ypred"][p] = r;//people[p].Xpred[1][i];
              }
              j_pred.push_back(j_in);
          }
       }
    }

    j_pub["object_predicted"] = j_pred;
    j_pub["control_inputs"]   = control_inputs[0];
    std::string pub_string = j_pub.dump();
    predictionPublisher->send("PUB_PRED", pub_string.c_str(), pub_string.size());



   unsigned long now1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    
    File4 << now1 << ";" << control_inputs[0] << ";" << control_inputs[1] << ";" << vparams.R_gaussian << ";" << cparams.v_des << ";" << frenetPoint.x << ";" << frenetPoint.y << ";" << std::endl;
    
    File5 << now1 << ";" << currentPath.points.size() << std::endl;
     for(int i = 0 ; i < currentPath.points.size() ; i++){
     File5 << currentPath.points[i].x << ";" << currentPath.points[i].y << std::endl;
    }
     File6 << now1 << ";" << refpath.size() << std::endl;
     for(int i = 0 ; i < refpath.size() ; i++){
     File6 << refpath[i].x << ";" << refpath[i].y << std::endl;
    }


    int prediction_count = 0;
    for (int i=0; i<people.size(); i++) {
      if (people[i].x.size() >= 10) {
        prediction_count += 1;
      }
    }
    File7 << now1 << ";" << prediction_count << std::endl;

    for (int i=0; i<people.size(); i++) {
      if (people[i].x.size() < 10) {
        continue;
      }
      for (int j=10; j<40; ++j) {
        File7 << people[i].Xpred[0][j] + currentPose.x << ";" << people[i].Xpred[1][j] + currentPose.y << std::endl;
      }
    }


    static int cnt1 = 20; 
if (cnt1 >= 20){ 
File4.flush(); 
File5.flush();
File6.flush();
File7.flush(); 
cnt1 = 0;
}
++cnt1;
    
    //cout << Param1e.R_gaussian<<endl;

    //control_inputs[0] = control_inputs[1] = 0;
    HardwareGlobalInterface::getInstance().vehicleMove(control_inputs[0], control_inputs[1]);

  
  // ----------------------------- DRAWING
     drawCL(img, *mm, currentPath, {120,212,244});

     int cf, rf;
     mm->getMapCoordinatesAbs(currentPath.points[i_closest].x, currentPath.points[i_closest].y, rf, cf);
     if (IMG == true){
     cv::circle(img, cv::Point(cf, rf), 1, cv::Scalar(255, 100, 255), -1);
     }
     int cv, rv;
     mm->getMapCoordinatesAbs(currentPose.x, currentPose.y, rv, cv);
     if (IMG == true){
     cv::circle(img, cv::Point(cv, rv), 1, cv::Scalar(20, 100, 40), -1);
     }
  // navigatorPainter->drawPath(currentPath);
 // navigatorPainter->drawFrenetPoint(frenetPoint);
    //navigatorPainter->drawCentroid(Cv);
 //
 //navigatorPainter->drawOvetti(eggPlot);
 // navigatorPainter->drawFPermanent(centroid_voronoi(0),centroid_voronoi(1));
if (IMG == true){

     for (int i=0; i<dynObstacles.size(); ++i) {
       int r, c;
       int vr,vc;
       double x, y;
       double vx, vy;
       x = dynObstacles[i].pose.x + currentPose.x;
       y = dynObstacles[i].pose.y + currentPose.y;
       vx = dynObstacles[i].v.x;
       vy = dynObstacles[i].v.y;
       //std::cout << vx <<" , "<< vy<<std::endl;
       map.world2map(x, y, r, c);
       map.world2map(x+vx,y+vy,vr,vc);
       cv::circle(img, cv::Point(c,r), 1, cv::Scalar(255,0,0), -1);
       cv::line(img, cv::Point(c,r), cv::Point(vc,vr),cv::Scalar(0,0,255), 1);
      
     }
for (int n=0; n < NeighboursP.size(); n++){
   for (int i=0; i<NeighboursP[n].peopleNeigh.size(); ++i) {
  //std::cout<<people[i].Xpred.size()<<std::endl;
	if (NeighboursP[n].peopleNeigh[i].x.size() >= 10){
     	for (int j=10; j<40; ++j) {
     		int rt, ct;//, ctt, rtt;
    		//double xtt = people[i].Xpred[0][j-1]+ currentPose.x;
   		//double ytt = people[i].Xpred[1][j-1]+ currentPose.y;
   		double xt = NeighboursP[n].peopleNeigh[i].Xpred[0][j] + NeighboursP[n].pose.x;
   		double yt = NeighboursP[n].peopleNeigh[i].Xpred[1][j] + NeighboursP[n].pose.y;



                //map.world2map(xtt, ytt, rtt, ctt);
   		map.world2map(xt, yt, rt, ct);
 		cv::circle(img, cv::Point(ct, rt), 1, cv::Scalar(255,153,255), -1);
   		}
        	}
	
        }
}
for (int i=0; i<people.size(); ++i) {
  //std::cout<<people[i].Xpred.size()<<std::endl;

        if (people[i].x.size() >= 10){
        for (int j=10; j<40; ++j) {
                int rt, ct;//, ctt, rtt;
                //double xtt = people[i].Xpred[0][j-1]+ currentPose.x;
                //double ytt = people[i].Xpred[1][j-1]+ currentPose.y;
                double xt = people[i].Xpred[0][j] + currentPose.x;
                double yt = people[i].Xpred[1][j] + currentPose.y;


                //map.world2map(xtt, ytt, rtt, ctt);
                map.world2map(xt, yt, rt, ct);
                cv::circle(img, cv::Point(ct, rt), 1, cv::Scalar(255,153,255), -1);
                }
                }

        if (people[i].x.size() >= 10){
        for (int j=0; j<10; ++j) {
                int rt, ct;//, ctt, rtt;
                //double xtt = people[i].Xpred[0][j-1]+ currentPose.x;
                //double ytt = people[i].Xpred[1][j-1]+ currentPose.y;
                double xt = people[i].Xpred[0][j] + currentPose.x;
                double yt = people[i].Xpred[1][j] + currentPose.y;


                //map.world2map(xtt, ytt, rtt, ctt);
                map.world2map(xt, yt, rt, ct);
                cv::circle(img, cv::Point(ct, rt), 1, cv::Scalar(0,153,255), -1);
                }
                }
        }



     for (int i=0; i<NeighboursP.size(); ++i) {
       int r, c;
       double x, y;
       x = NeighboursP[i].pose.x; //+ currentPose.x;
       y = NeighboursP[i].pose.y; //+ currentPose.y;
       map.world2map(x, y, r, c);
       cv::circle(img, cv::Point(c,r), 2, cv::Scalar(255,255,0), -1);
     }
     for (int i=0; i<NeighboursP.size(); ++i) {
	for (int j = 0 ; j<NeighboursP[i].poseH.size(); j++){
       int r, c;
       double x, y;
       x = NeighboursP[i].poseH[j].x + NeighboursP[i].pose.x;
       y = NeighboursP[i].poseH[j].y + NeighboursP[i].pose.y;
       map.world2map(x, y, r, c);
       cv::circle(img, cv::Point(c,r), 2, cv::Scalar(20,20,100), -1);
        }
      }

}
     setImage(img);
    // cv::imwrite("vor.jpg", img);
}


std::vector<Neighbours> Navigation::getNeighbours() {
  std::vector<Neighbours> res;
  res.reserve(neighbours.size());
  {
    std::unique_lock<std::mutex> lock(neighboursMtx);
    for (auto const & n: neighbours) {
      res.emplace_back(n.second);
    }
  }
  return res;
}





