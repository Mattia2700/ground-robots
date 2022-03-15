#ifndef MAP_MANAGER_HH
#define MAP_MANAGER_HH

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "utils.hpp"
#include "robotstatus.h"

struct PosedMap
{
  cv::Mat map;
  Pose pose;
};


class MapManager
{
public:
  struct RobotGeometry 
  {
    double radius;
  };

  MapManager(int hsize, double resolution, RobotGeometry const & rg, std::vector<Obstacle> const & globalmap = {});

  // method to update costmap with new scan-data
  void
  update(ScanData const & scan);

  // method to init-reset costmap
  void 
  reset(int hsize, double resolution);

  int
  getSize();
  
  Point2d
  getPose();
  // utility methods
  // get cell (r,c) from coordinates (x,y)
  bool 
  getMapCoordinates(double x, double y, int & r, int & c);

  bool 
  getMapCoordinatesAbs(double x, double y, int & r, int & c);

  // get coordinates (x,y) from cell (r,c)
  bool 
  getWorldCoordinates(int r, int c, double & x, double & y);

  bool 
  getWorldCoordinatesAbs(int r, int c, double & x, double & y);

  void
  getGlobalMapCoordinates(double x, double y, int & r, int & c);

  // cv::Mat const 
  // map();

  double
  getResolution() { return resolution; }

  cv::Mat
  generateImage();

  const PosedMap &
  getGlobalMap();

  bool
  isInitialized() { return init; }

  std::vector<cv::Point2f> lidarPoint;

private:
  RobotGeometry rg;

  std::vector<PosedMap> maps_queue, raw_maps;
  cv::Mat raw_map, static_map, dynamic_map;
  PosedMap global_map;
  cv::Mat costmap;
  bool init;

  int hsize;
  double resolution;
  Point2d origin;

  void 
  update(PosedMap & map, double x, double y);

  void 
  process(PosedMap & map, cv::Mat & static_obst, cv::Mat & dynamic_obst);

  PosedMap
  createMap(ScanData const & scan);

  void
  storeMap(PosedMap const & map);

  void
  storeRawMap(const PosedMap &map);


  cv::Mat
  translate(cv::Mat const & map, double dx, double dy);

  void
  updateCostmap();

  void
  drawLidarPoints(std::vector<Point2d> const & lidarScan);


  void
  initGlobalMap(std::vector<Obstacle> const & globalmap);

  cv::Mat
  getGlobalView(int rmin, int cmin, int rmax, int cmax);

};


#endif
