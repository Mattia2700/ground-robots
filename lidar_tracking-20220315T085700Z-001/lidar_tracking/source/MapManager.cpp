#include "MapManager.hh"

#include <limits>

static const int32_t MAX_HISTORY   = 1;
static const uint8_t EMPTY         = 0;
static const uint8_t FULL          = 255;
static const double  MIN_THRESHOLD = 0.2;
static const double  MAX_THRESHOLD = 8.4;
static const double  MIN_AREA      = 50;
static const double  MIN_RATIO     = 1./30;
static const double  SCAN_RADIUS   = 0.05;

//using utils::Obstacle;
//using utils::ScanData;

template <typename T>
static inline int sgn(T const & v)
{
  return (v-T(0))>0 - (v-T(0))<0;
}

MapManager::MapManager(int hsize, double resolution, RobotGeometry const & rg, std::vector<Obstacle> const & globalmap):
  rg(rg), hsize(hsize), init(false), resolution(resolution), origin({0.,0})
{
  initGlobalMap(globalmap);
}

void
MapManager::initGlobalMap(std::vector<Obstacle> const & globalmap)
{
  global_map.pose = {0., 0., 0.};
  if (globalmap.empty()) return;

  // setup matrix
  double xmin, ymin, xmax, ymax;

  xmin = ymin = std::numeric_limits<double>::infinity();
  xmax = ymax = -std::numeric_limits<double>::infinity();
  for (auto obst: globalmap)
  {
    for (auto pt: obst)
    {
      xmin = std::min(xmin, pt.x);
      ymin = std::min(ymin, pt.y);
      xmax = std::max(xmax, pt.x);
      ymax = std::max(ymax, pt.y);
    }
  }

  int rows = std::ceil((ymax-ymin)/resolution);
  int cols = std::ceil((xmax-xmin)/resolution);
  global_map.map = cv::Mat(rows, cols, CV_8U, EMPTY);
  global_map.pose = {xmin, ymin, 0.};

  // fill
  for (auto obst: globalmap)
  {
    for (int i=1; i<obst.size(); ++i)
    {
      double x0 = obst[i-1].x-xmin; 
      double y0 = obst[i-1].y-ymin; 
      double x1 = obst[i].x-xmin; 
      double y1 = obst[i].y-ymin; 
      int c0 = x0/resolution + sgn(x0)*0.5;
      int r0 = y0/resolution + sgn(y0)*0.5;
      int c1 = x1/resolution + sgn(x1)*0.5;
      int r1 = y1/resolution + sgn(y1)*0.5;
      cv::LineIterator line_iter(global_map.map, cv::Point(c0,r0), cv::Point(c1,r1));
      for (int j=0; j<line_iter.count; ++j, ++line_iter)
      {
        const double RADIUS = std::ceil(SCAN_RADIUS/resolution);
        cv::circle(global_map.map, line_iter.pos(), RADIUS, FULL, -1);
      }
    }
  }
}

void
MapManager::update(ScanData const & scan)
{
  PosedMap map = createMap(scan);
  raw_map = map.map.clone();
  storeRawMap(map);
  process(map, static_map, dynamic_map); // comparison with previous maps, filtering
  storeMap(map); // add map to queue
  origin = {scan.pose.x, scan.pose.y};
  updateCostmap();
  init = true;
}



cv::Mat
MapManager::getGlobalView(int rmin, int cmin, int rmax, int cmax)
{
  //dbg.width = dbg.height = 0;

  assert(rmax-rmin==2*hsize && cmax-cmin==2*hsize);
  int size = 2*hsize+1;
  cv::Mat res(size, size, CV_8U, EMPTY);

  int grows = global_map.map.rows;
  int gcols = global_map.map.cols;

  if (rmin>=grows || rmax<0)
    return res;
  if (cmin>=gcols || cmax<0)
    return res;

  cv::Rect window(0, 0, size, size);
  if (rmax>=grows)
  {
    window.height -= rmax-grows+1;
  }
  if (cmax>=gcols)
  {
    window.width -= cmax-gcols+1;
  }
  if (rmin<0)
  {
    window.y -= rmin;
    window.height += rmin;
  }
  if (cmin<0)
  {
    window.x -= cmin;
    window.width += cmin;
  }

  cv::Mat global_map_roi = global_map.map(cv::Rect(cmin+window.x, rmin+window.y, window.width, window.height));
  cv::Mat res_roi = res(window);
  global_map_roi.copyTo(res_roi);

  return res;
}

void
MapManager::updateCostmap()
{
  double xMinLoc, yMinLoc;
  getWorldCoordinatesAbs(0, 0, xMinLoc, yMinLoc);

  int rMinGlob, cMinGlob, rMaxGlob, cMaxGlob;
  getGlobalMapCoordinates(xMinLoc, yMinLoc, rMinGlob, cMinGlob);
  rMaxGlob = rMinGlob + 2*hsize;
  cMaxGlob = cMinGlob + 2*hsize;

  /* copy all matrix from (rMinGlob, cMinGlob) to (rMaxGlob, cMaxGlob) */
  // -> same size and alignment of local map
  cv::Mat global_view = getGlobalView(rMinGlob, cMinGlob, rMaxGlob, cMaxGlob);

  //int buffer_size = std::ceil(rg.radius/resolution);
  //cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(buffer_size, buffer_size));
  //cv::dilate(global_view | static_map | dynamic_map, costmap, element);
  //cv::dilate(global_view, costmap, element); // TODO: for debugging, use only global, static map
  //costmap = cv::Mat(2*hsize+1, 2*hsize+1, CV_8U, EMPTY);
  //costmap = raw_map;
  //costmap = static_map | dynamic_map ; //| global_view;
  if (!raw_maps.empty()) {

      double dx = origin.x - raw_maps.front().pose.x;
      double dy = origin.y - raw_maps.front().pose.y;
      cv::Mat predicted = translate(raw_maps.front().map, -dx/resolution, -dy/resolution);
      costmap = predicted;
      for (int i=1; i<raw_maps.size(); ++i) {
          double dx = origin.x - raw_maps[i].pose.x;
          double dy = origin.y - raw_maps[i].pose.y;
          cv::Mat predicted = translate(raw_maps[i].map, -dx/resolution, -dy/resolution);
          costmap |= predicted;
      }
  }
  else {
      costmap = cv::Mat();
      std::cerr << "EMPTY" << std::endl;
  }
  int kSize = std::ceil(0.2/resolution);
  cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
                                          cv::Size(2*kSize+1, 2*kSize+1),
                                          cv::Point(kSize, kSize));
  //cv::dilate(costmap, costmap, element);
}


void
MapManager::storeMap(PosedMap const & map)
{
  maps_queue.push_back(map);
  if (maps_queue.size() > MAX_HISTORY)
    maps_queue.erase(maps_queue.begin());
}

void
MapManager::storeRawMap(PosedMap const & map)
{
  raw_maps.push_back(map);
  if (raw_maps.size() > MAX_HISTORY)
    raw_maps.erase(raw_maps.begin());
}

PosedMap
MapManager::createMap(ScanData const & scan)
{
  lidarPoint.clear();
  PosedMap map;
  map.map = cv::Mat(2*hsize+1, 2*hsize+1, CV_8U, EMPTY);
  map.pose = scan.pose;
  for (int i=0; i<scan.data.size(); ++i) {
    for (auto const & datum: scan.data[i].datum)
    {
      if (datum.distance>=MIN_THRESHOLD && datum.distance<=MAX_THRESHOLD) {
        double L = std::hypot(datum.y, datum.x);
        double theta = std::atan2(datum.y, datum.x);
        theta += scan.pose.theta;
        update(map, L*cos(theta), L*sin(theta));
      }
    }
  }
  return map;
}

void 
MapManager::process(PosedMap & map, cv::Mat & static_obst, cv::Mat & dynamic_obst)
{
  static_obst = cv::Mat(map.map.rows, map.map.cols, CV_8U, cv::Scalar(0));
  dynamic_obst = cv::Mat(map.map.rows, map.map.cols, CV_8U, cv::Scalar(0));

  std::vector<std::vector<cv::Point>> contours, filtered_contours;

  // Remove small blobs
  cv::findContours(map.map, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // find external contours of each blob
  for (auto c: contours)
  {
    if (cv::contourArea(c) >= MIN_AREA)
      filtered_contours.push_back(c);
  }
  cv::Mat filtered_map(map.map.rows, map.map.cols, CV_8U, cv::Scalar(0));
  cv::drawContours(filtered_map, filtered_contours, -1, cv::Scalar(255), -1);
  //map.map = filtered_map;

  if (maps_queue.empty())
    return;

  double dx = map.pose.x - maps_queue.front().pose.x;
  double dy = map.pose.y - maps_queue.front().pose.y;
  cv::Mat predicted = translate(maps_queue.front().map, -dx/resolution, -dy/resolution);
  cv::Mat overlap = predicted & map.map;

  for (auto c: filtered_contours)
  {
    cv::Mat c_mat = cv::Mat(map.map.rows, map.map.cols, CV_8U, cv::Scalar(0));
    cv::drawContours(c_mat, std::vector<std::vector<cv::Point>>{c}, -1, cv::Scalar(255), -1);
    int static_cnt = cv::countNonZero(c_mat & (overlap & filtered_map));
    int dynamic_cnt = cv::countNonZero(c_mat & (c_mat - (overlap & filtered_map))); //cv::contourArea(c) - static_cnt;
    if (((double)static_cnt)/dynamic_cnt >= MIN_RATIO)
    {
      static_obst.setTo(255, c_mat);
    }
    else
    {
      dynamic_obst.setTo(255, c_mat);
    }
  }

  map.map = filtered_map;
}

cv::Mat
MapManager::translate(cv::Mat const & map, double dx, double dy)
{
  cv::Mat result;
  cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, dx, 0, 1, dy);
  warpAffine(map, result, trans_mat, map.size(), CV_INTER_LINEAR, cv::BORDER_CONSTANT, 0);
  return result;
}

void 
MapManager::update(PosedMap & map, double x, double y)
{
  int r, c;
  bool inside = getMapCoordinates(x, y, r, c);
  if (!inside) return;

  //const double RADIUS = std::ceil(SCAN_RADIUS/resolution);
  //cv::circle(map.map, cv::Point(c,r), RADIUS, FULL, -1);
  map.map.at<uchar>(r, c) = FULL;

  lidarPoint.push_back(cv::Point2f(c,r));
}

void 
MapManager::reset(int hsize, double resolution)
{
  this->hsize = hsize;
  this->resolution = resolution;
  maps_queue.clear();
  raw_maps.clear();
}

int
MapManager::getSize() {
  return hsize;
}

Point2d
MapManager::getPose(){
  return origin;
}

bool 
MapManager::getMapCoordinates(double x, double y, int & r, int & c)
{
  //c = x/resolution + sgn(x)*0.5 + hsize;
  //r = y/resolution + sgn(y)*0.5 + hsize;
  c = x/resolution + hsize;
  r = y/resolution + hsize;
  return c>=0 && c<=2*hsize && r>=0 && r<=2*hsize;
}

bool 
MapManager::getMapCoordinatesAbs(double x, double y, int & r, int & c)
{
  x -= origin.x;
  y -= origin.y;
//  c = x/resolution + sgn(x)*0.5 + hsize;
//  r = y/resolution + sgn(y)*0.5 + hsize;
  c = x/resolution + hsize;
  r = y/resolution + hsize;
  return c>=0 && c<=2*hsize && r>=0 && r<=2*hsize;
}

void
MapManager::getGlobalMapCoordinates(double x, double y, int & r, int & c)
{
  x -= global_map.pose.x;
  y -= global_map.pose.y;
  c = x/resolution; // + sgn(x)*0.5;
  r = y/resolution; // + sgn(y)*0.5;
}

bool 
MapManager::getWorldCoordinates(int r, int c, double & x, double & y)
{
  x = (c - hsize) * resolution;
  y = (r - hsize) * resolution;
  return c>=0 && c<=2*hsize && r>=0 && r<=2*hsize;
}

bool 
MapManager::getWorldCoordinatesAbs(int r, int c, double & x, double & y)
{
  x = (c - hsize) * resolution + origin.x;
  y = (r - hsize) * resolution + origin.y;
  return c>=0 && c<=2*hsize && r>=0 && r<=2*hsize;
}

void
MapManager::drawLidarPoints(std::vector<Point2d> const & lidarScan)
{
    PosedMap map;
    map.map = cv::Mat(2*hsize+1, 2*hsize+1, CV_8U, EMPTY);

    for (int i=0;i<lidarScan.size();i++){
        int r1, c1;
        bool inside = getMapCoordinates(lidarScan[i].x, lidarScan[i].y, r1, c1);
        const double RADIUS = std::ceil(SCAN_RADIUS/resolution);
        cv::circle(map.map, cv::Point(c1,r1), 1, FULL, -1);
    }

    raw_map = map.map.clone();
    updateCostmap();
    init = true;
}

cv::Mat
MapManager::generateImage()
{
//   cv::Mat result(costmap.rows, costmap.cols, CV_8UC3, cv::Scalar(0,0,0));
//   result.setTo(cv::Scalar(0,0,255), static_map);
//   result.setTo(cv::Scalar(0,255,0), dynamic_map);

  cv::Mat result;

  if (!costmap.empty())
    cvtColor(costmap, result, CV_GRAY2BGR);
  //cvtColor(global_map.map, result, CV_GRAY2BGR);
  //result.setTo(cv::Scalar(0,0,255), costmap);
  
  return result;
}

const PosedMap &
MapManager::getGlobalMap()
{
  return global_map;
}
  
