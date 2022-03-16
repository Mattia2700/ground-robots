#include "planner.hpp"

#include "fmm.hpp"
#include "fm2.hpp"

#include "asyplot.hpp"
#include "timeperf.hpp"
#include "wayline_extractor.hpp"
#include "dp_smoothing.hpp"


#include <ClothoidList.hh>
#include <ClothoidAsyPlot.hh>

#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>

using std::cout;
using std::cerr;
using std::endl;

using G2lib::ClothoidCurve;
using G2lib::ClothoidList;


void 
FMPlanner::setMap(Map const * map) {
  this->map = map;
  mapUpdate();
}

void
FMPlanner::mapUpdate() {
  cv::Mat img;
  int kSize = std::ceil(0.375/map->resolution);
  cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
                                          cv::Size(2*kSize+1, 2*kSize+1),
                                          cv::Point(kSize, kSize));
  cv::dilate(this->map->img, img, element);
  //img = this->map->img.clone();
  cv::rectangle(img, cv::Rect(cv::Point(0, 0), this->map->img.size()), 255, 1); // set image borders as obstacle


  this->F = std::make_unique<Grid<real_type>>(img.rows, img.cols);
  Grid<real_type> & F = *this->F;
  for (int i=0; i<img.rows; ++i)
    for (int j=0; j<img.cols; ++j)
      F(i,j) = 1. - img.at<uchar>(i,j)/255.;
  F.h = map->resolution;
}

bool
FMPlanner::plan(State2d const & start, State2d const & goal, vector<DPSmoothing::Curve> & path, bool goalRegion) {
  idx_type is, js;
  map->world2map(start.x, start.y, is, js);

  idx_type ig, jg;
  map->world2map(goal.x, goal.y, ig, jg);

  TimePerf tp;
  tp.start();
  FM2 fm2;
  FM2::Result res = fm2.build(*F, F->rc2idx(ig, jg), F->rc2idx(is, js));
  //std::cerr << "PLANNING TIME: " << tp.getTime() << "ms" << std::endl;

  // real_type m = 0;
  // for (int i=0; i<res.T.size(); ++i) m = std::max(m, std::isinf(res.T(i))?0:res.T(i));
  // cv::Mat img(res.T.rows(), res.T.cols(), CV_8UC1);
  // for (int i=0; i<res.T.rows(); ++i)
  //   for (int j=0; j<res.T.cols(); ++j)
  //     img.at<uchar>(i,j) = (uchar)(std::round(res.T(i,j)*255./m));
  // cv::imshow("T", img);
  // cv::waitKey(0);
  

  tp.start();
  WaylineExtractor we;
  //vector<Wayline> waylines = we.extractwaylines(*F, res, is, js, start.theta, ig, jg, goal.theta);
  waylines = we.extractwaylines(*F, res, is, js, start.theta, ig, jg, goal.theta);

  if (waylines.empty()) {
    cerr << "NO SOLUTION FOUND" << endl;
    return false;
  }

  using Curve = DPSmoothing::Curve;
  vector<Curve> vcc;
  DPSmoothing smoothing(&res.V);
  if (!smoothing.optimise(waylines, start.theta, goal.theta, vcc)) { 
    cerr << "NO [SMOOTH] SOLUTION FOUND" << endl;
    return false;
  }
  //std::cerr << "SMOOTHING TIME: " << tp.getTime() << "ms" << std::endl;


  path.clear();
  real_type xv = start.x, yv = start.y;
  for (Curve const & cc: vcc) {
    path.push_back(cc);
    path.back().scale(map->resolution);
    path.back().changeOrigin(xv, yv);
    xv = path.back().xEnd();
    yv = path.back().yEnd();
  }
  
  return true;
}

bool
FMPlanner::planRaw(State2d const & start, State2d const & goal, vector<State2d> & path) {
  idx_type is, js;
  map->world2map(start.x, start.y, is, js);

  idx_type ig, jg;
  map->world2map(goal.x, goal.y, ig, jg);

  FM2 fm2;
  FM2::Result res = fm2.build(*F, F->rc2idx(ig, jg), F->rc2idx(is, js));

  // real_type m = 0;
  // for (int i=0; i<res.T.size(); ++i) m = std::max(m, std::isinf(res.T(i))?0:res.T(i));
  // cv::Mat img(res.T.rows(), res.T.cols(), CV_8UC1);
  // for (int i=0; i<res.T.rows(); ++i)
  //   for (int j=0; j<res.T.cols(); ++j)
  //     img.at<uchar>(i,j) = (uchar)(std::round(res.T(i,j)*255./m));
  // cv::imshow("T", img);
  // cv::waitKey(0);

  WaylineExtractor we;
  path = we.extractwaypoints(*F, res, is, js, start.theta, ig, jg, goal.theta);
  for (int i=0; i<path.size(); ++i) {
    map->map2world(path[i].y, path[i].x, path[i].x, path[i].y);
  }

  return !path.empty();
}



