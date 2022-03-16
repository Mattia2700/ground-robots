#pragma once

#include "fm2.hpp"
#include "pathextractor.hpp"
#include "gradient.hpp"
#include "utils.hpp"
#include <ClothoidAsyPlot.hh>

#include <opencv2/opencv.hpp>

// struct Waypoint {
//   real_type x, y, theta;
// };

// struct Wayline {
//   real_type x0, y0;
//   real_type x1, y1;
// };

// inline
// std::vector<std::pair<int,int>> toInt(std::vector<std::pair<real_type,real_type>> const & in) {
//   std::vector<std::pair<int,int>> out;
//   out.reserve(in.size());
//   for (auto const & p: in) {
//     out.emplace_back(std::round(p.first), std::round(p.second));
//   }
//   return out;
// }

// inline
// void cvdraw1(Grid<real_type> const & T, idx_type is, idx_type js, idx_type ig, idx_type jg, std::vector<std::pair<int,int>> path = {}, int size = 1, std::vector<Wayline> const & waylines = {}) {
//   cv::Mat img(T.rows(), T.cols(), CV_8UC3);
//   double M = 0;
//   for (int i=0; i<T.size(); ++i) { M = std::max(M, isinf(T(i))?0:T(i)); }
//   for (int i=0; i<img.rows; ++i) {
//     for (int j=0; j<img.cols; ++j) {
//       uchar val = T(i,j)*255./M;
//       //uchar val = std::min(T(i,j)/M*255, 255.);
//       //uchar val = isinf(T(i,j)) ? 0 : 255;
//       img.at<cv::Vec3b>(i,j)[0] = img.at<cv::Vec3b>(i,j)[1] = img.at<cv::Vec3b>(i,j)[2] = val;
//     }
//   }
  
//   cv::circle(img, {js,is}, 5, cv::Scalar(0,0,255), -1);
//   cv::circle(img, {jg,ig}, 5, cv::Scalar(255,0,0), -1);

//   for (auto pt: path) {
//     int i = pt.second, j = pt.first;
//     if (size == 1) {
//       img.at<cv::Vec3b>(i,j)[0] = 0;
//       img.at<cv::Vec3b>(i,j)[1] = 255;
//       img.at<cv::Vec3b>(i,j)[2] = 0;
//     }
//     else {
//       cv::circle(img, {j,i}, size, cv::Scalar(0,255,0), -1);
//     }
//   }

//   for (auto w: waylines) {
//     cv::line(img, {w.x0,w.y0}, {w.x1,w.y1}, cv::Scalar(200,200,0));
//   }

//   cv::imshow("win", img);
//   cv::waitKey(0);
// }

class WaylineExtractor {
public:
  inline
  WaylineExtractor(int npts = 3, real_type Lmax = 1.2) : npts(npts), Lmax(Lmax) {}
  
  inline
  vector<Wayline> extractwaylines(Grid<real_type> const & grid, FM2::Result const & res, idx_type is, idx_type js, real_type theta_s, idx_type ig, idx_type jg, real_type theta_g, bool goalRegion = false);

  inline
  vector<State2d> extractwaypoints(Grid<real_type> const & grid, FM2::Result const & res, idx_type is, idx_type js, real_type thetas, idx_type ig, idx_type jg, real_type thetag);

private:
  inline 
  Wayline trace(Grid<real_type> const & grid, State2d const & wp);

  int npts;
  real_type Lmax;

};

vector<Wayline>
WaylineExtractor::extractwaylines(Grid<real_type> const & grid, FM2::Result const & res, idx_type is, idx_type js, real_type theta_s, idx_type ig, idx_type jg, real_type theta_g, bool goalRregion) {
  //std::vector<Point2d> path = extractpath(res.parents, res.parents.rc2idx(is, js));
  std::vector<Point2d> path = gradientDescent(res.T, res.T.rc2idx(is, js));
  if (path.size()<2) return {};

  std::vector<real_type> accl;
  accl.push_back(0.);
  for (int i=1; i<path.size(); ++i) {
    real_type dx = path[i].x-path[i-1].x;
    real_type dy = path[i].y-path[i-1].y;
    real_type len = std::hypot(dx, dy);
    accl.push_back(accl.back()+len);
  }
  
  const real_type DL = 1.0/grid.h; // TODO: refactor
  int ncuts = std::ceil(accl.back()/DL);
  real_type dl = accl.back()/ncuts;
  std::vector<State2d> waypoints;
  for (int i=0; i<=ncuts; ++i) {
    double lcur = i*dl;
    int idx = 0;
    while (idx<accl.size() && lcur>accl[idx]) ++idx;
    if (idx==accl.size()) {
      // TODO: fix initial/final angle
      waypoints.push_back({path.back().x, path.back().y, theta_g});
    }
    else if (idx==0) {
      waypoints.push_back({path.front().x, path.front().y, theta_s});
    }
    else {
      real_type lprev = accl[idx-1];
      real_type lseg = lcur-lprev;
      real_type xp = path[idx-1].x;
      real_type yp = path[idx-1].y;
      real_type dx = path[idx].x-xp;
      real_type dy = path[idx].y-yp;
      real_type perc = lseg/(accl[idx]-accl[idx-1]);
      //real_type theta = std::atan2(dy,dx);
      real_type theta = 0;
      waypoints.push_back({xp+perc*dx, yp+perc*dy, theta});
    }
  }

  if (waypoints.size()>4) {
    // std::vector<State2d> tmp;
    // tmp.reserve(waypoints.size());
    int imin = 2; // keep first two waypoints
    int imax = waypoints.size()-3; // keep last two waypoints

    std::set<int> indexes;

    // tmp.push_back(waypoints[0]);
    // tmp.push_back(waypoints[1]);
    indexes.insert(0);
    indexes.insert(1);

    int idx1 = 1;
    int idx2 = 2;
    State2d p1 = waypoints[1];
    State2d p2 = waypoints[2];
  
    for(unsigned int i=3; i<=waypoints.size()-2; ++i)
    {
      real_type u1x = p2.x - p1.x;
      real_type u1y = p2.y - p1.y;
      real_type l1  = std::hypot(u1x,u1y);
      u1x /= l1; u1y /= l1;

      int idx3 = i;
      State2d p3 = waypoints[i];
      real_type u2x = p3.x - p2.x;
      real_type u2y = p3.y - p2.y;
      real_type l2  = std::hypot(u2x,u2y);
      u2x /= l2; u2y /= l2;
    
      real_type prod = u1x*u2x + u1y*u2y;
      if (fabs(1. - fabs(prod)) >= 5e-2)
      {
        // tmp.push_back(p2);
        indexes.insert(idx1);
        indexes.insert(idx2);
        indexes.insert(idx3);
      }

      p1 = p2;
      p2 = p3;
      idx1 = idx2;
      idx2 = idx3;
    }
    // tmp.push_back(p2);
    // tmp.push_back(waypoints.back());
    indexes.insert(idx2);
    indexes.insert(waypoints.size()-1);

    vector<State2d> tmp;
    tmp.reserve(indexes.size());
    for (int i: indexes) {
      tmp.push_back(waypoints[i]);
    }
    
    waypoints = tmp;
  }

  int ne  = waypoints.size()-1;
  int ne1 = waypoints.size()-2;
  real_type const m_2pi = 6.28318530717958647692528676656;  // 2*pi
  vector<real_type> omega(ne);
  vector<real_type> len(ne);
  real_type dx = waypoints[1].x-waypoints[0].x;
  real_type dy = waypoints[1].y-waypoints[0].y;
  omega[0] = std::atan2(dy,dx); // theta_s; 
  len[0]   = std::hypot(dy,dx);
  for (int j=1; j<ne; ++j) {
    dx       = waypoints[j+1].x-waypoints[j].x;
    dy       = waypoints[j+1].y-waypoints[j].y;
    omega[j] = std::atan2(dy,dx);
    len[j]   = std::hypot(dy,dx);
    real_type domega = omega[j]-omega[j-1];
    domega  -= round(domega/m_2pi)*m_2pi;
    omega[j] = omega[j-1]+domega;
  }

  waypoints[0].theta = omega[0];
  waypoints[ne].theta = omega[ne1];
  
  real_type omega_L = omega[0];
  real_type len_L   = len[0];
  for (int j=1; j<ne; ++j) {
    real_type omega_R = omega[j];
    real_type len_R   = len[j];
    waypoints[j].theta = (omega_L/len_L + omega_R/len_R) / (1/len_L + 1/len_R);
    omega_L = omega_R;
    len_L   = len_R;
  }

  //waypoints.front().theta = theta_s;
  //waypoints.back().theta = theta_g;

  // for (auto wp: waypoints) std::cout << wp.first << " " << wp.second << std::endl;
  // cvdraw1(grid, is, js, ig, jg, toInt(waypoints), 2);
  // std::cout << waypoints.size() << std::endl;
  
  vector<Wayline> waylines;
  waylines.emplace_back(waypoints[0].x, waypoints[0].y, waypoints[0].theta+M_PI/2., 0., 0);
  int imax = goalRregion ? waypoints.size() : waypoints.size()-1;
  for (int i=1; i<imax; ++i) {
    auto const & wp = waypoints[i];
    Wayline w = trace(res.V, wp);
    waylines.push_back(w);
  }
  if (!goalRregion) {
    waylines.emplace_back(waypoints.back().x, waypoints.back().y, waypoints.back().theta+M_PI/2., 0., 0);
  }

  //   {
  //   G2lib::AsyPlot plot("waylines.asy", false);
  //   for (int i=1; i<waypoints.size(); ++i) {
  //     plot.drawLine(waypoints[i-1].x, waypoints[i-1].y, waypoints[i].x, waypoints[i].y);
  //   }

  //   for (int i=0; i<waylines.size(); ++i) {
  //     real_type x0, y0, x1, y1;
  //     x0 = waylines[i].xc - waylines[i].L*std::cos(waylines[i].theta);
  //     y0 = waylines[i].yc - waylines[i].L*std::sin(waylines[i].theta);
  //     x1 = waylines[i].xc + waylines[i].L*std::cos(waylines[i].theta);
  //     y1 = waylines[i].yc + waylines[i].L*std::sin(waylines[i].theta);
  //     plot.drawLine(x0, y0, x1, y1, "green");
  //   }
  // }

  //exit(0);
  // std::cout << waylines.size() << std::endl;
  // cvdraw1(grid, is, js, ig, jg, toInt(path), 2, waylines);
  return waylines;
}


Wayline 
WaylineExtractor::trace(Grid<real_type> const & grid, State2d const & wp) {
  real_type x0 = wp.x, y0 = wp.y;
  real_type theta = wp.theta + M_PI/2;
  real_type dl = 1;
  for (real_type Lcur = dl; Lcur<Lmax/grid.h; Lcur += dl) {
    real_type x = wp.x + Lcur*std::cos(theta);
    real_type y = wp.y + Lcur*std::sin(theta);
    if (grid(grid.rc2idx(y, x))>0.1) {
      x0 = x; y0 = y;
    }
    else break;
  }
  real_type x1 = wp.x, y1 = wp.y;
  for (real_type Lcur = dl; Lcur<Lmax/grid.h; Lcur += dl) {
    real_type x = wp.x + Lcur*std::cos(theta+M_PI);
    real_type y = wp.y + Lcur*std::sin(theta+M_PI);
    if (grid(grid.rc2idx(y, x))>0.1) {
      x1 = x; y1 = y;
    }
    else break;
  }

  real_type xc = 0.5*(x0+x1);
  real_type yc = 0.5*(y0+y1);
  real_type L = 0.5*hypot(x1-x0, y1-y0);
  return {xc, yc, theta, L, npts};
}

std::vector<State2d>
WaylineExtractor::extractwaypoints(Grid<real_type> const & grid, FM2::Result const & res, idx_type is, idx_type js, real_type thetas, idx_type ig, idx_type jg, real_type thetag) {
  //std::vector<Point2d> path = extractpath(res.parents, res.parents.rc2idx(is, js));
  std::vector<Point2d> path = gradientDescent(res.T, res.T.rc2idx(is, js));

  if (path.empty()) return {};

  // std::vector<std::pair<idx_type,idx_type>> coords;
  // for (auto pt: path) {
  //   coords.emplace_back(pt.second, pt.first);
  // }
  //cvdraw1(grid, is, js, ig, jg, coords);

  std::vector<real_type> accl;
  accl.push_back(0.);
  for (int i=1; i<path.size(); ++i) {
    real_type dx = path[i].x-path[i-1].x;
    real_type dy = path[i].y-path[i-1].y;
    real_type len = std::hypot(dx, dy);
    accl.push_back(accl.back()+len);
  }
  
  const real_type DL = 1.0/grid.h;
  int ncuts = std::ceil(accl.back()/DL);
  real_type dl = accl.back()/ncuts;
  std::vector<State2d> waypoints;
  for (int i=0; i<=ncuts; ++i) {
    double lcur = i*dl;
    int idx = 0;
    while (idx<accl.size() && lcur>accl[idx]) ++idx;
    if (idx==accl.size()) {
      waypoints.push_back({path.back().x, path.back().y, thetag});
    }
    else if (idx==0) {
      waypoints.push_back({path.front().x, path.front().y, thetas});
    }
    else {
      real_type lprev = accl[idx-1];
      real_type lseg = lcur-lprev;
      real_type xp = path[idx-1].x;
      real_type yp = path[idx-1].y;
      real_type dx = path[idx].x-xp;
      real_type dy = path[idx].y-yp;
      real_type perc = lseg/(accl[idx]-accl[idx-1]);
      real_type theta = std::atan2(dy,dx);
      waypoints.push_back({xp+perc*dx, yp+perc*dy, theta});
    }
  }
  waypoints.front().theta = thetas;
  waypoints.back().theta = thetag;

  return waypoints;
}
