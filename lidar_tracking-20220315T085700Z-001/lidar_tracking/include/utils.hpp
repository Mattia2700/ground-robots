#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "robotstatus.h"

typedef double real_type;

struct Point2d {
  Point2d() {}
  Point2d(real_type x, real_type y): x(x), y(y) {}

  real_type x, y;
};

struct Pose {
  real_type x, y, theta;
};

typedef Pose State2d;

typedef std::vector<Point2d> Obstacle;

struct ScanData
{
  std::vector<RobotStatus::LidarData> data;
  Pose pose;
};


struct Wayline {
  real_type xc, yc;
  real_type theta;
  real_type L;
  size_t npts;

  //vector<real_type> _x, _y;

//public:
  Wayline(real_type xc, 
          real_type yc,
          real_type theta,
          real_type L,
          int npts):
    xc(xc), yc(yc), theta(theta), L(L), npts(npts)        
  {}

  size_t size() const {
    return 2*npts + 1;
  }

  
  real_type x(int idx) const {
    if (!npts) return xc;

    real_type ds = L/npts;
    real_type cos_theta = cos(theta);
    real_type sin_theta = sin(theta);
    
    // 0 ->  0
    // 1 ->  1, 2 -> -1
    // 3 ->  2, 4 -> -2
    // 5 ->  3, 6 -> -3
    int sign = idx%2==0 ? -1 : +1;
    idx = (idx+1)/2;
    real_type cl = sign*idx*ds;
    return xc+cl*cos_theta;
  } 
  
  real_type y(int idx) const { 
    if (!npts) return yc;

    real_type ds = L/npts;
    real_type cos_theta = cos(theta);
    real_type sin_theta = sin(theta);
    
    // 0 ->  0
    // 1 ->  1, 2 -> -1
    // 3 ->  2, 4 -> -2
    // 5 ->  3, 6 -> -3
    int sign = idx%2==0 ? -1 : +1;
    idx = (idx+1)/2;
    real_type cl = sign*idx*ds;
    return yc+cl*sin_theta;
  } 

};

struct AngularRange {
  real_type thetac;
  real_type L;
  size_t npts;
  std::vector<real_type> special;

  //vector<real_type> _x, _y;

//public:
  AngularRange(real_type thetac, 
               real_type L,
               int npts):
    thetac(thetac), L(L), npts(npts)        
  {}

  size_t size() const {
    return 2*npts + 1 + special.size();
  }

  real_type theta(int idx) const {
    if (!idx) return thetac;
    if (idx>2*npts) {
      return special[idx-2*npts-1];
    }

    real_type ds = L/npts;
       
    // 0 ->  0
    // 1 ->  1, 2 -> -1
    // 3 ->  2, 4 -> -2
    // 5 ->  3, 6 -> -3
    int sign = idx%2==0 ? -1 : +1;
    idx = (idx+1)/2;
    real_type cl = sign*idx*ds;
    return thetac+cl;
  } 

  void addSpecial(real_type theta) {
    special.push_back(theta);
  }

};

struct Map
{
  double resolution;
  double x0;
  double y0;
  cv::Mat img;

  void map2world(int i, int j, double & x, double & y) const
  {
//    x = (j+0.5)*resolution + x0;
//    y = (i+0.5)*resolution + y0;
    x = (j)*resolution + x0;
    y = (i)*resolution + y0;
  }

  void world2map(double x, double y, int & i, int & j) const
  {
//    i = (y-y0)/resolution+0.5;
//    j = (x-x0)/resolution+0.5;
    i = std::round((y-y0)/resolution);
    j = std::round((x-x0)/resolution);
  }
};

inline
int Signum(double x){
  if(x>0){
    return 1.0;
  }else if(x<0){
    return -1.0;
  }else{
    return 0;
  }
}
