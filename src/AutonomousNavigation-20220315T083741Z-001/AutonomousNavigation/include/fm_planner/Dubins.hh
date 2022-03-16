#pragma once

#include "dubins.h"
#include "utils.hpp"

#include <ClothoidList.hh>


class DubinsCurve {
public:
  inline
  void build_G1(real_type x0, real_type y0, real_type th0, real_type x1, real_type y1, real_type th1, real_type radius) {
    double q0[3] = {x0,y0,th0};
    double q1[3] = {x1,y1,th1};
    dubins_shortest_path(&path, q0, q1, radius);
  }
  
  inline 
  real_type length() const {
    return dubins_path_length(&path);
  }

  inline
  void eval(real_type s, real_type & x, real_type & y) const {
    double q[3];
    dubins_path_sample(&path, s, q);
    x = q[0]; y = q[1];
  }

  inline
  void eval(real_type s, real_type & x, real_type & y, real_type & theta) const {
    double q[3];
    dubins_path_sample(&path, s, q);
    x = q[0]; y = q[1]; theta = q[2];
  }

  inline
  void evaluate(real_type s, real_type & theta, real_type & k0, real_type & x, real_type & y) const {
    eval(s, x, y, theta);
    k0 = kappa(s);
  }

  inline
  real_type xEnd() const {
    real_type x, y;
    eval(length(), x, y);
    return x;
  }

  inline
  real_type yEnd() const {
    real_type x, y;
    eval(length(), x, y);
    return y;
  }

  inline
  real_type kappaSegment(int i) const {
    DubinsPathType type = dubins_path_type(&path);
    real_type KAPPA_MAX = 1./path.rho;
    switch (type) {
    case LSL:
      return (i==0 || i==2) ? KAPPA_MAX : 0;
    case LSR:
      return (i==0) ? KAPPA_MAX : ((i==2) ? -KAPPA_MAX : 0);
    case RSL:
      return (i==0) ? -KAPPA_MAX : ((i==2) ? KAPPA_MAX : 0);
    case RSR:
      return (i==0 || i==2) ? -KAPPA_MAX : 0;
    case LRL:
      return (i==0 || i==2) ? KAPPA_MAX : -KAPPA_MAX;
    case RLR:
      return (i==0 || i==2) ? -KAPPA_MAX : KAPPA_MAX;
    }
    return 0;
  }

  inline
  real_type kappa(real_type s) const {
    int i;
    for (i=0; i<=1; ++i) {
      real_type L = dubins_segment_length(&path, i);
      if (s>L) s -= L;
      else break;
    }
    return kappaSegment(i);
  }

  inline
  real_type dkappa() const {
    return 0.;
  }

  inline
  G2lib::ClothoidList toClothoidList() const {
    G2lib::ClothoidList res;
    real_type Ltot = 0;
    for (int i=0; i<3; ++i) {
      real_type x0, y0, theta0;
      eval(Ltot, x0, y0, theta0);
      real_type kappa0 = kappaSegment(i);
      real_type L = dubins_segment_length(&path, i);
      res.push_back(G2lib::ClothoidCurve(x0, y0, theta0, kappa0, 0., L));
      Ltot += L;
    }
    return res;
  }

  inline
  real_type integralCurvature2() const {
    real_type res = 0;

    DubinsPathType type = dubins_path_type(&path);
    real_type KMAX = 1/path.rho;
    for (int i=0; i<=2; ++i) {
      real_type L = dubins_segment_length(&path, i);
      real_type k0 = (i==1 && type!=LRL && type!=RLR) ? 0 : KMAX;
      res += L*k0*k0;
    }
    return res;
  }

  inline
  void scale(real_type s) {
    path.rho *= s;
  }

  inline
  void changeOrigin(real_type newx0, real_type newy0) {
    path.qi[0] = newx0;
    path.qi[1] = newy0;
  }

private:
  DubinsPath path;
};
