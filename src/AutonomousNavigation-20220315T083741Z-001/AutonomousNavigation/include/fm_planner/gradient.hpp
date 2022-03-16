#pragma once

#include "grid.hpp"
#include "utils.hpp"
#include <iostream>

template <typename T>
T sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

template <typename T>
std::vector<Point2d> gradientDescent(Grid<T> const & Tg, idx_type idx) {
  double step = 1.0;
  
  if (std::isinf(Tg(idx))) {
    std::cerr << "FMM FAILED TO FIND A VALID PATH" << std::endl;
    return {};
  }

  std::vector<Point2d> res;

  T px, py;
  idx_type cx, cy;
  Tg.idx2rc(idx, cy, cx);
  px = cx;
  py = cy;

  res.push_back({px,py});


  int cnt = 0;
  while (Tg(idx) != 0) {
    if (cnt>Tg.size()) {
      std::cerr << "[WARN] GradientDescent: infinite loop" << std::endl;
      return res;
    } 

    //T Tg_00 = Tg(idx+Tg.cols()-1); 
    T Tg_01 = Tg(idx+Tg.cols()); 
    //T Tg_02 = Tg(idx+Tg.cols()+1); 
    T Tg_10 = Tg(idx-1); 
    T Tg_12 = Tg(idx+1); 
    //T Tg_20 = Tg(idx-Tg.cols()-1); 
    T Tg_21 = Tg(idx-Tg.cols()); 
    //T Tg_22 = Tg(idx-Tg.cols()+1); 

    T Tg_l = -Tg_10; //-Tg_00 - 2*Tg_10 - Tg_20;
    T Tg_r =  Tg_12; //Tg_02 + 2*Tg_12 + Tg_22;
    T gx = Tg_l + Tg_r;
    if (std::isinf(Tg_l)) {
      if (Tg_r<Tg(idx)) gx = -1;
      else gx = 0;
    }
    else if (std::isinf(Tg_r)) {
      if (-Tg_l<Tg(idx)) gx = 1;
      else gx = 0;
    }
    // else if (std::isinf(gx)) {
    //   gx = sgn<T>(gx);
    // }
    T max_grad = std::abs(gx);
    
    T Tg_d = -Tg_21; //-Tg_20 - 2*Tg_21 - Tg_22;
    T Tg_u =  Tg_01; //Tg_00 + 2*Tg_01 + Tg_02;
    T gy = Tg_d + Tg_u;
    if (std::isinf(Tg_d)) {
      if (Tg_u<Tg(idx)) gy = -1;
      else gy = 0;
    }
    else if (std::isinf(Tg_u)) {
      if (-Tg_d<Tg(idx)) gy = 1;
      else gy = 0;
    }
    // else if (std::isinf(gy)) {
    //   gy = sgn<T>(gy);
    // }
    if (std::abs(max_grad) < std::abs(gy))
      max_grad = gy;

    if (gx==0 && gy==0) {
      std::cerr << "[WARN] GradientDescent: stuck" << std::endl;
      return {};
    }
    
    px -= step*gx/std::abs(max_grad);
    py -= step*gy/std::abs(max_grad);
    cx = int(px+0.5);
    cy = int(py+0.5);

    idx = Tg.rc2idx(cy, cx);
    //std::cerr << Tg(idx) << " " << px << " " << py << " " << cx << " " << cy << std::endl;
    res.push_back({px,py});

    ++cnt;
  }

  return res;
}