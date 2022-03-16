#pragma once

#include "grid.hpp"
#include <limits>
#include <algorithm>

static const real_type INF = std::numeric_limits<real_type>::infinity();

template <typename T>
class EikonalSolver {
public:
  EikonalSolver() {}
  real_type compute(Grid<T> const & grid, Grid<T> const & F, idx_type idx);

private:
  //void minNeighbour(Grid<T> const & grid, idx_type idx, idx_type r, idx_type c, real_type& min_r, real_type& min_c);  

};

template <typename T>
real_type EikonalSolver<T>::compute(Grid<T> const & grid, Grid<T> const & F, idx_type idx) {
  //idx_type r, c;
  real_type min_r, min_c;
  //grid.idx2rc(idx, r, c);
  //minNeighbour(grid, idx, r, c, min_r, min_c);
  grid.minNeighbour(idx, min_r, min_c, INF);
  std::array<real_type, 2> tt;
  int cnt = 0;
  //if (min_r<INF) tt[cnt++] = min_r;
  tt[cnt] = min_r; cnt += min_r<std::min(INF,grid(idx));
  //if (min_c<INF) tt[cnt++] = min_c;  
  tt[cnt] = min_c; cnt += min_c<std::min(INF,grid(idx));
  if (!cnt) return INF;
  
  // sort by increasing value
  if (cnt>1 && tt[0]>tt[1]) {
    std::swap(tt[0], tt[1]);
  }
  real_type res = tt[0] + grid.h/F(idx);
  if (cnt>1 && res>=tt[1]) {
    real_type a = 2; // Ndim
    real_type b = -2*(tt[0]+tt[1]);
    real_type c = tt[0]*tt[0]+tt[1]*tt[1]-grid.h*grid.h/(F(idx)*F(idx));
    real_type delta = b*b-4*a*c;
    if (delta >= 0) {
      real_type sol =  (-b + sqrt(delta))/(2*a);
      if ((sol-tt[1])>1e-10) 
        res = sol;
    }
  }
  return res;
}

// template <typename T>
// void EikonalSolver<T>::minNeighbour(Grid<T> const & grid, idx_type idx, idx_type r, idx_type c, real_type& min_r, real_type& min_c) {
//     min_r = INF;
//     if (r>0) {
//       min_r = grid(idx-grid.cols());
//     }
//     if (r<grid.rows()-1) {
//       min_r = std::min(min_r, grid(idx+grid.cols()));
//     }
    
//     min_c = INF;
//     if (c>0) {
//       min_c = grid(idx-1);
//     }
//     if (c<grid.cols()-1) {
//       min_c = std::min(min_c, grid(idx+1));
//     }
//   }