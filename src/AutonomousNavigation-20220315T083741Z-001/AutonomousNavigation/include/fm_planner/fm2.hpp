#pragma once

#include "grid.hpp"
#include "fmm.hpp"

#include "timeperf.hpp"

#include <vector>

class FM2 {
public:
  struct Result {
    Grid<real_type> V;
    Grid<real_type> T;
  };

  FM2()
  {}

  Result build(Grid<real_type> const & F, 
               idx_type const & goal,
               idx_type const & start);
    
};

FM2::Result
FM2::build(Grid<real_type> const & F, 
           idx_type const & goal,
           idx_type const & start) {

  Result res;
  std::vector<idx_type> seeds;
  for (int i=0; i<F.size(); ++i) {
    if (F(i)==0) { seeds.push_back(i); }
  }

  FMM fmm;
  res.V = fmm.build(F, seeds);
  real_type M = 0;
  for (int i=0; i<res.V.size(); ++i) {
    if (std::isinf(res.V(i))) { continue; }
    M = std::max(M, res.V(i));
      //std::cerr << T1(i,j) << std::endl;
  } 

  real_type MAX_DISTANCE = 0.1;
  //real_type vMax = MAX_DISTANCE/T1.h;
  real_type vMax = std::min(MAX_DISTANCE, M);
  for (int i=0; i<res.V.size(); ++i) {
    // res.V(i) /= M;
    // if (res.V(i)>=vMax) res.V(i) = 1;
    // else res.V(i) /= vMax;
    res.V(i) = res.V(i)>=vMax ? 1 : res.V(i)/vMax;
  }

  //res.T = fmm.build(res.V, {goal});
  res.T = fmm.build(res.V, {goal}, start, false);
  return res;
}
