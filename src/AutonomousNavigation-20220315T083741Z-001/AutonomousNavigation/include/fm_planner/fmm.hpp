#pragma once

#define MINHEAP_SIMPLE

#include <unordered_set>
#include "grid.hpp"

#ifdef MINHEAP_SIMPLE
#include "minheap_simple.hpp"
#else
//#include "minheap.hpp"
#endif

#include "eikonal.hpp"

#include "timeperf.hpp"

class FMM {
public:
  inline
  FMM()
  {}

  inline
  Grid<real_type> build(Grid<real_type> const & F, 
                        vector<idx_type> const & seeds,
                        idx_type goal = -1,
                        bool heur_time = false
                        );

    
};


Grid<real_type> 
FMM::build(Grid<real_type> const & F, 
           vector<idx_type> const & seeds,
           idx_type goal,
           bool heur_time) {

  int rows = F.rows(), cols = F.cols();
  Grid<real_type> T(rows, cols, INF);
  T.h = F.h;

  //std::unordered_set<idx_type> frozen;
  vector<bool> frozen(rows*cols, false);
  vector<bool> narrow(rows*cols, false);
  MinHeap<idx_type> narrowH(rows*cols);

  EikonalSolver<real_type> es;

  bool heur = goal>=0;
  if (heur) {
    F.updateDistances(goal);
  }

  auto hdistance = [&](idx_type idx) -> real_type { 
    return heur?(heur_time?F.distance(idx)/F(idx):F.distance(idx)):0; 
  };

  for (auto s: seeds) {
    T(s) = 0.;
    narrowH.insert(s, hdistance(s));
    narrow[s] = true;
  }
  
  while (!narrowH.empty()) {
    idx_type x_i;
    
    narrowH.extractMin(x_i);
    
    if (heur && goal==x_i) break;

    #ifdef MINHEAP_SIMPLE
    if (frozen[x_i]) continue;
    #endif
    narrow[x_i] = false;
    frozen[x_i] = true;

    int cnt;
    auto res = F.neighbours(x_i, cnt);
    for (idx_type k=0; k<cnt; ++k) {
      idx_type x_j = res[k];
      //if (frozen.count(x_j)>0) continue;
      if (frozen[x_j] || F(x_j)==0) continue;
      
      real_type T_j_new = es.compute(T, F, x_j);
      
      if (narrow[x_j]) {
        if (T_j_new<T(x_j)) {
          T(x_j) = T_j_new;
          #ifdef MINHEAP_SIMPLE
          narrowH.insert(x_j, T_j_new+hdistance(x_j));
          #else
          narrowH.decreaseValue(x_j, T_j_new+hdistance(x_j));
          #endif
        }
      }
      else {
        T(x_j) = T_j_new;
        narrowH.insert(x_j, T_j_new+hdistance(x_j));
        narrow[x_j] = true;
      }
    }
    //frozen.insert(x_i);
  }

  return T;
}
