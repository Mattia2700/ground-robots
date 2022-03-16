#pragma once
#include "../hardwareglobalinterface.h"
#include "Clothoid.hh"
#include "Dubins.hh"
#include "grid.hpp"
#include "utils.hpp"
//#include <opencv2/opencv.hpp>


class DPSmoothing {
public:
#ifdef USE_DUBINS
  typedef DubinsCurve Curve;
#else
  typedef G2lib::ClothoidCurve Curve;
#endif

  DPSmoothing(Grid<real_type> const * grid);
  bool optimise(vector<Wayline> const & waylines, real_type theta_s, real_type theta_g, vector<Curve> & path/*parameters*/);

private:

  std::unique_ptr<Grid<real_type>> F;
 

  bool computeP2P(real_type x0, 
                  real_type y0,
                  real_type th0,
                  real_type x1,
                  real_type y1,
                  real_type th1,
                  Curve & path) const;
  
  bool collision(Curve const & path, real_type & score) const;

  bool optimise_inner();

    
  struct Cost {
    real_type score = 0;
    real_type L = 0;

    static Cost WORST() { 
      return {1e20, 0};
    };

    friend bool operator<(const Cost& l, const Cost& r)
    {
      //real_type const W_S = 0.2;
      //real_type sl = W_S*(-l.score) + (1-W_S)*l.L;
      //real_type sr = W_S*(-r.score) + (1-W_S)*r.L;
      //return sl<sr;
      //return (l.score>r.score) || ((l.score==r.score) && (l.L<r.L));
      return (l.score<r.score);
    }

    friend std::ostream& operator<<(std::ostream& output, Cost const & c) {
      output << "(" << c.score << "," << c.L << ")";
      return output; 
    }

    Cost& operator+=(Cost const & rhs) {
      score += rhs.score;
      //score = std::min(score, rhs.score);
      L += rhs.L;
      return *this;
    }
  };

  struct Cell {
    Cost cost;
    int sampleNxt;
    int wNxt;
    Cell(): cost() {}
  };
  
  std::vector<std::vector<std::vector<Cell>>> cells;
  

  Grid<real_type> const * grid;

  vector<Wayline> waylines;
  vector<AngularRange> angles;
  vector<idx_type> vangles;
  vector<idx_type> vwaylines;
};
