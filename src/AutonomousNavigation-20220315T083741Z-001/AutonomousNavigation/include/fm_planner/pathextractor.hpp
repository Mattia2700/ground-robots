#pragma once

#include "grid.hpp"
#include "utils.hpp"
#include <iostream>

inline
std::vector<Point2d> extractpath(Grid<idx_type> const & parents, idx_type idx) {
  if (parents(idx) == -1) return {};

  std::vector<Point2d> res;
  idx_type cx, cy;
  parents.idx2rc(idx, cy, cx);
  res.push_back({cx,cy});
  while (parents(idx) != idx) {
    idx = parents(idx);
    parents.idx2rc(idx, cy, cx);
    res.push_back({cx,cy});
  };

  return res;
}