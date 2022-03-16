#pragma once

#include <opencv2/opencv.hpp>
#include <ClothoidList.hh>

#include "grid.hpp"
#include "utils.hpp"
#include "dp_smoothing.hpp"



class FMPlanner {
public:
  FMPlanner() : map(nullptr) {}

	void setMap(Map const * map);
  bool plan(State2d const & start, State2d const & goal, vector<DPSmoothing::Curve> & path, bool goalRegion = false);
	bool planRaw(State2d const & start, State2d const & goal, vector<State2d> & path);

	vector<Wayline> waylines; // TODO: REMOVE
private:
	Map const * map;
	std::unique_ptr<Grid<real_type>> F;

	void
	mapUpdate();

};
