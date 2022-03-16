#ifndef VORONOI_H
#define VORONOI_H

#include "utils.hpp"
#include "pathpoint.h"
//#include "Navigation.hh"
#include <unordered_map>

struct DynamicObstacle {
  int id;
  Point2d pose;
  Point2d v;
};
  struct People {
     int id;
     std::vector<double> x;
     std::vector<double> y;
     std::vector<double> vx;
     std::vector<double> vy;
     std::vector<std::vector<double>> Xpred;
     bool isupdated = false;
  };



struct Neighbours {
  double time;
  int id;
  Point2d pose;
  std::vector<Point2d> poseH;
  std::vector<Point2d> vH;
  std::vector<Point2d> comm;
  std::vector<People> peopleNeigh;
  std::unordered_map<int, double> ncost;
};

struct StaticObstacle {
  int nObs;
  real_type p1X ;
  real_type p1Y ;
  real_type p2X ;
  real_type p2Y ;
};

struct ControlParams {
    double const dtC        = 10         ;
    double const k_brake    = 15          ;//15  ;
    double k_angular        = 2.5;//2;//1;          ;//8   ;
    double const k_angularC = 2; 
    double const alpha      = 0.5       ;
    double const k_i        = 8;//5;//2;//0.5;//0.2;        ;//7   ;
    double  v_des      = 0.4        ; //0.25 ;
    double  v_desC     = 0.4        ;//0.25 ;
    double const theta_th   = 3.14/7;//3.141592/7 ;//3.141592/5.5;
    double const beta_th    = 0.5;
};

struct VoronoiParams {
    double R_cohesion      = 15    ;
    double R_agent         = 1     ;
    double dA              = 0.4 ;
    double dO              = 0.05  ;
    double R_sensing       = 1.5   ; 
    double R_communication = 0.1; //R_sensing * 3.0 ;
    double dt              = 0.033 ;
    int    EOT             = 50    ;
    float  dx              = 0.075 ;
    float  U_0             = 10    ;
    float        R_gaussian      = 0.5   ; 
    float        rhoD            = 0.15   ;
    float  const dr              = 0.01  ;
    float  const dtheta          = 0.1   ;
    float  const wp_threshold    = 1.5   ;
    float  const L_min           = 1     ;
    float  const L_max           = 1     ;
    int          count           = 0     ;
    std::vector<real_type> xlim = {-R_sensing,R_sensing}  ;
    std::vector<real_type> ylim = {-R_sensing,R_sensing}  ;
};

Point2d voronoiCentroid(Map const & map,
                        Point2d const & vpose,
                        VoronoiParams const & params,
                        std::vector<DynamicObstacle> const & dObstacles,
                        std::vector<Neighbours> const & NeighboursP,
                        PathPoint const & frenetPoint,
                        cv::Mat & img,
                        nlohmann::json & jdata, double thetav);

#endif // VORONOI_H
