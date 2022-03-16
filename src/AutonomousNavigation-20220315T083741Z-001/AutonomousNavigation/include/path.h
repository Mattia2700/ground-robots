#ifndef PATH_H
#define PATH_H

#include "pathpoint.h"
#include <vector>
#include <string>
//#include <map.h>

/*! \brief This class describes the Path object.
 * \details The Path object describes the path information that are stored in the database and used to plot paths by the Map_Painter class.
*/

/// @brief Gives the sign of a variable
template <typename T> int sgn(T val){
  return (T(0)<val) - (val < T(0));
}

static double diff_angle(double beta, double alpha){
  //set alpha, beta between 0 and 2*pi
  while(fabs(beta)>2.0*M_PI){
    beta = beta - 2.0*M_PI*sgn(beta);
  }
  if(beta<0.0){
    beta = beta + 2.0*M_PI;
  }
  while(fabs(alpha)>2.0*M_PI){
    alpha = alpha - 2.0*M_PI*sgn(alpha);
  }
  if(alpha<0.0){
    alpha = alpha + 2.0*M_PI;
  }
  double difference = beta - alpha;
  if(difference>M_PI){
    difference = difference - 2.0*M_PI;
  }
  if(difference<-M_PI){
    difference = difference + 2.0*M_PI;
  }
  return difference;
}

class Path
{
public:
  Path()
  {}

  Path(std::vector<PathPoint> const & pts)
  {
    points = pts;
  }

  std::vector<PathPoint> points; /*!< Vector of PathPoint defining the points that compose the current path. */

  void getDynamicFrenetPoint(const double &x, const double &y, const double &theta, DynamicFrenetPoint &dfp, int &actual_path_index);
  void getClosestPoint(const double &x, const double &y, const double &theta, DynamicFrenetPoint &dfp, int &index);
  void getClosestPoint(const double &x, const double &y, const double &theta, DynamicFrenetPoint &dfp);
  PathPoint getLastPoint();

  double getCurvature(const double& s, const int& actual_path_index = 0);
  double getDistanceToEnd(const double& s);

  double maxCurvature = 0;
  void findMaxCurvature();
  double getMaxCurvature();

  void removeNegativePoints(double actual_s);
};

#endif // PATH_H
