#ifndef PATHPOINT_H
#define PATHPOINT_H

#include <json.hpp>
#include <string>
#include <sstream>

/*! \brief This class describes the PathPoint object.
 * \details The PathPoint is a particular point since it is composed by:
 * \arg x x-position of the point in meters.
 * \arg y y-position of the point in meters.
 * \arg theta desired angle that the path has at the specific point.
 * \arg s is the curvilinear abscissa at a specific point
 * \arg c desired curvature that the path has at the specific point.
 * \arg dc desired curvature derivative that the path has at the specific point.
*/

struct DynamicFrenetPoint{

  double x;
  double y;
  double l_x;   ///< The x-distance of the Walker from the path.
  double l_y;   ///< The y-distance of the Walker from the path.
  double theta_tilde;  ///< The yaw error of the.
  double s;   ///< The curvilinear abscissa of the Frenet Point.
  double c;   ///< The curvature of the Frenet Point.
  double dc; ///< The derivative of the curvature of the preview point

  /// @brief Constructor.
  ///
  /// This is the constructor of the structure.
  DynamicFrenetPoint(): x(0.0f), y(0.0f), l_x(0.0f), l_y(0.0f), theta_tilde(0.0f), s(0.0f), c(0.0f), dc(0.0f) { }

  /// @brief Constructor.
  ///
  /// This is the copy constructor of the structure.
  DynamicFrenetPoint(const DynamicFrenetPoint &in) :
    x(in.x),
    y(in.y),
    l_x(in.l_x),
    l_y(in.l_y),
    theta_tilde(in.theta_tilde),
    s(in.s),
    c(in.c),
    dc(in.dc) { }

};

class PathPoint
{
public:
  //! Class base constructor. Each element is set as 0.
  PathPoint(){
    x = 0;
    y = 0;
    theta = 0;
    s = 0;
    c = 0;
    dc = 0;
    backwards = false;
  }
  //! Class constructor that takes element by element and populates the current object.
  PathPoint(double x_p, double y_p, double theta_p, double s_p, double c_p, double dc_p){
    x = x_p;
    y = y_p;
    theta = theta_p;
    s = s_p;
    c = c_p;
    dc = dc_p;
    backwards = false;
  }
  //! Class constructor that populates the current object for reference.
  PathPoint(const PathPoint & pt){
    x = pt.x;
    y = pt.y;
    theta = pt.theta;
    s = pt.s;
    c = pt.c;
    dc = pt.dc;
    backwards = pt.backwards;
  }

  double x; /*!< double expressing the x Cartesian coordinates w.r.t. a given map. */
  double y; /*!< double expressing the y Cartesian coordinates w.r.t. a given map. */
  double theta; /*!< double expressing the orientations w.r.t. a given map. */
  double s; /*!< double expressing the curvilinear abscissa at a given point. */
  double c; /*!< double expressing the curvature of the path at the given point. */
  double dc; /*!< double expressing the curvature derivative of the path at the given point. */
  bool backwards = false;

  //! Member that takes the current PathPoint and generate the following QString: "x y theta c dc".
  std::string stringlify();
  //! Member that takes a string with the same form of PathPoint::stringlify and populates the current object parameters.
  void fromString(std::string str);

  void toJson(nlohmann::json &j);
};

#endif // PATHPOINT_H
