#include "path.h"
#include <iterator>

//#include <Clothoid.hh>


// void Path::toJson(nlohmann::json &j){
//     nlohmann::json j_wp;
//     nlohmann::json j_in = nlohmann::json::array();
//     for (std::vector<PathPoint>::iterator it = points.begin() ; it != points.end(); ++it){
//         j_wp.clear();
//         (*it).toJson(j_wp);
//         j_in.push_back(j_wp);
//     }
//     j["id"] = id;
//     j["name"] = name.toStdString();
//     j["data"] = j_in;
// }

// void Path::fromJson(const nlohmann::json &j){
//     nlohmann::json j_data = j.at("data");
//     if(j_data.size()==0) return;
//     assert(j_data.type() == nlohmann::json::value_t::array);
//     std::string name_tmp = "";
//     try{
//         //name_tmp = j.at("path").at("name");
//     }catch(std::exception &e){
//         //name_tmp = "default";
//     }

//     name = QString::fromStdString(name_tmp);

//     points.clear();
//     PathPoint point;
//     nlohmann::json j_point;
//     for (nlohmann::json::iterator it = j_data.begin(); it != j_data.end(); ++it) {
//         j_point.clear();
//         j_point = *it;
//         point.x = j_point["x"];
//         point.y = j_point["y"];
//         point.theta   = j_point["theta"];
//         point.s       = j_point["s"];
//         point.c       = j_point["c"];
//         point.dc      = j_point["dc"];
//         points.push_back(point);
//     }

// }

void Path::getClosestPoint(const double &x, const double &y, const double &theta, DynamicFrenetPoint &dfp){
    double min_distance;
    double temp_norm = 0.0;
    size_t index = 0;
    double theta_d;

    /* Define the maximum possible distance */
    min_distance = std::numeric_limits<double>::max();

    /* Iterate over the elements of the path */
    for (size_t i = 0; i < points.size(); i++) {

      /* Obtain the distance from the current point to the walker */
      temp_norm = sqrt(pow((points[i].x - x), 2) + pow((points[i].y - y), 2));

      /* Check the minimum distance from the path to the walker */
      if (temp_norm < min_distance) {

        /* Update the minimum distance and the corresponding index */
        min_distance = temp_norm;
        index = i;
      }

    }

    theta_d = points[index].theta;

    dfp.x = points[index].x;
    dfp.y = points[index].y;
    dfp.l_x = cos(theta_d)*(x - points[index].x) + sin(theta_d)*(y - points[index].y);
    dfp.l_y = cos(theta_d)*(y - points[index].y) - sin(theta_d)*(x - points[index].x);

    dfp.theta_tilde = diff_angle(theta,theta_d);
    dfp.s = points[index].s;
    dfp.c = points[index].c;
    dfp.dc = points[index].dc;
}

void Path::getClosestPoint(const double &x, const double &y, const double &theta, DynamicFrenetPoint &dfp, int &index){
    double min_distance;
    double temp_norm = 0.0;
    double theta_d;

    /* Define the maximum possible distance */
    min_distance = std::numeric_limits<double>::max();

    /* Iterate over the elements of the path */
    for (size_t i = 0; i < points.size(); i++) {

      /* Obtain the distance from the current point to the walker */
      temp_norm = sqrt(pow((points[i].x - x), 2) + pow((points[i].y - y), 2));

      /* Check the minimum distance from the path to the walker */
      if (temp_norm < min_distance) {

        /* Update the minimum distance and the corresponding index */
        min_distance = temp_norm;
        index = i;
      }

    }

    theta_d = points[index].theta;

    dfp.x = points[index].x;
    dfp.y = points[index].y;
    dfp.l_x = cos(theta_d)*(x - points[index].x) + sin(theta_d)*(y - points[index].y);
    dfp.l_y = cos(theta_d)*(y - points[index].y) - sin(theta_d)*(x - points[index].x);

    dfp.theta_tilde = diff_angle(theta,theta_d);
    dfp.s = points[index].s;
    dfp.c = points[index].c;
    dfp.dc = points[index].dc;
}

void Path::getDynamicFrenetPoint(const double &x, const double &y, const double &theta, DynamicFrenetPoint &dfp, int &actual_path_index){
    double min_distance;
    int path_range = 50;
    int min_path_index = std::max(actual_path_index - path_range,0);
    int max_path_index = std::min(actual_path_index + path_range,(int)points.size()-1);

    int i_min1 = 0;  //index s min distance 1
    int i_min2 = 0; //index s min distance 2

    double x_path = 0.0; //x path of point
    double y_path = 0.0; //y path of point
    double theta_d = 0.0; //theta des path of point
    double c_s = 0.0; //curvature path of point
    double dc_ds = 0.0; //derivative of the curvature path of point

    double x_path1 = 0.0; //x path of point 1
    double y_path1 = 0.0; //y path of point 1
    double theta_d1 = 0.0; //theta des path of point 1
    double c_s1 = 0.0; //curvature path of point 1
    double dc_ds1 = 0.0; //derivative of the curvature path of point 1

    double x_path2 = 0.0; //x path of point 2
    double y_path2 = 0.0; //y path of point 2
    double theta_d2 = 0.0; //theta des path of point 2
    double c_s2 = 0.0; //curvature des path of point 2
    double dc_ds2 = 0.0; //derivative of the curvature pathof point 2

    double s1;
    double s2;

    /* Define the maximum possible distance */
    min_distance = std::numeric_limits<double>::max();

    for (size_t i = min_path_index; i <= max_path_index; i++) {
        if(fabs(points[i].s - dfp.s) <= min_distance){
            x_path1      = points[i].x;
            y_path1      = points[i].y;
            theta_d1     = points[i].theta;
            c_s1         = points[i].c;
            dc_ds1       = points[i].dc;
            min_distance = fabs(points[i].s - dfp.s);
            i_min1       = i;
        }
    }

    actual_path_index = i_min1;

    if(i_min1>0 && i_min1 < points.size() && points.size()>=3){
      if(fabs(points[i_min1+1].s - dfp.s) < fabs(points[i_min1-1].s - dfp.s)){
        i_min2 = i_min1 + 1;
      }else{
        i_min2 = i_min1 - 1;
      }
      x_path2      = points[i_min2].x;
      y_path2      = points[i_min2].y;
      theta_d2     = points[i_min2].theta;
      c_s2         = points[i_min2].c;
      dc_ds2       = points[i_min2].dc;

      s1 = points[i_min1].s;
      s2 = points[i_min2].s;

      x_path  = x_path1 + (x_path2-x_path1)/(s2-s1)*(dfp.s-s1);
      y_path  = y_path1 + (y_path2-y_path1)/(s2-s1)*(dfp.s-s1);
      theta_d = theta_d1 + (diff_angle(theta_d2,theta_d1))/(s2-s1)*(dfp.s-s1);
      c_s     = c_s1 + (c_s2-c_s1)/(s2-s1)*(dfp.s-s1);
      dc_ds   = dc_ds1 + (dc_ds2-dc_ds1)/(s2-s1)*(dfp.s-s1);

    }else{
      x_path  = x_path1;
      y_path  = y_path1;
      theta_d = theta_d1;
      c_s     = c_s1;
      dc_ds   = dc_ds1;
    }

    dfp.l_x = cos(theta_d)*(x - x_path) + sin(theta_d)*(y - y_path);
    dfp.l_y = cos(theta_d)*(y - y_path) - sin(theta_d)*(x - x_path);

    dfp.theta_tilde = diff_angle(theta,theta_d);
    dfp.c = c_s;
    dfp.dc = dc_ds;
    dfp.x = x_path;
    dfp.y = y_path;

}

PathPoint Path::getLastPoint(){
    return points.back();
}

double Path::getCurvature(const double& s, const int& actual_path_index) {
    int i = actual_path_index;
    if (s > points[0].s && s < points.back().s) {
        for (; s < points[i].s; --i);
        for (; s > points[i].s; ++i);
        double cPrev, sPrev, cPost, sPost;
        cPrev = points[i-1].c;
        cPost = points[i].c;
        sPrev = points[i-1].s;
        sPost = points[i].s;
        return cPrev+(cPost-cPrev)/(sPost-sPrev)*(s-sPrev);
    }
    else {
        if (s <= points[0].s) {
            return points[0].c;
        }
        else {
            return points.back().c;
        }
    }
}

double Path::getDistanceToEnd(const double& s) {
    return points.back().s-s;
}

void Path::findMaxCurvature(){
    maxCurvature = 0;
    for (size_t i = 0; i <= points.size(); i++) {
        if(fabs(points[i].c) > maxCurvature){
          maxCurvature = fabs(points[i].c);
        }
    }
}

double Path::getMaxCurvature()
{
    double curve = 0;
    for (size_t i = 0; i <= points.size(); i++) {
        if(fabs(points[i].c) > curve){
          curve = fabs(points[i].c);
        }
    }
    return curve;
}

void Path::removeNegativePoints(double actual_s)
{
    /*for (size_t i = 0; i <= points.size(); i++) {
        if(fabs(points[i].s) < actual_s){
          points.re
        }
    }*/
}
