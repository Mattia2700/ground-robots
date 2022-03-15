#include "voronoi.h"
#include "graph.h"
#include <cmath>
#include "concavehull.h"
#include <iostream>
#include <fstream> 
#include <chrono>


int CM_FLAG = 0;
extern int R_ID;
std::ofstream File("/home/eecs/log/POS.txt");
std::ofstream File1("/home/eecs/log/NEIGH.txt");
std::ofstream File2("/home/eecs/log/SETS.txt");
std::ofstream File3("/home/eecs/log/HUM.txt");


//std::ofstream File3;
//int inPolygon(std::vector<Point2d> const & points, double testx, double testy){
//  int i, j, c = 0;
//  int nvert = points.size();
//  for (i = 0, j = nvert-1; i < nvert; j = i++) {
//    if ( ((points[i].y>testy) != (points[j].y>testy)) && (testx < (points[j].x-points[i].x) * (testy-points[i].y) / (points[j].x-points[i].y) + points[i].x)) {
//      c = !c;
//    }
//  }
//  return c;
//}

static inline double isLeft(Point2d const & P0, Point2d const & P1, Point2d const & P2)
{
  return ((P1.x - P0.x) * (P2.y - P0.y) -(P2.x - P0.x) * (P1.y - P0.y));
}

static bool inPolygon(std::vector<Point2d> const & vertices, Point2d const & testpoint)
{
  int wn = 0; // the winding number counter

  //std::cout << "VS: " << vertices.size() << std::endl;

  // loop through all edges of the polygon
  for (int i=1; i<vertices.size(); ++i)
  {
    // edge from V[i] to V[i+1]
    if (vertices.at(i-1).y <= testpoint.y)
    {
      // start y <= P.y
      if (vertices.at(i).y > testpoint.y) // an upward crossing
        if (isLeft(vertices.at(i-1), vertices.at(i), testpoint) > 0) // P left of edge
          ++wn; // have a valid up intersect
    }
    else
    {
      // start y > P.y (no test needed)
      if (vertices.at(i).y <= testpoint.y) // a downward crossing
        if (isLeft(vertices.at(i-1), vertices.at(i), testpoint) < 0) // P right of edge
          --wn; // have a valid down intersect
    }
  }
  return wn != 0;
}

std::vector<Point2d> setFiltering(Point2d const & vpose,
                        	  VoronoiParams const & params,
                                  std::vector<Point2d> const & refinedSetN, 
                                  Neighbours const & Neigh) {
  
  std::vector<Point2d> refinedSetNCM;

  for (int i = 0; i < refinedSetN.size(); ++i) {
    Point2d ptc = {refinedSetN[i].x, refinedSetN[i].y};
   // bool valid = true  ;
    //std::cout << "NC_SIZE: " << Neigh.comm.size() << std::endl;
    if (inPolygon(Neigh.comm, ptc)) {
    //  valid = false ;
    //  break;
   // }          
  
    //if (valid) {
      refinedSetNCM.push_back(refinedSetN[i]);
    }
  }

  return refinedSetNCM;
}


Point2d voronoiCentroid(Map const & map,
                        Point2d const & vpose,
                        VoronoiParams const & params,
                        std::vector<DynamicObstacle> const & dObstacles1,
                        std::vector<Neighbours> const & NeighboursP,
                        PathPoint const & frenetPoint,
                        cv::Mat & img,
                        nlohmann::json & jdata, double thetav) {

  std::vector<Point2d> visiblePoints;

  int y0M, x0M;
  map.world2map(vpose.x, vpose.y, y0M, x0M);
  for (int th = 0; th<2*M_PI / params.dtheta ; ++th) {
    double x1 = params.R_sensing * std::cos(th*params.dtheta) + vpose.x;
    double y1 = params.R_sensing * std::sin(th*params.dtheta) + vpose.y;
    int y1M, x1M;
    map.world2map(x1, y1, y1M, x1M);

    //cv::LineIterator it(map.img, cv::Point(x0M,y0M), cv::Point(x1M,y1M), 8, false);
    cv::LineIterator it(map.img, cv::Point(x0M,y0M), cv::Point(x1M,y1M), 8, false);
    for(int i = 0; i < it.count; i++, ++it)
    {
      uchar val = map.img.at<uchar>(it.pos());
      if (val>0 || (i==(it.count-1))) { // occupied, invalid point
        double xc = it.pos().x, yc = it.pos().y;
        visiblePoints.push_back(Point2d{xc, yc});
        break;
      }
    }
  }
  visiblePoints.push_back(visiblePoints.front());

  double x_min = -params.R_sensing + vpose.x;
  double x_max = params.R_sensing + vpose.x;
  double y_min = -params.R_sensing + vpose.y;
  double y_max = params.R_sensing + vpose.y;

  int r_min, r_max, c_min, c_max;
  map.world2map(x_min, y_min, r_min, c_min);
  map.world2map(x_max, y_max, r_max, c_max);

  std::vector<Point2d> visibilitySet;
  for(int i = r_min; i <= r_max; i++) {
    for(int j = c_min; j <= c_max; j++) {
      Point2d pt = {(double)j, (double)i};
      if (inPolygon(visiblePoints, pt)) {
        double x, y;
        map.map2world(i, j, x, y);
        visibilitySet.push_back(Point2d{ x, y });
//        img.at<cv::Vec3b>(i,j)[0] = 20;
//        img.at<cv::Vec3b>(i,j)[1] = 150;
//        img.at<cv::Vec3b>(i,j)[2] = 20;
      }
    }
  }
//communication set

  std::vector<Point2d> communicationPoints;
  std::vector<cv::Point2f> communicationSet1;


  int y0Mc, x0Mc;
  map.world2map(vpose.x, vpose.y, y0Mc, x0Mc);
  for (int th = 0; th<2*M_PI / params.dtheta ; ++th) {
    double x1c = params.R_communication * std::cos(th*params.dtheta) + vpose.x;
    double y1c = params.R_communication * std::sin(th*params.dtheta) + vpose.y;
    int y1Mc, x1Mc;
    map.world2map(x1c, y1c, y1Mc, x1Mc);

    //cv::LineIterator it(map.img, cv::Point(x0M,y0M), cv::Point(x1M,y1M), 8, false);
    cv::LineIterator it(map.img, cv::Point(x0Mc,y0Mc), cv::Point(x1Mc,y1Mc), 8, false);
    for(int i = 0; i < it.count; i++, ++it)
    {
      uchar val = map.img.at<uchar>(it.pos());
      if (val>0 || (i==(it.count-1))) { // occupied, invalid point
        bool validF = true;
        for (int j = 0; j < NeighboursP.size() ; j++){
          double xP, yP; 
          //int xMap = it.pos().x, yMap = it.pos().y;
	  map.map2world(it.pos().y, it.pos().x, xP,yP);

          if (val>0 && std::hypot(NeighboursP[j].pose.x - xP, NeighboursP[j].pose.y - yP)< 1 )  {
            validF = false;
            break;
          }
        }
        
        if (validF) {
	  double xcc = it.pos().x, ycc = it.pos().y;
          communicationPoints.push_back(Point2d{xcc, ycc});
          break;
        }
      }
    }
  }
  communicationPoints.push_back(visiblePoints.front());

  double x_minc = -params.R_communication + vpose.x;
  double x_maxc =  params.R_communication + vpose.x;
  double y_minc = -params.R_communication + vpose.y;
  double y_maxc =  params.R_communication + vpose.y;

  int r_minc, r_maxc, c_minc, c_maxc;
  map.world2map(x_minc, y_minc, r_minc, c_minc);
  map.world2map(x_maxc, y_maxc, r_maxc, c_maxc);

  std::vector<Point2d> communicationSet;
  for(int i = r_minc; i <= r_maxc; i++) {
    for(int j = c_minc; j <= c_maxc; j++) {
      Point2d ptc = {(double)j, (double)i};
      if (inPolygon(communicationPoints, ptc)) {
        double xc, yc;
        map.map2world(i, j, xc, yc);
        communicationSet1.push_back(cv::Point2f{ xc, yc });
   }
    }
      }
     typedef std::array<double,2> point_type;
     std::vector<point_type> communicationSet1Arr;
     communicationSet1Arr.reserve(communicationSet1.size());
     for(int j = 0; j < communicationSet1.size(); j++){
        communicationSet1Arr.emplace_back(point_type{communicationSet1[j].x,communicationSet1[j].y});
     } 

   //findContours
 // compute the contour of communicationSet.
   std::vector<int> hullindex;
   if (communicationSet1.size()>0){
   cv::convexHull(communicationSet1, hullindex, false, false);
   auto communicationSetArr = concaveman<double, 16>(communicationSet1Arr, hullindex, 2, 0 );
   communicationSet.reserve(communicationSetArr.size()+1);
   for(int j = 0; j < communicationSetArr.size(); j++){
        communicationSet.emplace_back(communicationSetArr[j][0],communicationSetArr[j][1]);
     } 

   communicationSet.emplace_back(communicationSet.front());
 	
//std::cout << communicationSet.size() << " , "<< communicationSetArr.size() << " , " << communicationSet1.size() << " , " << communicationSet1Arr.size() << std::endl;
 }

  jdata["comm"]    = communicationSet;
for (int i=0; i< dObstacles1.size(); i++){
  jdata["poseH"][i]   = dObstacles1[i].pose;
  jdata["vH"][i]      = dObstacles1[i].v; 
}
  //jadata["poseH"] = NeighboursP

  std::vector<DynamicObstacle> dObstacles = dObstacles1;
  for (int i=0; i < NeighboursP.size(); i++){
     DynamicObstacle dObst;
     dObst.pose.x = NeighboursP[i].pose.x - vpose.x;
     dObst.pose.y = NeighboursP[i].pose.y - vpose.y; 
     dObstacles.push_back(dObst);
  }

  
  std::vector<Point2d> dynamicSet;
  for (auto const & pt: visibilitySet) {
    bool inCell = true;
    for (auto const & dObs: dObstacles) {
      double x_d = dObs.pose.x + vpose.x;
      double y_d = dObs.pose.y + vpose.y;
      double d_obst = std::hypot(pt.x-x_d, pt.y-y_d);
      double d_robot = std::hypot(pt.x-vpose.x, pt.y-vpose.y);
      inCell = d_obst > d_robot;
      if (!inCell) break;
    }
    if (inCell) {
      dynamicSet.push_back(pt);
    }
  }

  std::vector<Point2d> refinedSet;

  for (int i = 0; i < dynamicSet.size(); ++i) {
    bool valid = true;
    for (int j = 0; j < dObstacles.size(); ++j) {
      double uvec[2] = {0., 0.};

      double d_x, d_y;
      d_x = dObstacles[j].pose.x + vpose.x;
      d_y = dObstacles[j].pose.y + vpose.y;

      double xm   = 0.5 * ( d_x + vpose.x );
      double ym   = 0.5 * ( d_y + vpose.y );
      double dm   = std::hypot(xm-vpose.x, ym-vpose.y);
      if (dm < (params.dA+params.dO)) {
        double L = std::hypot(vpose.x-d_x, vpose.y-d_y);
        uvec[0]    =  ( vpose.x-d_x ) / L;
        uvec[1]    =  ( vpose.y-d_y )/ L;
        double m    =  ( vpose.y - d_y )/( vpose.x - d_x );
        double solx = xm + (params.dO+params.dA-dm)*uvec[0];
        double soly = ym + (params.dO+params.dA-dm)*uvec[1];

        if( vpose.y > -1/m*(vpose.x-solx)+soly ){
          double di_x = dynamicSet[i].x;
          double di_y = dynamicSet[i].y;
          if (!( di_y > -1/m * ( di_x - solx) + soly) ) {
            valid = false;
          }
        }
        else{
          double di_x = dynamicSet[i].x;
          double di_y = dynamicSet[i].y;
          if (!( di_y < -1/m * ( di_x - solx) + soly) ) {
            valid = false;
          }
        }
      }
    }
    if (valid) {
      refinedSet.push_back(dynamicSet[i]);
    }
  }


  //Neighbours staffs  
  
  std::vector<Point2d> dynamicSetN;
  for (auto const & pt: dynamicSet) {
    bool inCell = true;
    for (auto const & Neigh: NeighboursP) {
      double x_d = Neigh.pose.x ;//+ vpose.x;
      double y_d = Neigh.pose.y ;//+ vpose.y;
      double d_obst = std::hypot(pt.x-x_d, pt.y-y_d);
      double d_robot = std::hypot(pt.x-vpose.x, pt.y-vpose.y);
      inCell = d_obst > d_robot;
      if (!inCell) break;
    }
    if (inCell) {
      dynamicSetN.push_back(pt);
    }
  }

  std::vector<Point2d> refinedSetN;

  for (int i = 0; i < dynamicSetN.size(); ++i) {
    bool valid = true;
    for (int j = 0; j < NeighboursP.size(); ++j) {
      double uvec[2] = {0., 0.};

      double d_x, d_y;
      d_x = NeighboursP[j].pose.x ;//+ vpose.x;
      d_y = NeighboursP[j].pose.y ;//+ vpose.y;

      double xm   = 0.5 * ( d_x + vpose.x );
      double ym   = 0.5 * ( d_y + vpose.y );
      double dm   = std::hypot(xm-vpose.x, ym-vpose.y);
      if (dm < (params.dA+params.dA)) {
        double L = std::hypot(vpose.x-d_x, vpose.y-d_y);
        uvec[0]    =  ( vpose.x-d_x ) / L;
        uvec[1]    =  ( vpose.y-d_y )/ L;
        double m    =  ( vpose.y - d_y )/( vpose.x - d_x );
        double solx = xm + (params.dA+params.dA-dm)*uvec[0];
        double soly = ym + (params.dA+params.dA-dm)*uvec[1];

        if( vpose.y > -1/m*(vpose.x-solx)+soly ){
          double di_x = dynamicSetN[i].x;
          double di_y = dynamicSetN[i].y;
          if (!( di_y > -1/m * ( di_x - solx) + soly) ) {
            valid = false;
          }
        }
        else{
          double di_x = dynamicSetN[i].x;
          double di_y = dynamicSetN[i].y;
          if (!( di_y < -1/m * ( di_x - solx) + soly) ) {
            valid = false;
          }
        }
      }
    }
    if (valid) {
      refinedSetN.push_back(dynamicSetN[i]);
    }
  }
  

//Connectivity Maintenance staffs 


/*     std::vector<Point2d> refinedSetNCM;
     
     
     for (int i = 0; i < refinedSetN.size(); ++i) {
        Point2d ptc = {refinedSetN[i].x, refinedSetN[i].y};
	bool valid = true  ;
        for (int j = 0; j < NeighboursP.size(); ++j) {
      if (!(inPolygon(NeighboursP[j].comm, ptc))) {
         valid = false ;
         break;
      }          
        }
        if (valid) {
        refinedSetNCM.push_back(refinedSetN[i]);
        }
     }

  jdata["ncost"] = {};
*/


   std::unordered_map<int, double> neighbourCost;
   //if (NeighboursP.empty()) {
   //  refinedSetNCM = refinedSetN;
   //}
   //else {
        
     //int dopt = 0;
     for (int j = 0; j < NeighboursP.size(); ++j) {
       std::vector<Point2d> refinedSetNCMCurr = setFiltering(vpose, params, refinedSetN, NeighboursP[j]);
       int dcurr = refinedSetN.size() - refinedSetNCMCurr.size();
       neighbourCost[NeighboursP[j].id] = dcurr;
       //std::cout << "NID: " << NeighboursP[j].id << " DCURR: " << dcurr << std::endl;

       //if (dcurr>dopt) {
         //dopt = dcurr;
         //refinedSetNCM = refinedSetNCMCurr;
       //}
     }
   //}
   jdata["ncost"] = neighbourCost;

   
  Graph g;
  

  std::map<std::pair<int,int>, int> costs;
  for (int i = 0 ; i < NeighboursP.size(); ++i) {
    int nid = NeighboursP[i].id;
    for (auto n: NeighboursP[i].ncost) {
      int nnid = n.first;
      int nc = n.second;
      costs[{nid, nnid}] = nc;
    }
  }

  for (int i = 0 ; i < NeighboursP.size(); ++i) {
    int nid = NeighboursP[i].id;
    int nc = neighbourCost[nid];
    costs[{R_ID, nid}] = nc;
  } 


  for (auto v: costs) {
    auto d_edge = v.first;
    int d_cost = v.second;
    if (d_edge.first>d_edge.second) continue;
    if (costs.count({d_edge.second, d_edge.first})) {
      int d_cost_rev = costs[{d_edge.second, d_edge.first}];
      g.addEdge(d_edge.first, d_edge.second, d_cost+d_cost_rev);
      //std::cout << "EDGE: " << d_edge.first << " " << d_edge.second << " " << (d_cost+d_cost_rev) << std::endl;
    }
  }

  std::vector<iPair> mst = g.kruskalMST();
  
  std::vector<int> myNeighbours;
  
  //std::cout << "MST:" << std::endl; 
  for (auto item: mst) {
    //std::cout <<  "(" << item.first << "," << item.second << ")" << std::endl;
    if (item.first == R_ID)
      myNeighbours.push_back(item.second);
    if (item.second == R_ID) 
      myNeighbours.push_back(item.first);
  }
double meanX = 0;
double meanY = 0;

  std::unordered_map<int, int> Neigh_idx2pos;
  for (int i = 0 ; i < NeighboursP.size(); ++i) {
    int nid = NeighboursP[i].id;
    Neigh_idx2pos[nid] = i;
    meanX += NeighboursP[i].pose.x ;
    meanY += NeighboursP[i].pose.y ;
  } 


  std::vector<Point2d> refinedSetNCM = refinedSetN;
  for (int neigh: myNeighbours) {
  //for (int neigh = 0; neigh < NeighboursP.size(); ++neigh){ 
   refinedSetNCM = setFiltering(vpose, params, refinedSetNCM, NeighboursP[Neigh_idx2pos[neigh]]);
}

 meanX = meanX/NeighboursP.size();
 meanY = meanY/NeighboursP.size();
  
  //std::cout << meanX <<" ,"<< meanY <<std::endl;
/*  for (int i = 0 ; i < NeighboursP.size(); ++i) {
    int nid = NeighboursP[i].id;
    for (auto n: NeighboursP[i].ncost) {
      int nnid = n->first;
      double nc = n->second;
      if (nnid == R_ID) {
        if (neighbourCost.count(nid)) {
          g.addEdge(nid, nnid, nc+neighbourCost[nid]);
        }
      }
      else if (nnid>nid) {
        if (NeighboursP[nnid].count(nid)) {
          g.addEdge(nid, nnid, nc+neighbourCost[nid]);
        }
      }
    }
  }
*/    
   
  

  double mass    = 0   ;
  double Cx      = 0   ;
  double Cy      = 0   ;
  // for (int j = 0 ; j < NeighboursP.size(); ++j){
  //     for (int i = 0 ; i < NeighboursP[j].comm.size(); ++i){
  //       int r, c;
  //       map.world2map(NeighboursP[j].comm[i].x, NeighboursP[j].comm[i].y, r, c);
  //       //cv::circle(img, cv::Point(c,r), 1, cv::Scalar(100, 0, 100), -1);
  //       if (r <0 || r >= img.rows || c<0 || c>= img.cols) continue; 
  //       img.at<cv::Vec3b>(r,c)[0] = 30;
  //       img.at<cv::Vec3b>(r,c)[1] = 144;
  //       img.at<cv::Vec3b>(r,c)[2] = 255;
  //     }
  //  }

 if(CM_FLAG == 1){

  for (int i = 0 ; i < refinedSetNCM.size(); ++i){
    Point2d tmp = Point2d{ refinedSetNCM[i].x-frenetPoint.x, refinedSetNCM[i].y-frenetPoint.y };
   //rendezvous
   //Point2d tmp = Point2d{ refinedSetNCM[i].x - meanX, refinedSetNCM[i].y - meanY };
    double norm = std::hypot(tmp.x, tmp.y);
    double val = params.U_0 * exp(-pow(norm,2)/params.R_gaussian);
    mass += val;
    Cx   += refinedSetNCM[i].x * val;
    Cy   += refinedSetNCM[i].y * val;

     int r, c;
     map.world2map(refinedSetNCM[i].x, refinedSetNCM[i].y, r, c);
         if (r <0 || r >= img.rows || c<0 || c>= img.cols) continue; 
     //cv::circle(img, cv::Point(c,r), 1, cv::Scalar(100, 0, 100), -1);
     img.at<cv::Vec3b>(r,c)[0] = 255;
     img.at<cv::Vec3b>(r,c)[1] = 144;
     img.at<cv::Vec3b>(r,c)[2] = 30;
  }
 }else{
   for (int i = 0 ; i < refinedSetN.size(); ++i){
    Point2d tmp = Point2d{ refinedSetN[i].x-frenetPoint.x, refinedSetN[i].y-frenetPoint.y };
    double norm = std::hypot(tmp.x, tmp.y);
    double val = params.U_0 * exp(-pow(norm,2)/params.R_gaussian);
    mass += val;
    Cx   += refinedSetN[i].x * val;
    Cy   += refinedSetN[i].y * val;
  

   int r, c;
     map.world2map(refinedSetN[i].x, refinedSetN[i].y, r, c);
         if (r <0 || r >= img.rows || c<0 || c>= img.cols) continue; 

     //cv::circle(img, cv::Point(c,r), 1, cv::Scalar(100, 0, 100), -1);
     img.at<cv::Vec3b>(r,c)[0] = 255;
     img.at<cv::Vec3b>(r,c)[1] = 144;
     img.at<cv::Vec3b>(r,c)[2] = 30;

   }
 }
  static int cnt = 20;

  Point2d centroid = {Cx/mass, Cy/mass};
  // Point2d centroid = {frenetPoint.x, frenetPoint.y};
     //log data 
     unsigned long now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
     //File.open("POS.txt");
     
     File << now <<";"<< vpose.x << ";" << vpose.y << ";" << thetav << ";" << centroid.x << ";" << centroid.y << std::endl;

     //File1.open("NEIGH.txt");
     
     File1 << now << ";" << myNeighbours.size() << std::endl;
     for(int i = 0 ; i <  myNeighbours.size() ; i++)
     File1 << myNeighbours[i] << std::endl;
     

     //File2.open("SETS.txt");
    
     File2 << now << ";" << refinedSetN.size() << std::endl;
     for(int i = 0 ; i <  refinedSetN.size() ; i++)
     File2 << refinedSetN[i].x << ";" << refinedSetN[i].y << std::endl;
     
     File3 << now << ";" << dObstacles.size() << std::endl;
     for(int i = 0 ; i < dObstacles.size() ; i++)
     File3 << dObstacles[i].pose.x  << ";" << dObstacles[i].pose.y << ";" << dObstacles[i].v.x << ";" << dObstacles[i].v.y << ";" << dObstacles[i].id << std::endl;
     if (cnt>=20) {
      File.flush();
      File1.flush();
      File2.flush();
      File3.flush();
      cnt = 0;
     }
     ++cnt;
    
     
    // File3.open("SETS2.txt");
    // File3 << CommunicationSet.size()  << std::endl;
    // for(int i = 0 ; i <  CommunicationSet.size() ; i++)
    // File3 << CommunicationSet[i].x << ";" << CommunicationSet[i].y << std::endl;
    // File3.close();



  
   int r, c;
   map.world2map(centroid.x, centroid.y, r, c);
   cv::circle(img, cv::Point(c,r), 1, cv::Scalar(100, 200, 100), -1);
  return centroid;
}

