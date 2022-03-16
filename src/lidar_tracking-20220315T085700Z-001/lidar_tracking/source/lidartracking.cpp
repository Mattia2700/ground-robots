#include "lidartracking.h"
#include "hardwareglobalinterface.h"
#include "robotstatus.h"
//#include "opencv2/imgproc.hpp"
//#include "utils.h"
#include <set>



int distance_th = 10;
double resolution = 0.05;

/*!
  \def TRACE(x)
  Macro to print a variable \a x along with its name and the position where the macro is called (file an line)
*/
#define TRACE(x) do {\
    std::cerr<<"file:"<<__FILE__<<"Line:"<<__LINE__<<" :  "<<#x << '=' << x<<std::endl;\
    std::cerr.flush();\
 } while (0)

LidarTracking::LidarTracking()
{
  mm = std::make_unique<MapManager>(240, 0.035, MapManager::RobotGeometry{0.5});

  timerMapProcessing = std::make_unique<Timer>();
  //connect(timerMapProcessing,SIGNAL(timeout()),this,SLOT(callback_timerMapProcessing()));
  timerMapProcessing->registerCallback([this]() { callback_timerMapProcessing(); });
  timerMapProcessing->start(100);

  timerMapUpdate = std::make_unique<Timer>();
  //connect(timerMapUpdate,SIGNAL(timeout()),this,SLOT(callback_timerMapUpdate()));
  timerMapUpdate->registerCallback([this]() { callback_timerMapUpdate(); });
  timerMapUpdate->start(100);

  trackingPublisher = std::make_unique<Common::Publisher>(HardwareGlobalInterface::getInstance().getParams().trackingPublisher);

//  trackingServer = std::make_unique<Common::Replier>();
//  trackingServer->register_callback([&](const std::string& received, void * data, std::string & tosend){
//    nlohmann::json j_in; //= nlohmann::json::parse(received);
//    nlohmann::json j_out;
//    try{
//      j_in = nlohmann::json::parse(std::string(received));
//    }catch(std::exception &e){
//      std::cerr << "error parsing json callback guidance client: " << std::endl;
//      return;
//    }
//    processMessage(j_in,j_out);
//    tosend = std::string("");
//    tosend = j_out.dump() + '\0';
//  }, nullptr);

//  trackingServer->start("tcp://127.0.0.1:1235");
}

LidarTracking::~LidarTracking()
{
//  subLOC.stop();
//  subLIDAR.stop();
//  subREALSENSE.stop();
}

//void LidarTracking::startTracking(){
  //int nag = 115;
//  HardwareParameters params = HardwareGlobalInterface::getInstance().getParams();
//#ifdef LOC_REALSENSE
//  subLOC.start(params.realSensePosePublisher, "POS");
//#else
//  subLOC.start(params.kartoPosePublisher, "POS");
//#endif
//  subLIDAR.start(params.frontLidarPublisher, "LIDAR");
//  subREALSENSE.start(params.realSensePublisher, "LIDAR");

  //    subLOC.start("tcp://10.196.80."  + std::to_string(nag) + ":5563","LOC");
  //    subLIDAR.start("tcp://10.196.80."  + std::to_string(nag) + ":7500","LIDAR");
//}

//void LidarTracking::stopTracking(){
//  subLIDAR.stop();
//  subLOC.stop();
//  subREALSENSE.stop();
//}

//void LidarTracking::poseSubscriber(const char *topic, const char *buf, size_t size, void *data)
//{
//  nlohmann::json j;

//  try{
//    j = nlohmann::json::parse(std::string(buf, size));
//    std::unique_lock<std::mutex> lock(poseMutex);
//    pose.x = j.at("loc_data").at("x");
//    pose.y = j.at("loc_data").at("y");
//    pose.theta = j.at("loc_data").at("theta");
//    //std::cout << (int)(pose.theta*180.0/M_PI)%360 << std::endl;
//    loc_init = true;
//  }
//  catch(std::exception &e){
//    std::cerr << "error parsing loc data: " << e.what() << std::endl;
//  }
//}

//void LidarTracking::lidarSubscriber(const char *topic, const char *buf, size_t size, void *data)
//{
//  nlohmann::json j;

//  //static int count = 0; // TODO: why static??
//  int count = 0;
//  utils::ScanData scan;

//  try{
//    j = nlohmann::json::parse(std::string(buf, size));

//    int size = j.at("size");
//    double val;
//    double angle = 0;
//    double inc = 360.0/((double)(size));
//    double angle_offset = 0;

//    std::vector<RobotStatus::LidarDatum> datum;

//    for(int i=0;i<size;i++){
//      val = j.at("data")[i];

//      angle = count*inc*M_PI/180.0 + angle_offset;
//      count++;

//      if(val < 0.05){
//        continue;
//      }

//      //Normalize the angle within 0/2Pi
//      angle = atan2(sin(angle),cos(angle));

//      RobotStatus::LidarDatum datumLidar;
//      datumLidar.angle = angle;
//      datumLidar.distance = val;
//      datumLidar.x = val*cos(angle);
//      datumLidar.y = val*sin(angle);
//      datumLidar.isSafe = true;
//      datum.push_back(datumLidar);
//    }

//    {
//      std::unique_lock<std::mutex> lock(lastDataMutex);
//      lastLidar.x_offset = 0;
//      lastLidar.y_offset = 0;
//      lastLidar.datum = datum;
//      lastLidar.lidarTimer =std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//    }

//  }
//  catch(std::exception &e){
//    std::cerr << "error parsing lidar data: " << e.what() << std::endl;
//  }
//}

//void LidarTracking::realsenseSubscriber(const char *topic, const char *buf, size_t size, void *data)
//{
//  nlohmann::json j;

//  //static int count = 0; // TODO: why static??
//  int count = 0;
//  utils::ScanData scan;

//  try{
//    j = nlohmann::json::parse(std::string(buf, size));

//    int size = j.at("size");
//    double val;
//    double angle = 0;
//    double inc = 360.0/((double)(size));
//    double angle_offset = 0;

//    std::vector<RobotStatus::LidarDatum> datum;

//    for(int i=0;i<size;i++){
//      val = j.at("data")[i];

//      angle = count*inc*M_PI/180.0 + angle_offset;
//      count++;

//      if(val < 0.05){
//        continue;
//      }

//      //Normalize the angle within 0/2Pi
//      angle = atan2(sin(angle),cos(angle));

//      RobotStatus::LidarDatum datumLidar;
//      datumLidar.angle = angle;
//      datumLidar.distance = val;
//      datumLidar.x = val*cos(angle);
//      datumLidar.y = val*sin(angle);
//      datumLidar.isSafe = true;
//      datum.push_back(datumLidar);
//    }

//    {
//      HardwareParameters params;
//      std::unique_lock<std::mutex> lock(lastDataMutex);
//      lastRealsense.x_offset = params.cameraOffsetX;
//      lastRealsense.y_offset = params.cameraOffsetY;
//      lastRealsense.datum = datum;
//      lastRealsense.lidarTimer =std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//    }

//  }
//  catch(std::exception &e){
//    std::cerr << "error parsing lidar data: " << e.what() << std::endl;
//  }
//}

void LidarTracking::callback_timerMapUpdate() {
  RobotStatus::LidarData ldata;
  HardwareGlobalInterface::getInstance().getFrontLidarData(ldata);
  RobotStatus::LocalizationData pose;
#ifdef LOC_REALSENSE
  HardwareGlobalInterface::getInstance().getLocalizationDataRealSense(pose);
#else
  HardwareGlobalInterface::getInstance().getLocalizationDataKarto(pose);
#endif
  RobotStatus::LidarData rdata;
  HardwareGlobalInterface::getInstance().getRealSenseLidarData(rdata);

  HardwareParameters params;
  rdata.setMountingPosition(params.cameraOffsetX, params.cameraOffsetY);

  ScanData scan;
  scan.data = {ldata, rdata};
  scan.pose = {pose.x, pose.y, pose.theta};

  {
    std::unique_lock<std::mutex> lock(mmMutex);
    mm->update(scan);
  }
}

//void LidarTracking::callback_timerMapProcessing(){
//  cv::Mat img;
//  {
//    std::unique_lock<std::mutex> lockTracker(trackerMutex);
//    img = mm->generateImage();
//  }
//    //Generate the map according to the lidar scans

//  static int count = 0;


//        if (!img.empty())
//        {
//          cv::Mat imgF;
//          cv::flip(img, imgF, 0);

//          std::vector<std::vector<cv::Point>> contours, cont_single;
//          cv::Mat imgMask;
//          cvtColor(imgF, imgMask, cv::COLOR_BGR2GRAY);


//          cv::findContours(imgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // find external contours of each blob
//          cv::Mat newMask = cv::Mat(imgMask.rows, imgMask.cols, CV_8UC1, cv::Scalar(0));
//          std::vector<cv::Point2f> centroids;

//          for (int i=0; i<contours.size(); ++i)
//          {
//            double area = cv::contourArea(contours[i]);
//            double perimeter = cv::arcLength(contours[i], true);

//            if (perimeter>100) continue; // filter too small contours to remove false positives
//            cont_single = {contours[i]};
//drawContours(newMask, cont_single, -1, cv::Scalar(255), -1, cv::LINE_AA);

//            cv::Moments mu = cv::moments(contours[i], false);

//            // get the centroid of figures.
//            centroids.push_back(cv::Point2f(mu.m10/mu.m00, mu.m01/mu.m00));
//          }

//          cvtColor(newMask, imgF, cv::COLOR_GRAY2BGR);

//          //Get all the centroinds

//          std::set<int> visible;
//            for (auto pt: centroids)
//            {
//              double dmin = 999999;
//              int idmin = -1;

//              for (auto element: tracks)
//              {
//                int id = element.second.id;
//                cv::Point2f & ptc = element.second.centroid;
//                double dcur = std::hypot(ptc.x-pt.x, ptc.y-pt.y);
//                if (dcur < dmin)
//                {
//                  dmin = dcur;
//                  idmin = id;
//                }
//              }

//              if (idmin==-1 || dmin>MIN_DST)
//              {
//                // create new element
//                Object obj;
//                obj.id = NXT_ID++;
//                obj.centroid = pt;
//                obj.cnt = 0;
//                obj.obj_id = -1;
//                tracks.insert({obj.id, obj});
//                visible.insert(obj.id);
//              }
//              else
//              {
//                Object & obj = tracks[idmin];
//                obj.centroid = pt;
//                //obj.cnt++;
//                visible.insert(obj.id);
//              }
//            }

//            std::unordered_map<int, Object> tracksNew;
//            for (auto& element: tracks)
//            {
//              if (visible.find(element.first) != visible.end())
//              {
//                if (element.second.cnt == 15 && element.second.obj_id == -1)
//                {
//                    element.second.cnt = 100;
//                    element.second.obj_id = NXT_OBJ_ID++;
//                }
//                else if (element.second.cnt!=15 && element.second.cnt<100) element.second.cnt++;
//                tracksNew.insert(element);
//              }
//              else
//              {
//                element.second.cnt--;
//                if (element.second.cnt > 50)
//                  tracksNew.insert(element);
//              }
//            }
//            tracks = tracksNew;

//            for (auto element: tracks)
//            {
//              if (element.second.obj_id!=-1)
//              {
//                cv::circle(imgF, element.second.centroid, 4, cv::Scalar(0,0,255), -1);
//                cv::putText(imgF, std::to_string(element.second.obj_id), element.second.centroid, 1, 1, cv::Scalar(255,0,0));
//              }
//            }


//          objectsTrackedVector.clear();
//          for (auto element: tracks)
//          {
//              if (element.second.obj_id!=-1)
//              {
//                  ObjectItem tmp;
//                  tmp.id = element.second.obj_id;
//                  tmp.probability = element.second.cnt;
//                  double tmp_x = (double)(mm->getSize() - element.second.centroid.y)*mm->getResolution() ;
//                  double tmp_y = (double)(mm->getSize() - element.second.centroid.x)*mm->getResolution();

//                  //tmp.x = tmp_x*cos(pose.theta - M_PI_2) + tmp_y*sin(pose.theta - M_PI_2);
//                  //tmp.y = tmp_y*cos(pose.theta - M_PI_2) - tmp_x*sin(pose.theta - M_PI_2);

//                  //Perchè ruoto la mappa e tengo fermo il robot
//                  tmp.x = tmp_y*sin(- M_PI_2);
//                  tmp.y = -tmp_x*sin(- M_PI_2);

//                  tmp.y = round(tmp.y*100.0)/100.0;
//                  tmp.x = round(tmp.x*100.0)/100.0;
//                  tmp.theta = 0.0;
//                  objectsTrackedVector.push_back(tmp);
//              }
//          }

//        //Theta of the tracked element if there is one that we want to track
//        bool at_least_one = false;
//        static double x_old = 0.0;
//        static double y_old = 0.0;
//        static double theta_act = 0.0;
//        static double x_robot_old = 0.0;
//        static double y_robot_old = 0.0;

//        static double delta_x_abs = 0.0;
//        static double delta_y_abs = 0.0;

//        static double theta_array[3] = {0.0,0.0,0.0};
//        static int theta_count = 0;

//        for (std::vector<ObjectItem>::iterator it = objectsTrackedVector.begin() ; it != objectsTrackedVector.end(); ++it){
//            //Check if the trackingID still exists
//            if((*it).id == trackingID || (*it).id == last_id_tracked){
//                at_least_one = true;
//                trackingID = last_id_tracked;
//                std::unique_lock<std::mutex> lockPose(poseMutex);

//                double delta_x_rel = (*it).x-x_old;
//                double delta_y_rel = (*it).y-y_old;
//                double d = pose.x - x_robot_old;
//                double delta_x_rob_tmp = pose.x - x_robot_old;
//                double delta_y_rob_tmp = pose.y - y_robot_old;

//                double dist = sqrt(pow(delta_x_rob_tmp,2.0)+pow(delta_y_rob_tmp,2.0));
//                double theta1 = atan2(delta_y_rob_tmp,delta_x_rob_tmp);
//                double theta2 = pose.theta - theta1;
//                double delta_x_rob = dist*cos(theta2);
//                double delta_y_rob = dist*sin(theta2);

//                delta_x_abs = delta_x_rel + delta_x_rob;
//                delta_y_abs = delta_y_rel + delta_y_rob;

//                if(sqrt(pow(delta_x_abs,2)+pow(delta_y_abs,2))>0.15){
//                    theta_act = atan2(delta_y_abs,delta_x_abs);
//                    theta_array[theta_count] = theta_act;
//                    theta_count++;
//                    if(theta_count == 3) theta_count = 0;
//                    x_old = (*it).x;
//                    y_old = (*it).y;
//                    x_robot_old = pose.x;
//                    y_robot_old = pose.y;
//                }

//                lockPose.unlock();

//            //double theta_filtered = 0.0;
//            double sx = 0.0;
//            double cx = 0.0;
//            for(int i=0;i<3;i++){
//              sx += sin(theta_array[i]);
//              cx += cos(theta_array[i]);
//            }
//            //theta_filtered = atan2(sx,cx);
//            (*it).theta = atan2(sx,cx);
//          }
//        }

//        if(!at_least_one){
//            trackingID = -1;
//        }

//        nlohmann::json j_pub;
//        nlohmann::json j_array = nlohmann::json::array();

//        for (std::vector<ObjectItem>::iterator it = objectsTrackedVector.begin() ; it != objectsTrackedVector.end(); ++it){
//            nlohmann::json j_in;
//            (*it).getJson(j_in);
//            j_array.push_back(j_in);
//        }

//        j_pub["object_tracked"] = j_array;
//        j_pub["tracking_id"] = trackingID;

//        //For ovetti
//        //First of all remove all the single points
//        if(ovetti){
//            std::vector<std::vector<cv::Point>> contoursOvetti, cont_singleOvetti;
//            cv::Mat imgMaskOvetti;
//            cvtColor(img, imgMaskOvetti, cv::COLOR_BGR2GRAY);
//            cv::findContours(imgMaskOvetti, contoursOvetti, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // find external contours of each blob
//            cv::Mat newMaskOvetti = cv::Mat(imgMaskOvetti.rows, imgMaskOvetti.cols, CV_8UC1, cv::Scalar(0));

//            for (int i=0; i<contoursOvetti.size(); ++i){
//                double area = cv::contourArea(contoursOvetti[i]);
//                double perimeter = cv::arcLength(contoursOvetti[i], true);

//                if (area<10) continue; // filter too small contours to remove false positives
//                cont_singleOvetti = {contoursOvetti[i]};
//                drawContours(newMaskOvetti, cont_singleOvetti, -1, cv::Scalar(255), -1, cv::LINE_AA);
//            }

//            cv::Mat imgOvetti = newMaskOvetti.clone();

//            cvtColor(newMaskOvetti, imgOvetti, cv::COLOR_GRAY2BGR);

//            cv::Mat imgGray = imgOvetti.clone();
//            cv::cvtColor(imgGray,imgGray,CV_BGR2GRAY);
//std::vector<cv::Vec4i> linesFull; // will hold the results of the detection
//            //cv::blur(imgGray,imgGray,cv::Size(3,3));
//            cv::HoughLinesP(imgGray, linesFull, 1, M_PI/180,10,2,10); // runs the actual detection
//            cv::Mat houghMat;
//            std::vector<cv::Vec3d> linesNewTransform;
//            double rhoMin = 0.0f, rhoMax = 360.0f, rhoStep = 1;
//            double thetaMin = 0.0f, thetaMax = CV_PI / 2.0f, thetaStep = CV_PI / 180.0f;
//            cv::HoughLinesPointSet(mm->lidarPoint,linesNewTransform,20,1,rhoMin,rhoMax,rhoStep,thetaMin,thetaMax,thetaStep);
//            //houghMat.copyTo(linesNewTransform);



//            //Let survive only the lines which have enough inliers
//            std::vector<cv::Point> validPoints;
//            std::vector<cv::Vec4i> lines; // will hold the results of the detection
//            for (std::vector<cv::Vec4i>::iterator it = linesFull.begin() ; it != linesFull.end(); it++){
//                double xm = (*it)[0];
//                double ym = (*it)[1];
//                double xM = (*it)[2];
//                double yM = (*it)[3];
//                validPoints.clear();
//                utils::extractInliers(imgGray,validPoints,xm,ym,xM,yM,3);
//                if(validPoints.size()>50){
//                    lines.push_back(*it);
//                }
//            }

//            //lines = linesFull;

//            cv::cvtColor(imgGray,imgOvetti,CV_GRAY2BGR);

//            for( size_t i = 0; i < linesNewTransform.size(); i++ )
//            {
//                float rho = linesNewTransform[i][1], theta = linesNewTransform[i][2];
//                //std::cout << rho << " | " << theta << std::endl;
//                  cv::Point pt1, pt2;
//                  double a = cos(theta), b = sin(theta);
//                  double x0 = a*rho, y0 = b*rho;
//                  pt1.x = cvRound(x0 + 1000*(-b));
//                  pt1.y = cvRound(y0 + 1000*(a));
//                  pt2.x = cvRound(x0 - 1000*(-b));
//                  pt2.y = cvRound(y0 - 1000*(a));
//                  //std::cout << pt1.x << "|" << pt1.y << "|" << pt2.x << "|" << pt2.y << std::endl;
//                  //line(imgOvetti, pt1, pt2, cv::Scalar(255,0,0), 1);
//            }

//            std::vector<cv::Vec4i> linesFiltered;
//            std::vector<cv::Vec4i> linesNotPaired = lines;

//            std::vector<cv::Vec4i> winners;
//            bool active = true;
//            while (active) {
//                winners.clear();
//                active = false;
//                for (std::vector<cv::Vec4i>::iterator it = linesNotPaired.begin() ; it != linesNotPaired.end(); it++){
//                    bool entered = false;
//                    for(std::vector<cv::Vec4i>::iterator it1 = winners.begin() ; it1 != winners.end(); it1++){
//                        double distance = utils::DistanceBetweenSegments(*it,*it1);
//                        //TRACE(distance);
//                        if(distance<distance_th){
//                            entered = true;
//                            if(std::hypot((*it)[0]-(*it)[2], (*it)[1]-(*it)[3])>std::hypot((*it1)[0]-(*it1)[2], (*it1)[1]-(*it1)[3])){
//                                *it1 = *it;
//                                break;
//                            }
//                        }
//                    }
//                    if(!entered){
//                        winners.push_back(*it);
//                    }else{
//                        active = true;
//                    }

//                }
//                //TRACE(winners.size());
//                linesNotPaired = winners;
//            }
//            categoryVector.clear();
//            for (std::vector<cv::Vec4i>::iterator it = winners.begin() ; it != winners.end(); it++){
//                Category tmp;
//                tmp.leader = (*it);
//                tmp.lines.clear();
//                categoryVector.push_back(tmp);
//            }
//            //Assegna ogni linea ad un gruppo
//            for (std::vector<cv::Vec4i>::iterator it = lines.begin() ; it != lines.end(); it++){
//                for (std::vector<Category>::iterator it1 = categoryVector.begin() ; it1 != categoryVector.end(); it1++){
//                    double distance = utils::DistanceBetweenSegments(*it,(*it1).leader);
//                    //TRACE(distance);
//                    if(distance<distance_th){
//                        double x1 = (*it)[0];
//                        double x2 = (*it)[2];
//                        double y1 = (*it)[1];
//                        double y2 = (*it)[3];
//                        double m = (y2-y1)/(x2-x1);
//                        double q = y1 - m*x1;
//                        (*it1).m = ((*it1).m*(*it1).lines.size() + m)/((*it1).lines.size()+1);
//                        (*it1).q = ((*it1).q*(*it1).lines.size() + q)/((*it1).lines.size()+1);
//                        (*it1).lines.push_back(*it);
//                        break;
//                    }
//                }
//            }
//            int cnt = 0;
//            double x1;
//            double x2;
//            double y1;
//            double y2;
//            nlohmann::json j_tmp;
//            for (std::vector<Category>::iterator it1 = categoryVector.begin() ; it1 != categoryVector.end(); it1++){
//                nlohmann::json j_array_lines = nlohmann::json::array();
//                 for (std::vector<cv::Vec4i>::iterator it = (*it1).lines.begin() ; it != (*it1).lines.end(); it++){
//                     nlohmann::json j_in;
//                     cv::Vec4i l = (*it);
//                     mm->getWorldCoordinates(l[0],l[1],x1,y1);
//                     mm->getWorldCoordinates(l[2],l[3],x2,y2);
//                     j_in["x1"] = x1;
//                     j_in["x2"] = x2;
//                     j_in["y1"] = y1;
//                     j_in["y2"] = y2;
//                     j_array_lines.push_back(j_in);
//                 }
//                 //j_tmp["m"] = (*it1).m;
//                 //j_tmp["q"] = (*it1).q;
//                 j_tmp[std::to_string(cnt)] = j_array_lines;
//                 cnt++;
//            }
//            j_pub["category"] = j_tmp;


//            /*std::vector<std::pair<double,double>> categoriesParam;
//            for (std::vector<cv::Vec4i>::iterator it = lines.begin() ; it != lines.end(); it++){
//                for(std::vector<cv::Vec4i>::iterator it1 = winners.begin() ; it1 != winners.end(); it1++){
//                    double distance = utils::DistanceBetweenSegments(*it,*it1);
//                    //TRACE(distance);
//                    if(distance<5){
//                        //The line belongs to the category. Estimate m and q
//                        std::pair<double,double> estimate;
//                        estimate.first = atan2((*it)[1]-(*it)[3],(*it)[0]-(*it)[2]);
//                        estimate.second = (*it)[1] - estimate.first*(*it)[0];
//                        //Make an average with the m and q of the same set
//                    }
//                }
//            }*/

//            //Compute the estimate line filtered
//            std::vector<cv::Vec4i> winnersFiltered;
//            for (std::vector<Category>::iterator it1 = categoryVector.begin() ; it1 != categoryVector.end(); it1++){
//                cv::Vec4i l;
//                l[0] = (*it1).leader[0];
//                l[1] = (*it1).leader[0]*(*it1).m+(*it1).q;
//                l[2] = (*it1).leader[2];
//                l[3] = (*it1).leader[2]*(*it1).m+(*it1).q;
//                winnersFiltered.push_back(l);
//            }


//            nlohmann::json j_array_lines = nlohmann::json::array();
//            for( size_t i = 0; i < winners.size(); i++ )
//            {
//                cv::Vec4i l = winners[i];
//                //line(imgOvetti, cv::Point(l[0],l[1]), cv::Point(l[2],l[3]), cv::Scalar(0,0,255), 1);

//                mm->getWorldCoordinates(l[0],l[1],x1,y1);
//                mm->getWorldCoordinates(l[2],l[3],x2,y2);

//                nlohmann::json j_in;
//                j_in["x1"] = x1;
//                j_in["x2"] = x2;
//j_in["y1"] = y1;
//                j_in["y2"] = y2;
//                j_array_lines.push_back(j_in);
//            }

//            j_pub["lines"] = j_array_lines;

//            j_array_lines = nlohmann::json::array();
//            for( size_t i = 0; i < lines.size(); i++ )
//            {
//                cv::Vec4i l = lines[i];
//                line(imgOvetti, cv::Point(l[0],l[1]), cv::Point(l[2],l[3]), cv::Scalar(0,0,255), 1);

//                mm->getWorldCoordinates(l[0],l[1],x1,y1);
//                mm->getWorldCoordinates(l[2],l[3],x2,y2);

//                nlohmann::json j_in;
//                j_in["x1"] = x1;
//                j_in["x2"] = x2;
//                j_in["y1"] = y1;
//                j_in["y2"] = y2;
//                j_array_lines.push_back(j_in);
//            }

//            //j_pub["lines_starting"] = j_array_lines;




//            static bool save = true;
//            if(save){
//                std::vector<int> cp;
//                cp.push_back(CV_IMWRITE_PNG_COMPRESSION);
//                cp.push_back(1);
//                cv::imwrite("/home/stefano/test.png",img,cp);
//                save = false;
//            }

//            if(showImage){
//                cv::cvtColor(imgOvetti,imgOvetti,CV_BGR2RGB);
//                cv::imshow("RAW IMAGE", img);
//                cv::imshow("Ovetti",imgOvetti);
//                cv::imshow("IMG2",imgF);
//            }
//        }

//        std::string pub_string = j_pub.dump();
//        trackingPublisher->send("PUB_TRACKER", pub_string.c_str(), pub_string.size());

//        if(showImage && !ovetti){
//            cv::imshow("RAW IMAGE", img);
//            cv::imshow("IMG2",imgF);
//        }

//        //cv::imwrite("imgs/img"+std::to_string(count++)+".jpg", imgF);

//    }
//}

void LidarTracking::callback_timerMapProcessing() {
  cv::Mat img;
  {
    std::unique_lock<std::mutex> lockTracker(mmMutex);
    img = mm->generateImage();
  }

  if (!img.empty()) {
    cv::Mat imgF;
    cv::flip(img, imgF, 0);

    std::vector<std::vector<cv::Point>> contours, cont_single;
    cv::Mat imgMask;
    cvtColor(imgF, imgMask, cv::COLOR_BGR2GRAY);

    // Dilation
    int dilation_size = 3;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((dilation_size*2)+1, (dilation_size*2)+1));
    cv::dilate(imgMask, imgMask, kernel);

    cv::findContours(imgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // find external contours of each blob
    cv::Mat newMask = cv::Mat(imgMask.rows, imgMask.cols, CV_8UC1, cv::Scalar(0));
    std::vector<cv::Point2f> centroids;

    for (int i=0; i<contours.size(); ++i) {
      double area = cv::contourArea(contours[i]);

      cv::Rect cRect = boundingRect(contours[i]); //it is an iterator for your contours vector
      cv::Mat subImg = imgMask(cRect).clone();
      double pixels = cv::countNonZero(subImg);
      double perimeter = cv::arcLength(contours[i], true);

      //std::cerr << pixels << std::endl;
     // if (pixels < 15 || perimeter > 30) continue; // filter too small contours to remove false positives
      if (pixels < 60 || perimeter > 100) continue; 
      cont_single = {contours[i]};
      drawContours(newMask, cont_single, -1, cv::Scalar(255), -1, cv::LINE_AA);
      //std::cout << contours[i].size() << std::endl;
      cv::Moments mu = cv::moments(contours[i], false);

      // get the centroid of figures.
      if (area>0)
        centroids.push_back(cv::Point2f(mu.m10/mu.m00, mu.m01/mu.m00));
      else
        centroids.push_back(cv::Point2f(contours[i].front()));
    }

    cvtColor(newMask, imgF, cv::COLOR_GRAY2BGR);

    //Get all the centroinds

    std::vector<Object> objects;
    for (auto pt: centroids) {
      // create new element
      Object obj;
      obj.id = NXT_ID++;
      obj.centroid = pt;
      obj.cnt = 0;
      obj.obj_id = -1;
      objects.push_back(obj);
    }

    //    std::unordered_map<int, Object> tracksNew;
    //    for (auto& element: tracks)
    //    {
    //      if (visible.find(element.first) != visible.end())
    //      {
    //        if (element.second.cnt == 15 && element.second.obj_id == -1)
    //        {
    //          element.second.cnt = 100;
    //          element.second.obj_id = NXT_OBJ_ID++;
    //        }
    //        else if (element.second.cnt!=15 && element.second.cnt<100) element.second.cnt++;
    //        tracksNew.insert(element);
    //      }
    //      else
    //      {
    //        element.second.cnt--;
    //        if (element.second.cnt > 50)
    //          tracksNew.insert(element);
    //      }
    //    }
    //    tracks = tracksNew;

    //    for (auto element: tracks)
    //    {
    //      if (element.second.obj_id!=-1)
    //      {
    //        cv::circle(imgF, element.second.centroid, 4, cv::Scalar(0,0,255), -1);
    //        cv::putText(imgF, std::to_string(element.second.obj_id), element.second.centroid, 1, 1, cv::Scalar(255,0,0));
    //      }
    //    }

    objectsTrackedVector.clear();

    for (auto element: objects) {
      ObjectItem tmp;
      tmp.id = element.obj_id;
      //tmp.probability = element.second.cnt;
      double tmp_x = (double)(mm->getSize() - element.centroid.y)*mm->getResolution() ;
      double tmp_y = (double)(mm->getSize() - element.centroid.x)*mm->getResolution();

      //tmp.x = tmp_x*cos(pose.theta - M_PI_2) + tmp_y*sin(pose.theta - M_PI_2);
      //tmp.y = tmp_y*cos(pose.theta - M_PI_2) - tmp_x*sin(pose.theta - M_PI_2);

      //Perchè ruoto la mappa e tengo fermo il robot
      tmp.x = tmp_y*sin(- M_PI_2);
      tmp.y = -tmp_x*sin(- M_PI_2);

      tmp.y = round(tmp.y*100.0)/100.0;
      tmp.x = round(tmp.x*100.0)/100.0;
      tmp.theta = 0.0;
      objectsTrackedVector.push_back(tmp);
    }

    Point2d ciccia = mm->getPose();

    static double poseprevX = ciccia.x;
    static double poseprevY = ciccia.y;
    static bool initialization = true;
    static int id = 0;

    int ciccio = 4;
    static std::vector<double> dtvec(ciccio); 
    static std::vector<Point2d> posevec(ciccio);
    unsigned long now;
    now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    static unsigned long oldnow = now;
    double dt = (now-oldnow)/1000.0;
    oldnow = now;
    if (dt < 0.0001) {
      dt = 0.1;
    }
    std::rotate( dtvec.begin(), dtvec.begin() + 1, dtvec.end() );
    dtvec.back() = dt;
    std::rotate( posevec.begin(), posevec.begin() + 1, posevec.end() );
    posevec.back() = {ciccia.x,ciccia.y};

    //std::vector<bool> isupdated(objFilt.id.size(),false);
    //std::vector<bool> todelete(objFilt.id.size(),false);
    for (std::vector<ObjectItem>::iterator it = objectsTrackedVector.begin(); it != objectsTrackedVector.end(); ++it) {
      double min_d = 100;
      double d, tmp_id;

      if (!objFiltVec.empty()) {
        for (std::vector<ObjectsFilter>::iterator ip = objFiltVec.begin(); ip != objFiltVec.end(); ++ip) {
          d = std::sqrt( std::pow( (*ip).y.back()+poseprevY - ((*it).y+ciccia.y), 2 ) + std::pow( (*ip).x.back()+poseprevX - ((*it).x+ciccia.x), 2 ) ); 
          if (d<min_d) {
            min_d = d;
            tmp_id = (*ip).id;
          }
        }
      }
      
      if (min_d < 0.5) {
        // Retrieve index
        double idx;
        for (ObjectsFilter const& ipp : objFiltVec) {
          if (ipp.id == tmp_id) {
            idx = std::addressof(ipp) - std::addressof(objFiltVec[0]);
          }
        }
        double dtAcc = 0;
        // Update object
        if ( objFiltVec[idx].x.size() >= ciccio ) {
          for (int jj = 0; jj < ciccio; ++jj){
          dtAcc += dtvec[jj];
          }

          //objFiltVec[idx].vx = ( (*it).x - objFiltVec[idx].x[objFiltVec[idx].x.size() - ciccio] ) / ( ciccio*0.1 );
          //objFiltVec[idx].vy = ( (*it).y - objFiltVec[idx].y[objFiltVec[idx].y.size() - ciccio] ) / ( ciccio*0.1 );
          objFiltVec[idx].vx = ( ((*it).x + posevec[ciccio-1].x) - (objFiltVec[idx].x[objFiltVec[idx].x.size() - ciccio] + posevec[0].x) ) / ( ciccio*0.1 );
          objFiltVec[idx].vy = ( ((*it).y + posevec[ciccio-1].y) - (objFiltVec[idx].y[objFiltVec[idx].y.size() - ciccio] + posevec[0].y) ) / ( ciccio*0.1 );

	} else {
          objFiltVec[idx].vx = 0.0;
          objFiltVec[idx].vy = 0.0;
        }
        objFiltVec[idx].x.push_back((*it).x);
        objFiltVec[idx].y.push_back((*it).y);
        objFiltVec[idx].theta = (*it).theta;
        objFiltVec[idx].isupdated = true;
        objFiltVec[idx].todelete  = false;
      } else {
        // New object (or objFilt initialization)
        id++;
        ObjectsFilter newObj;
        newObj.id = id;
        newObj.x.push_back((*it).x);
        newObj.y.push_back((*it).y);
        newObj.theta = (*it).theta;
        newObj.vx = 0.0;
        newObj.vy = 0.0;
        newObj.count = 0;
        newObj.isupdated = true;
        newObj.todelete  = false;
        objFiltVec.push_back(newObj);
      }

    }

    for (std::vector<ObjectsFilter>::iterator ip = objFiltVec.begin(); ip != objFiltVec.end(); ++ip) {
      if ( (*ip).isupdated == false ) {
        (*ip).count = (*ip).count + 1; // increase miss detection count
      } else {
        (*ip).count = 0; // reset count
        (*ip).isupdated = false;
      }
      if ( (*ip).count >= 9 ) {
        (*ip).todelete = true;
      }
    }

    for(auto iter = objFiltVec.begin(); iter != objFiltVec.end(); ){
      if (iter->todelete == true) {
        iter = objFiltVec.erase(iter);
      } else {
        ++iter;
      }
    }
    
    poseprevX = ciccia.x;
    poseprevY = ciccia.y;
    
    nlohmann::json j_pub;
    nlohmann::json j_array = nlohmann::json::array();

    for (std::vector<ObjectsFilter>::iterator ip = objFiltVec.begin(); ip != objFiltVec.end(); ++ip) {
      nlohmann::json j_in;
      j_in["x"] = (*ip).x.back();
      j_in["y"] = (*ip).y.back();
      j_in["id"] = (*ip).id;
      j_in["theta"] = (*ip).theta;
      j_in["vx"] = (*ip).vx;
      j_in["vy"] = (*ip).vy;
      j_array.push_back(j_in);       
    }
      //nlohmann::json j_inobj;
      //j_inobj["obj"] = objFiltVec;
      //j_array.push_back(j_inobj);     
    //for (std::vector<ObjectItem>::iterator it = objectsTrackedVector.begin() ; it != objectsTrackedVector.end(); ++it){
    //  nlohmann::json j_in;
    //  (*it).getJson(j_in);
    //  j_array.push_back(j_in);
    //}

    j_pub["object_tracked"] = j_array;
    j_pub["tracking_id"] = trackingID;

    std::string pub_string = j_pub.dump();
    trackingPublisher->send("PUB_TRACKER", pub_string.c_str(), pub_string.size());
  }
}

//void LidarTracking::processMessage(const nlohmann::json & j_in, nlohmann::json & j_out){
//  std::cout << "Traker server received message: " << j_in.dump() << std::endl;

//  j_out["ack"] = std::string("false");

//  try{
//    if(j_in.at("cmd")=="server_alive"){
//      j_out["ack"] = std::string("true");
//    }else if(j_in.at("cmd")=="tracking_id"){
//      std::unique_lock<std::mutex> lock(trackerMutex);
//      trackingID = j_in.at("id");
//      j_out["ack"] = std::string("true");
//      last_id_tracked = trackingID;
//      lock.unlock();
//    }
//    else if(j_in.at("cmd")=="stopTracking"){
//      std::unique_lock<std::mutex> lock(trackerMutex);
//      j_out["ack"] = std::string("true");
//      trackingID = -1;
//      lock.unlock();
//    }
//  }
//  catch(std::exception & e){
//    std::cout << "Error while processing request to server tracker: " << e.what() << std::endl;
//    j_out["ack"] = std::string("false");
//  }
//}
