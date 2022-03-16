#include "RequesterSimple.hpp"
#include "Subscriber.hpp"
#include "timer.hpp"
#include "json.hpp"

#include <signal.h>
#include <chrono>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ros_layer_walker/wheels_current.h>
#include <geometry_msgs/Pose2D.h>

#include <fstream>
using namespace std;
using json = nlohmann::json;

//************************************************************************************************************************************
//************************************************************************************************************************************

enum WALKER_NODE{DESELECT = 0, FRONT_LEFT = 1, FRONT_RIGHT = 2, REAR_RIGHT = 3, REAR_LEFT = 4};

/*! \details Structure representing the front data of the walker. */
struct OdomInfo{
  double x; /*!< linear position x. */
  double y; /*!< linear position y.*/
  double theta; /*!< angular position theta.*/
  double v; /*!< linear velocity.*/
  double omega; /*!< angular velocity.*/
};

/*! \details Structure representing the front data of the walker. */
struct FrontNode{
  double speed; /*!< \a velocity of the front wheel. */
  double angle; /*!< \a angle of the wheel.*/
  int current; /*!< \a absorbed current in mA.*/
  bool isAlive; /*!< boolean identifying the status of the node.*/
  bool isDisabled; /*!< boolean identifying if the node is disabled.*/
};

//************************************************************************************************************************************
//************************************************************************************************************************************

static volatile int terminating = 0;

OdomInfo odomI;

FrontNode frontLeft; /*!< Front left node */
FrontNode frontRight; /*!< Front right node */

ZMQCommon::RequesterSimple req("tcp://192.168.3.2:5601"); 
ZMQCommon::Subscriber subLOC;
ZMQCommon::Subscriber subHW;

void intHandler(int dummy) {
	terminating = 1;
}

double timeNow() {
  using chrono::duration_cast;
  using chrono::milliseconds;
  using chrono::system_clock;
  return (long long int)duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.;
}


bool sendRequest(ZMQCommon::RequesterSimple & req, json const & jobj) {
  string in_msg = jobj.dump();
  string out_msg;
  ZMQCommon::RequesterSimple::status_t status;
  req.request(in_msg, out_msg, status);
//  cout << out_msg << endl;
  return status==ZMQCommon::RequesterSimple::status_t::STATUS_OK; // TODO: check ACK
}

bool powerEnable(ZMQCommon::RequesterSimple & req, bool enable) {
  json jobj;
  jobj["cmd"] = string("set_power_enable");
  jobj["enable"] = enable;
  return sendRequest(req, jobj);
}

bool setDeviceMode(ZMQCommon::RequesterSimple & req, int deviceMode) {
  json jobj;
  jobj["cmd"] = string("set_device_mode");
  jobj["device_mode"] = deviceMode;
  return sendRequest(req, jobj);
}

bool powerOn(ZMQCommon::RequesterSimple & req) {
  if (!powerEnable(req, true)) return false;
  if (!setDeviceMode(req, 5)) return false;
  return true;
}

bool walkerStop(ZMQCommon::RequesterSimple & req) {
  json jobj;
  jobj["cmd"] = string("walker_stop");
  return sendRequest(req, jobj);
}


bool powerOff(ZMQCommon::RequesterSimple & req) {
  return walkerStop(req) && powerEnable(req, false);
}

bool currentSteering(ZMQCommon::RequesterSimple & req, double left, double right, double acceleration) {
  json j_req;
  j_req["cmd"] = std::string("set_single_node_current");
  j_req["dest"] = WALKER_NODE::REAR_LEFT;
  j_req["value"] = acceleration;
  sendRequest(req, j_req);
  j_req["cmd"] = std::string("set_single_node_current");
  j_req["dest"] = WALKER_NODE::REAR_RIGHT;
  j_req["value"] = -acceleration;
  sendRequest(req, j_req);

  json jobj;
  jobj["cmd"] = string("set_front_current");
  jobj["right"] = right;
  jobj["left"] = left;

  return sendRequest(req, jobj);
}


void loc_callback(const char *topic, const char *buf, size_t size, void *data){
  json j;
  try{

    j = json::parse(string(buf, size));
    //j = j.at("state");
    odomI.x = round((double)j.at("state").at("x")*100)/100.0;
    odomI.y = round((double)j.at("state").at("y")*100)/100.0;
    //int z = j.at("z");
    odomI.theta = j.at("state").at("theta");
    odomI.omega = j.at("state").at("omega");
    odomI.v = j.at("state").at("v");
  
    /*double covxx = j.at("covariance")[0];
    double covxy = j.at("covariance")[1];
    double covyy = j.at("covariance")[2];
    double tmp[4] = {covxx,covxy,covxy,covyy};
    QGenericMatrix<2, 2, double> cov(tmp);
    cout << j.dump() << endl << endl;*/
    
  }catch(exception &e){
    cerr << "error parsing: " << e.what() << endl;
  }
}

void odometry(ros::Publisher odom_pub, tf::TransformBroadcaster odom_broadcaster, ros::Time current_time)
{
	double x = odomI.x;
	double y = odomI.y;
	double th = odomI.theta;

	double vx = odomI.v*cos(th);
	double vy = odomI.v*sin(th);
	double vth = odomI.omega;

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vth;

	//publish the message
	odom_pub.publish(odom);
}

void hwInfo(ros::Publisher hw_pub)
{
	//we'll publish the wheels angles message over ROS
	geometry_msgs::Pose2D infos;
	infos.x = frontLeft.angle;
  infos.y = frontRight.angle;

	//publish the message
	hw_pub.publish(infos);
}

void hw_callback(const char *topic, const char *buf, size_t size, void *data){
  json j;
  json j_out;
  try{
    j = json::parse(string(buf, size));

    j_out = j.at("1");
    frontLeft.isDisabled = false;
    j_out = j_out.at("state");
    frontLeft.angle = j_out.at("pos");
    frontLeft.speed = j_out.at("vel");
    frontLeft.current = j_out.at("cur");
    j_out =  j.at("1").at("status");
    frontLeft.isAlive = j_out.at("alive");
    frontLeft.isDisabled = true;

    j_out = j.at("2");
    frontRight.isDisabled = false;
    j_out = j_out.at("state");
    frontRight.angle = j_out.at("pos");
    frontRight.speed = j_out.at("vel");
    frontRight.current = j_out.at("cur");
    j_out = j.at("2").at("status");
    frontRight.isAlive = j_out.at("alive");
    frontRight.isDisabled = true;

  }catch(exception &e){
    cerr << "error parsing: " << e.what() << endl;
  }
}

void command_torque(const geometry_msgs::Pose2D &curr)
{
	currentSteering(req,curr.x,curr.y,curr.theta);
}


int main (int argc, char *argv[])
{  
  ZMQCommon::RequesterSimple arucolist("tcp://192.168.3.1:5567");
  ZMQCommon::RequesterSimple::status_t status;
  string response;

  ifstream ifs("/home/mattiafranzin/Desktop/tirocinio/src/ros_layer_walker/ros_layer_walker/listaQR.json");
  json jobj_aruco = json::parse(ifs);
  cout << sendRequest(arucolist,jobj_aruco) << endl;
  arucolist.close();
  
  // Subscribe to the location ZMQ publisher of the robot
  subLOC.register_callback([&](const char *topic, const char *buf, size_t size, void *data){loc_callback(topic,buf,size,data);});
  subLOC.start("tcp://192.168.3.1:5563","LOC");

  // Subscribe to the hardware information ZMQ publisher of the robot
  subHW.register_callback([&](const char *topic, const char *buf, size_t size, void *data){hw_callback(topic,buf,size,data);});
  subHW.start("tcp://192.168.3.2:6000","PUB_HW");

  while (!terminating && !powerOn(req)) {
    cerr << "[WARN] Cannot start robot..." << endl;
    this_thread::sleep_for(chrono::milliseconds(100));
  }

  if (terminating) return 0;

  ros::init(argc, argv, "ros_layer_walker");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Publisher hw_pub = n.advertise<geometry_msgs::Pose2D>("wheels_angle", 50);

  ros::Subscriber ros_sub = n.subscribe("cmd_current", 50, command_torque);

  ros::Time current_time;

  ros::Rate r(20.0);
  while(n.ok() && !terminating){
    ros::spinOnce();
    current_time = ros::Time::now();
    
    odometry(odom_pub,odom_broadcaster,current_time);

    hwInfo(hw_pub);

    r.sleep();
  }
  
  subLOC.stop();
  walkerStop(req);
  powerOff(req);
  req.close();
      
  return 0;
}
