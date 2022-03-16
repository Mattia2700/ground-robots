#include <iostream>
#include <iomanip>
#include <atomic>
#include <future>
#include <thread>
#include <signal.h>


#include "Publisher.hpp"
#include "Subscriber.hpp"
#include "Replier.hpp"
#include "json.hpp"

using namespace std;
using namespace nlohmann;


static const double LEFT_INCREMENTS_PER_TOUR = 294912.;
static const double RIGHT_INCREMENTS_PER_TOUR = 294570.;
static const double LEFT_B_D = 1.5764889187282987 * 1.08203;
static const double RIGHT_B_D = 1.5784089237419516 * 1.08203;


static const double RATIO_LEFT_RIGHT = RIGHT_B_D/LEFT_B_D;
static const double RADIUS_SCALING = 0.476; //0.45971498589878657;
static const double LEFT_RADIUS = RADIUS_SCALING;
static const double RIGHT_RADIUS = LEFT_RADIUS/RATIO_LEFT_RIGHT;

//static const double WHEEL_RADIUS = 0.125; //0.162; // 0.1291; //0.1288659793814433; //0.125;
//static const double WHEEL_BASE = 0.387; //0.96; //0.387;
//static const double WHEEL_RADIUS = 0.031;
//static const double WHEEL_BASE = 0.2975;

struct OdomData {
  double x, y, theta;
  double v, omega;
  double last_odom_update;
  std::mutex mtx;

  OdomData():
    x(-6.5), y(17.0), theta(M_PI), v(0), omega(0), last_odom_update(0)
    //x(340), y(-16.5), theta(M_PI), v(0), omega(0), last_odom_update(0)
  {}
} odomData;

struct WheelSpeed {
  double dTick;
  //double lastTick;
  double lastTime;
  bool init = false;
};

WheelSpeed wsLeft, wsRight;



static volatile int keepRunning = 1;

void intHandler(int dummy) {
  keepRunning = 0;
}


void run();
void updateOdometry(OdomData & odomData, double current_time, WheelSpeed const & vl, WheelSpeed const & vr);
void publishOdometry(Common::Publisher & pub, OdomData & odomData);
void processMessage(const nlohmann::json & j_in, nlohmann::json & j_out, OdomData & odomData);


int main(int argc, char* argv[]) {
  signal(SIGINT, intHandler);

  if (argc!=1 && argc!=4) {
    std::cerr << "Usage: \"" << argv[0] << "\" or \"" << argv[0] << " x0 y0 theta0\"" << std::endl;
    return 0;
  }

  if (argc==4) {
    odomData.x = std::atof(argv[1]);
    odomData.y = std::atof(argv[2]);
    odomData.theta = std::atof(argv[3]);
  }

  run();
  return 0;
}




void run() {
  Common::Publisher publisherOdom("tcp://*:9128");
  Common::Replier odomServer;
  ZMQCommon::Subscriber hwSubscriber;

  bool init = false;
  double tl0;
  double tr0;

  hwSubscriber.register_callback([&](const char *topic, const char *buf, size_t size, void *data) {
    nlohmann::json j;
    try
    {
      j = nlohmann::json::parse(std::string(buf, size));
      double vl = j.at("4").at("state").at("vel");
      double vr = j.at("3").at("state").at("vel");
      vr = -vr;

      // double tl = j.at("4").at("state").at("tck");
      // double tr = j.at("3").at("state").at("tck");

      //std::cerr << j.at("4").at("state").at("vel") << std::endl;

      // if (!init) {
      //   init = true;
      //   tl0 = tl;
      //   tr0 = tr;
      // }
      // else {
      //   std::cout << (tl-tl0) << " ";
      //   std::cout << (tr-tr0) << std::endl;
      // }

      double current_time = j.at("time"); 
      current_time /= 1000.; // convert from ms to s

//      double pL = j.at("4").at("state").at("tck");
//      double pR = j.at("3").at("state").at("tck");
//      pL = -pL;

      if (!wsLeft.init) {
        wsLeft.dTick = 0;
        //wsLeft.lastTick = pL;
        wsLeft.lastTime = current_time;
        wsLeft.init = true;
      }
      else {
        wsLeft.dTick = (vl*36/(2*M_PI)*2048)*(current_time-wsLeft.lastTime); //pL-wsLeft.lastTick;
        //wsLeft.lastTick = pL;
        wsLeft.lastTime = current_time;
      }

      if (!wsRight.init) {
        wsRight.dTick = 0;
        //wsRight.lastTick = pR;
        wsRight.lastTime = current_time;
        wsRight.init = true;
      }
      else {
        wsRight.dTick = (vr*36/(2*M_PI)*2048)*(current_time-wsRight.lastTime); //pR-wsRight.lastTick;
        //wsRight.lastTick = pR;
        wsRight.lastTime = current_time;
      }

      //std::cerr << vl << " " << vr << std::endl;
      updateOdometry(odomData, current_time, wsLeft, wsRight);

      //updateOdometry(odomData, current_time, vl, vr);
      publishOdometry(publisherOdom, odomData);
    }
    catch(std::exception &e) {
      std::cerr << e.what() << std::endl;
    }
  });

  odomServer.register_callback([&](const std::string& received, void * data, std::string & tosend) {
    json j_in;
    json j_out;
    try {
      j_in = nlohmann::json::parse(std::string(received));
    } catch(std::exception &e) {
      std::cerr << "Error parsing json callback locServer" << std::endl;
      std::cerr << "Messagge:" << std::endl;
      std::cerr << std::string(received) << std::endl;
      std::cerr << e.what() << std::endl;
      return;
    }
    processMessage(j_in, j_out, odomData);
    tosend = j_out.dump();
  }, nullptr);

  odomServer.start("tcp://*:9129");
  hwSubscriber.start("tcp://10.196.80.122:6000", "PUB_HW");
  //hwSubscriber.start("tcp://127.0.0.1:6000", "PUB_HW");

  while (keepRunning) {
    std::this_thread::sleep_for(200ms);
  }

  odomServer.stop();
  hwSubscriber.stop();
}

void updateOdometry(OdomData & odomData, double current_time, WheelSpeed const & wl, WheelSpeed const & wr) {
  std::unique_lock<std::mutex> lock(odomData.mtx);
  double seconds_since_last_update = current_time - odomData.last_odom_update;
  if (seconds_since_last_update == 0.) {
    return;
  }

  odomData.last_odom_update = current_time;

  double ticks_l = wl.dTick;
  double ticks_r = wr.dTick;

  //std::cerr << ticks_l << " " << (ticks_l/LEFT_INCREMENTS_PER_TOUR)/seconds_since_last_update << std::endl;
  
  double df = 0.5 * (ticks_l/LEFT_INCREMENTS_PER_TOUR * 2*M_PI * LEFT_RADIUS + ticks_r/RIGHT_INCREMENTS_PER_TOUR * 2*M_PI * RIGHT_RADIUS);
  double dtheta = (ticks_r/RIGHT_INCREMENTS_PER_TOUR * 2*M_PI * 2./RIGHT_B_D - ticks_l/LEFT_INCREMENTS_PER_TOUR * 2*M_PI * 2./LEFT_B_D);

  double dx = df * std::cos ( odomData.theta + 0.5*dtheta );
  double dy = df * std::sin ( odomData.theta + 0.5*dtheta );

  odomData.x += dx;
  odomData.y += dy;
  odomData.theta += dtheta;
  odomData.omega = dtheta/seconds_since_last_update;
  odomData.v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;
}

void publishOdometry(Common::Publisher & pub, OdomData & odomData) {
  double x, y, theta, v, omega, time;
  {
    std::unique_lock<std::mutex> lock(odomData.mtx);
    x        = odomData.x;
    y        = odomData.y;
    theta    = odomData.theta;
    v        = odomData.v;
    omega    = odomData.omega;
    time     = odomData.last_odom_update;
  }

  json j_state;
  j_state["x"]        = x;
  j_state["y"]        = y;
  j_state["theta"]    = theta;
  j_state["v"]        = v;
  j_state["omega"]    = omega;

  json jobj;
  jobj["time"] = time*1000.;
  jobj["state"] = j_state;

  string msg = jobj.dump();
  pub.send("LOC", msg.c_str(), msg.size());
}

void processMessage(const nlohmann::json & j_in, nlohmann::json & j_out, OdomData & odomData) {
  std::cout << "Odom. server received message: " << j_in.dump() << std::endl;
  j_out["ack"] = std::string("false");

  try {
    if (j_in.at("cmd")=="server_alive") {
      j_out["ack"] = std::string("true");
    }
    else if (j_in.at("cmd")=="set") {
      double x = j_in.at("x"), y = j_in.at("y"), theta = j_in.at("theta");
      {
        std::unique_lock<std::mutex> lock(odomData.mtx);
        odomData.x = x;
        odomData.y = y;
        odomData.theta = theta;
      }
      j_out["ack"] = std::string("true");
    }
  }
  catch (std::exception & e) {
    std::cout << "Error while processing request to server RealSense loc.: " << e.what() << std::endl;
    j_out["ack"] = std::string("false");
  }

  return;
}
