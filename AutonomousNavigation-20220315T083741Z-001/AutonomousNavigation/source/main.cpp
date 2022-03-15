#include "Publisher.hpp"
#include "Replier.hpp"
#include "json.hpp"
#include "utils.hpp"
#include "MapManager.hh"

#include "taskexecutor.h"
#include "hardwareglobalinterface.h"

#include <string>
#include <vector>
#include <fstream>
#include <unistd.h>
#include "timer.hpp"

#include <omp.h>

using namespace nlohmann;
int R_ID = 2;

void processMessage(const json & j_in, json & j_out);
void run(std::string mapfile);
void publishState(Common::Publisher & publisher);
std::unique_ptr<TaskExecutor> executor;

int main(int argc, char* argv[])
{
// omp_set_dynamic(0);
//  omp_set_num_threads(2);
  std::cout<< omp_get_max_threads() << std::endl;
  if (argc<2) {
    std::cerr << "Usage: " << argv[0] << " <mapfile>" << std::endl;
    return 0;
  }

  run(argv[1]);
  return 0;
}


void run(std::string mapfile)
{  
  HardwareParameters hpRobot(R_ID);
  HardwareGlobalInterface::initialize(&hpRobot);

  bool cadmap = !(hasEnding(mapfile, "yml") || hasEnding(mapfile, "yaml"));

  if (cadmap) {
    TaskExecutor::setMap(mapfile);
  }
  else {
    PosedMap map;
    if (!loadMapFile(mapfile, map)) {
      throw std::runtime_error("Failed to parse yaml file \"" + mapfile + "\"");
    }
    TaskExecutor::setMap(map);
  }
  

  

  Common::Replier navServer; 

  navServer.register_callback([&](const std::string& received, void * data, std::string & tosend){
    json j_in; //= nlohmann::json::parse(received);
    json j_out;
    try{
      j_in = json::parse(std::string(received));
    }catch(std::exception &e){
      std::cerr << "error parsing json callback autnav: " << std::endl;
      return;
    }

    processMessage(j_in, j_out);
    tosend = std::string("");
    tosend = j_out.dump() + '\0';
  }, nullptr);
Common::Publisher navPublisher(hpRobot.autnavPublisher);
std::unique_ptr<Timer> pubTimer = std::make_unique<Timer>();
pubTimer->registerCallback([&](){ publishState(navPublisher); });
pubTimer->start(200); //250
  navServer.start("tcp://*:1991");

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}


void processMessage(const json & j_in, json & j_out){
  std::cout << "AutNav server received message: " << j_in.dump() << std::endl;

  j_out["ack"] = std::string("false");

  try{

    if(j_in.at("cmd")=="server_alive") {
      j_out["ack"] = std::string("true");
    } else if(j_in.at("cmd")=="start") {
      if (!executor || executor->getState() == TaskExecutor::STOPPED) {
        json j_tasks = j_in["tasks"];
        
        std::vector<std::shared_ptr<Task>> tasks;
        if (!TaskExecutor::loadGoals(j_tasks, tasks))
        {
          std::cerr << "[WARN] Error during the parsing of the tasks" << std::endl;          
        }
        else {
          executor = std::make_unique<TaskExecutor>(tasks);
          executor->start();
          j_out["ack"] = std::string("true");
        }
      }
    }
    else if(j_in.at("cmd")=="pause"){
      if (executor) {
        if (executor->getState() == TaskExecutor::RUNNING) {
          executor->pause();
        }
        else if (executor->getState() == TaskExecutor::PAUSED) {
          executor->start();
        }
        j_out["ack"] = std::string("true");
      }
    }
    else if(j_in.at("cmd")=="stop"){
      if (executor) {
        executor->stop();
        executor.reset();
        j_out["ack"] = std::string("true");
      }
    }
  }
  catch(std::exception & e){
    std::cout << "Error while processing request to server AutNav: " << e.what() << std::endl;
    j_out["ack"] = std::string("false");
  }

  return;
}





void publishState(Common::Publisher & publisher) {
  json j_msg;
  j_msg["id"] = R_ID;
  RobotStatus::LocalizationData locData;
  std::vector<PersonItem> trackedPeople;
  HardwareGlobalInterface::getInstance().getLocalizationDataRealSense(locData);
  HardwareGlobalInterface::getInstance().getTrackingData(trackedPeople); 

  j_msg["x"] = locData.x;
  j_msg["y"] = locData.y;
  j_msg["theta"] = locData.theta;

  if (executor)
    j_msg["data"] = executor->getDataToPublish();
  else 
    j_msg["data"] = {};
  std::string str = j_msg.dump(2);

  //std::cout << "PUB x: " << locData.x << " PUB_y: " << locData.y << std::endl;
  publisher.send("NSTAT", str.c_str(), str.size());
}




