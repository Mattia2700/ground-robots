#include "taskexecutor.h"
#include <thread>

#include "doordisinfection.h"
#include "Navigation.hh"
#include "coverage.hh"
#include "rendezvous.hh"
std::string TaskExecutor::mapfile = "";
bool TaskExecutor::cadmap = true;
PosedMap TaskExecutor::map;

TaskExecutor::TaskExecutor(const std::vector<std::shared_ptr<Task> > &tasks)
{
  for (auto const & t: tasks) {
    this->tasks.push_back(t);
  }
}

TaskExecutor::~TaskExecutor() 
{
  stop();
}


void TaskExecutor::start()
{
  if (state == STOPPED) {
    std::thread t([&]() {
      state = RUNNING;
      while (!tasks.empty()) {
        currentTask = tasks.front();
        tasks.pop_front();
        currentTask->setExecutor(this);
        currentTask->start();
        //if (currentTask->hasFailed())
      }
      currentTask.reset();
      state = STOPPED;
    });
    t.detach();
  }
  else if (state == PAUSED) {
    if (currentTask) {
      state = RUNNING;
      currentTask->resume();
    }
  }
}

void TaskExecutor::pause()
{
  if (state == RUNNING && currentTask) {
    currentTask->pause();
    state = PAUSED;
  }
}

cv::Mat TaskExecutor::getImage()
{
  cv::Mat res;
  if (currentTask) {
    res = currentTask->getImage();
  }
  return res;
}

void TaskExecutor::stop()
{
  if (currentTask) {
    currentTask->stop();
    currentTask.reset();
  }
  tasks.clear();
  state = STOPPED;
}





void TaskExecutor::setMap(std::string filename) {
  cadmap = true;
  mapfile = filename;
}

void TaskExecutor::setMap(PosedMap const & map) {
  cadmap = false;
  TaskExecutor::map = map;
}


bool TaskExecutor::loadGoals(nlohmann::json const & j_tasks, std::vector<std::shared_ptr<Task>> & tasks)
{
  using nlohmann::json;

  tasks.clear();
  if (cadmap && mapfile.empty()) {
    std::cerr << "[ERROR] Mapfile not set." << std::endl;
    throw std::runtime_error("Cannot create the tasks.");
  }

  try {
    for (json::const_iterator it = j_tasks.begin(); it != j_tasks.end(); ++it) {
      std::string type = (*it).at("type");
      if (type == "goto") {
        std::vector<State2d> waypoints;

        json gx = (*it).at("gx");
        json gy = (*it).at("gy");
        json gtheta = (*it).at("gtheta");


        for (int i=0; i<gx.size(); ++i) {
          State2d wp;
          wp.x = gx[i];
          wp.y = gy[i];
          wp.theta = gtheta[i];
          waypoints.push_back(wp);
        }


        bool spray = false;
        if ((*it).count("spray")) {
          spray = (*it).at("spray");
        }
        if (waypoints.size()>=2) {
          std::shared_ptr<Navigation> task;
          if (cadmap) 
            task = std::make_shared<Navigation>(mapfile, waypoints);
          else
            task = std::make_shared<Navigation>(map, waypoints);
          task->setSpray(spray);
          tasks.push_back(task);
        }
        else if (waypoints.size()==1) {
          std::shared_ptr<Navigation> task;
          if (cadmap) 
            task = std::make_shared<Navigation>(mapfile, waypoints[0]);
          else
            task = std::make_shared<Navigation>(map, waypoints[0]);
          task->setSpray(spray);
          tasks.push_back(task);
        }
        else {
          std::cerr << "[WARN] GOTO task without waypoints. Skipping..." << std::endl;
        }
      }
      else if (type == "cleandoor") {
        bool spray = false;
        if ((*it).count("spray")) {
          spray = (*it).at("spray");
        }
        tasks.push_back(std::make_shared<DoorDisinfection>(spray));
      }
      else if (type == "coverage") {
        tasks.push_back(std::make_shared<Coverage>());
      }
      else if (type == "rendezvous") {
        tasks.push_back(std::make_shared<Rendezvous>());
      }
    }

    return true;
  }
  catch (std::exception & e) {
    std::cerr << "[WARN] " << e.what() << std::endl;
    return false;
  }
}

void TaskExecutor::setDataToPublish(nlohmann::json const & data) {
  {
    std::unique_lock<std::mutex> lock(dataToPublishMtx);
    dataToPublish = data;
  }
}

nlohmann::json TaskExecutor::getDataToPublish() {
  nlohmann::json ret;
  {
    std::unique_lock<std::mutex> lock(dataToPublishMtx);
    ret = dataToPublish;
  }
  return ret;
}
