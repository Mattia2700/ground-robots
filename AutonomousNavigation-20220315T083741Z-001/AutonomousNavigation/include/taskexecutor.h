#ifndef TASKEXECUTOR_H
#define TASKEXECUTOR_H

#include <vector>
#include <deque>
#include <memory>

#include "task.h"
#include "json.hpp"
#include "utils.hpp"
#include <mutex>

#include <opencv2/opencv.hpp>




class TaskExecutor
{
public:
  enum State { RUNNING, PAUSED, STOPPED };

  TaskExecutor(std::vector<std::shared_ptr<Task>> const & tasks);
  virtual ~TaskExecutor();
  virtual void start();
  virtual void stop();
  void pause();
  cv::Mat getImage();
  void setDataToPublish(nlohmann::json const & data);
  nlohmann::json getDataToPublish();

  State getState() { return state; }

  static void setMap(std::string filename);
  static void setMap(PosedMap const & map);
  static bool loadGoals(nlohmann::json const & j_tasks, std::vector<std::shared_ptr<Task>> & tasks);

private:
  std::shared_ptr<Task> currentTask;
  std::deque<std::shared_ptr<Task>> tasks;
  
  std::mutex dataToPublishMtx;
  nlohmann::json dataToPublish;

  State state = STOPPED;

  static bool cadmap;
  static std::string mapfile;
  static PosedMap map;
};

#endif // TASKEXECUTOR_H
