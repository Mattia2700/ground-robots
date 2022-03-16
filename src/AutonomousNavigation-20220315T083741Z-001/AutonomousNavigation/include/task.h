#ifndef TASK_H
#define TASK_H

#include <opencv2/opencv.hpp>

class TaskExecutor;

class Task {
public:
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void pause() = 0;
  virtual void resume() = 0;
  virtual cv::Mat getImage() = 0;
  virtual void setExecutor(TaskExecutor * executor) {
    this->executor = executor;
  }

protected:
  Task() {}
  
  TaskExecutor * executor = nullptr;

};

#endif // TASK_H
