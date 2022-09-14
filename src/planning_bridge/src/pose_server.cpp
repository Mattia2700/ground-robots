#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "rclcpp/rclcpp.hpp"

#include "planning_bridge_msgs/srv/current_pose.hpp"


class echoListener
{
public:
  /**
   * @brief Buffer to store transofrmations
   * 
   */
  tf2_ros::Buffer buffer_;
  /**
   * @brief Transform listener pointer to listen to the transformations
   * 
   */
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  /**
   * @brief Construct a new echo Listener object
   * 
   * @param clock current clock
   */
  explicit echoListener(rclcpp::Clock::SharedPtr clock)
  : buffer_(clock)
  {
    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
  }
  /**
   * @brief Destroy the echo Listener object
   */
  ~echoListener(){}
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::Rate rate(1.0);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("current_pose");

  rclcpp::Clock::SharedPtr clock = node->get_clock();
  echoListener echoListener(clock);

  rclcpp::Service<planning_bridge_msgs::srv::CurrentPose>::SharedPtr service = node->create_service<planning_bridge_msgs::srv::CurrentPose>("get_current_pose", [&](const std::shared_ptr<planning_bridge_msgs::srv::CurrentPose::Request> request, std::shared_ptr<planning_bridge_msgs::srv::CurrentPose::Response> response) {
      while (rclcpp::ok() && !echoListener.buffer_.canTransform("map", "base_link", tf2::TimePoint())) rate.sleep();

      geometry_msgs::msg::TransformStamped echo_transform = echoListener.buffer_.lookupTransform("map", "base_link", tf2::TimePoint());

      response->x = echo_transform.transform.translation.x;
      response->y = echo_transform.transform.translation.y;
  });

  rclcpp::spin(node);
  return 0;
}