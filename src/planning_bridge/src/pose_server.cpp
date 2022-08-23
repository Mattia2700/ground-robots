// #include "geometry_msgs/msg/pose.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "tf2_msgs/msg/tf_message.hpp"
// #include "planning_bridge_msgs/srv/current_pose.hpp"

// #include <tf2/exceptions.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>

// #include <memory>

// #include <mutex>
// #include <condition_variable>
// std::condition_variable g_cv;
// std::mutex g_mutex;
// bool g_isReady = false; 

// namespace current_pose_server {
// class CurrentPose : public rclcpp::Node {
// public:
//   explicit CurrentPose() : Node("current_pose") {

//     // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//     // transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//     // try {
//     //   transformStamped_  = tf_buffer_->lookupTransform(
//     //     "map", "odom",
//     //     tf2::TimePointZero);

//     //   RCLCPP_INFO(this->get_logger(), "Current pose: %f %f %f",
//     //     transformStamped_.transform.translation.x,
//     //     transformStamped_.transform.translation.y,
//     //     transformStamped_.transform.rotation.w);
//     // } catch (tf2::TransformException & ex) {
//     //   RCLCPP_INFO(
//     //     this->get_logger(), "Could not transform %s to %s: %s",
//     //     "odom", "map", ex.what());
//     //   return;
//     // }

//     std::thread t(&CurrentPose::subscribe_tf, this);

//     this->tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
//         "tf", 10, [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
//           // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reading tf topic");
//           if (msg->transforms[0].header.frame_id == "odom") {
//             this->last_pose_->position.x = msg->transforms[0].transform.translation.x;
//             this->last_pose_->position.y = msg->transforms[0].transform.translation.y;
//             this->last_pose_->position.z = msg->transforms[0].transform.translation.z;
//             this->last_pose_->orientation.x = msg->transforms[0].transform.rotation.x;
//             this->last_pose_->orientation.y = msg->transforms[0].transform.rotation.y;
//             this->last_pose_->orientation.z = msg->transforms[0].transform.rotation.z;
//             this->last_pose_->orientation.w = msg->transforms[0].transform.rotation.w;
//           }
//         } else if (msg->transforms[0].header.frame_id == "map") {
//           this->last_pose_->position.x = msg->transforms[0].transform.translation.x;
//           this->last_pose_->position.y = msg->transforms[0].transform.translation.y;
//           this->last_pose_->position.z = msg->transforms[0].transform.translation.z;
//           this->last_pose_->orientation.x = msg->transforms[0].transform.rotation.x;
//           this->last_pose_->orientation.y = msg->transforms[0].transform.rotation.y;
//           this->last_pose_->orientation.z = msg->transforms[0].transform.rotation.z;
//           this->last_pose_->orientation.w = msg->transforms[0].transform.rotation.w;
//         }
//         );
//     this->service_ = this->create_service<planning_bridge_msgs::srv::CurrentPose>(
//         "get_current_pose",
//         [this](const std::shared_ptr<planning_bridge_msgs::srv::CurrentPose::Request> request, std::shared_ptr<planning_bridge_msgs::srv::CurrentPose::Response> response) {
//           RCLCPP_INFO(this->get_logger(), "Received request");
//           response->current_pose = *this->last_pose_;
//           RCLCPP_INFO(this->get_logger(), "Sending response with position (%f,%f,%f) and orientation (%f,%f,%f,%f)",
//                       response->current_pose.position.x, response->current_pose.position.y, response->current_pose.position.z,
//                       response->current_pose.orientation.x, response->current_pose.orientation.y, response->current_pose.orientation.z, response->current_pose.orientation.w);
//         });
//   }

// private:
//   rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
//   rclcpp::Service<planning_bridge_msgs::srv::CurrentPose>::SharedPtr service_;
//   std::shared_ptr<geometry_msgs::msg::Pose> last_pose_ =
//       std::make_shared<geometry_msgs::msg::Pose>();
//   std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
//   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//   geometry_msgs::msg::TransformStamped transformStamped_;

//   void subscribe_tf(){
//     tf_sub_ = create_subscription< tf2_msgs::msg::TFMessage>(
//       "tf", 10, std::bind(&CurrentPose::tf_callback, this, _1));
//   }

//   void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg){
//     if (msg->transforms[0].header.frame_id == "odom") {
//       this->last_pose_->position.x = msg->transforms[0].transform.translation.x;
//       this->last_pose_->position.y = msg->transforms[0].transform.translation.y;
//       this->last_pose_->position.z = msg->transforms[0].transform.translation.z;
//       this->last_pose_->orientation.x = msg->transforms[0].transform.rotation.x;
//       this->last_pose_->orientation.y = msg->transforms[0].transform.rotation.y;
//       this->last_pose_->orientation.z = msg->transforms[0].transform.rotation.z;
//       this->last_pose_->orientation.w = msg->transforms[0].transform.rotation.w;
//     } else if (msg->transforms[0].header.frame_id == "map") {
//       this->last_pose_->position.x = msg->transforms[0].transform.translation.x;
//       this->last_pose_->position.y = msg->transforms[0].transform.translation.y;        this->last_pose_->position.z = msg->transforms[0].transform.translation.z;
//       this->last_pose_->orientation.x = msg->transforms[0].transform.rotation.x;
//       this->last_pose_->orientation.y = msg->transforms[0].transform.rotation.y;
//       this->last_pose_->orientation.z = msg->transforms[0].transform.rotation.z;
//       this->last_pose_->orientation.w = msg->transforms[0].transform.rotation.w;
//     }
//   }

// };
// } // namespace current_pose_server

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);

//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to return pose.");

//   rclcpp::spin(std::make_shared<current_pose_server::CurrentPose>());
//   rclcpp::shutdown();
//   return 0;
// }

/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2015, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  explicit echoListener(rclcpp::Clock::SharedPtr clock)
  : buffer_(clock)
  {
    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
  }

  ~echoListener()
  {
  }
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

      response->current_pose.x = echo_transform.transform.translation.x;
      response->current_pose.y = echo_transform.transform.translation.y;
      response->current_pose.z = echo_transform.transform.translation.z;
  });

  rclcpp::spin(node);
  return 0;
}