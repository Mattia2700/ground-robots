#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

#include "waypoint_msgs/srv/current_pose.hpp"

#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace waypoint_follower_cpp {
class FollowWaypoints : public rclcpp::Node {
public:
  using Waypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleWaypoints = rclcpp_action::ClientGoalHandle<Waypoints>;
  using CurrentPose = waypoint_msgs::srv::CurrentPose;
  using ServiceResponseFuture = rclcpp::Client<CurrentPose>::SharedFuture;
  using PoseInt = long unsigned int;

  explicit FollowWaypoints(const rclcpp::NodeOptions &options)
      : Node("waypoint_follower_client", options) {
    this->waypoint_client_ptr_ =
        rclcpp_action::create_client<Waypoints>(this, "FollowWaypoints");

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&FollowWaypoints::send_goal, this));

    this->current_pose_ = std::make_shared<geometry_msgs::msg::Pose>();
    this->current_pose_client_ptr_ =
        this->create_client<CurrentPose>("get_current_pose");
  }

  void get_pose() {
    auto future_pose = this->current_pose_client_ptr_->async_send_request(
        std::make_unique<CurrentPose::Request>(),
        [this](ServiceResponseFuture future) {
          auto result = future.get();
          RCLCPP_INFO(
              this->get_logger(), "%f, %f, %f", result->current_pose.position.x,
              result->current_pose.position.y, result->current_pose.position.z);
        });
    // auto result = future_pose.get();
    // RCLCPP_INFO(this->get_logger(), "%f %f %f",
    // result->current_pose.position.x, result->current_pose.position.y,
    // result->current_pose.position.z);
  }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->waypoint_client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    // get_pose();

    // create new pose array
    auto points = std::make_shared<geometry_msgs::msg::PoseArray>();
    auto point = std::make_shared<geometry_msgs::msg::Pose>();
    point->position.x = 1.7952730655670166;
    point->position.y = -1.1385191679000854;
    point->position.z = 0.0803813561797142;
    points->poses.push_back(*point);
    point->position.x = 4.782073497772217;
    point->position.y = 0.4766668677330017;
    point->position.z = 0.03367391973733902;
    points->poses.push_back(*point);
    point->position.x = 0.4081433117389679;
    point->position.y = -0.03935748711228371;
    point->position.z = 0.046331923454999924;
    points->poses.push_back(*point);

    auto last_quaternion = std::make_shared<geometry_msgs::msg::Quaternion>();

    // find quaternion of the next point (using next point and following one)
    for (PoseInt i = 0; i < points->poses.size() - 1; i++) {
      auto next_point = points->poses[i].position;
      auto following_next_point = points->poses[i + 1].position;

      auto quaternion = std::make_shared<geometry_msgs::msg::Quaternion>();
      auto cross_result = std::make_shared<geometry_msgs::msg::Vector3>();
      cross_result->x = next_point.y * following_next_point.z -
                        next_point.z * following_next_point.y;
      cross_result->y = next_point.z * following_next_point.x -
                        next_point.x * following_next_point.z;
      cross_result->z = next_point.x * following_next_point.y -
                        next_point.y * following_next_point.x;
      quaternion->x = cross_result->x;
      quaternion->y = cross_result->y;
      quaternion->z = cross_result->z;

      double dot_result = next_point.x * following_next_point.x +
                          next_point.y * following_next_point.y +
                          next_point.z * following_next_point.z;
      double length1_squared = next_point.x * next_point.x +
                               next_point.y * next_point.y +
                               next_point.z * next_point.z;
      double length2_squared = following_next_point.x * following_next_point.x +
                               following_next_point.y * following_next_point.y +
                               following_next_point.z * following_next_point.z;
      double length_result = std::sqrt(length1_squared * length2_squared);
      quaternion->w = dot_result + length_result;

      RCLCPP_INFO(this->get_logger(), "%f", dot_result/length_result);
      if(std::abs((float)dot_result/length_result) > 0.97) {
        quaternion->w = 0;
        quaternion->x = next_point.y + next_point.z;
        quaternion->y = next_point.z - next_point.x;
        quaternion->z = - next_point.x - next_point.y;
      }
      

      double quaternion_norm = std::sqrt(
          quaternion->x * quaternion->x + quaternion->y * quaternion->y +
          quaternion->z * quaternion->z + quaternion->w * quaternion->w);
      quaternion->x /= quaternion_norm;
      quaternion->y /= quaternion_norm;
      quaternion->z /= quaternion_norm;
      quaternion->w /= quaternion_norm;

      points->poses[i].orientation = *quaternion;

      RCLCPP_INFO(
          this->get_logger(), "%f %f %f %f", points->poses[i].orientation.x,
          points->poses[i].orientation.y, points->poses[i].orientation.z,
          points->poses[i].orientation.w);

      last_quaternion = quaternion;
    }

    // add last quaternion to last point
    points->poses[points->poses.size() - 1].orientation = *last_quaternion;

    RCLCPP_INFO(this->get_logger(), "%f %f %f %f",
                points->poses[points->poses.size() - 1].orientation.x,
                points->poses[points->poses.size() - 1].orientation.y,
                points->poses[points->poses.size() - 1].orientation.z,
                points->poses[points->poses.size() - 1].orientation.w);

    // // new goal message
    auto goal_msg = std::make_unique<Waypoints::Goal>();
    for (PoseInt i = 0; i < points->poses.size(); i++) {
      goal_msg->poses.push_back(geometry_msgs::msg::PoseStamped());
      goal_msg->poses.back().pose = points->poses[i];
      goal_msg->poses.back().header.stamp = this->now();
      goal_msg->poses.back().header.frame_id = "map";
    }
    // goal_msg->poses.back().pose.orientation.x = 0.672013521194458;
    // goal_msg->poses.back().pose.orientation.y = 0.21718136966228485;
    // goal_msg->poses.back().pose.orientation.z = -0.6736872792243958;
    // goal_msg->poses.back().pose.orientation.w = 0.21765950322151184;

    // goal_msg->poses.back().pose.orientation.x = -0.20075558125972748;
    // goal_msg->poses.back().pose.orientation.y = 0.6844624876976013;
    // goal_msg->poses.back().pose.orientation.z = 0.17868560552597046;
    // goal_msg->poses.back().pose.orientation.w = 0.6777018904685974;

    // goal_msg->poses.back().pose.orientation.x = -0.7176815271377563;
    // goal_msg->poses.back().pose.orientation.y = 0.020915275439620018;
    // goal_msg->poses.back().pose.orientation.z = 0.6955828070640564;
    // goal_msg->poses.back().pose.orientation.w = 0.025695830583572388;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<Waypoints>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&FollowWaypoints::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&FollowWaypoints::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&FollowWaypoints::result_callback, this, _1);
    this->waypoint_client_ptr_->async_send_goal(*goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Waypoints>::SharedPtr waypoint_client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<CurrentPose>::SharedPtr current_pose_client_ptr_;
  geometry_msgs::msg::Pose::SharedPtr current_pose_;

  void goal_response_callback(
      std::shared_future<GoalHandleWaypoints::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void
  feedback_callback(GoalHandleWaypoints::SharedPtr,
                    const std::shared_ptr<const Waypoints::Feedback> feedback) {
    std::stringstream ss;
    ss << "Current: ";
    ss << feedback->current_waypoint << " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleWaypoints::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }
    std::stringstream ss;
    if (result.result->missed_waypoints.size() > 0) {
      ss << "Missed waypoints: ";
      for (auto number : result.result->missed_waypoints) {
        ss << number << " ";
      }
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "No missed waypoints");
    }
    rclcpp::shutdown(); // or wait for other points
  }
}; // class FollowWaypoints

} // namespace waypoint_follower_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_follower_cpp::FollowWaypoints)