#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace waypoint_follower_cpp
{
class FollowWaypoints : public rclcpp::Node
{
public:
  using Waypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleWaypoints = rclcpp_action::ClientGoalHandle<Waypoints>;

  explicit FollowWaypoints(const rclcpp::NodeOptions & options)
  : Node("waypoint_follower_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Waypoints>(
      this,
      "FollowWaypoints");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FollowWaypoints::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    // new goal message
    auto goal_msg = std::make_unique<Waypoints::Goal>();
    goal_msg->poses.push_back(geometry_msgs::msg::PoseStamped());
    goal_msg->poses.back().pose.position.x = 1.6444191238989023;
    goal_msg->poses.back().pose.position.y = -0.9912744944543527;
    goal_msg->poses.back().pose.position.z = 0.0;
    goal_msg->poses.back().pose.orientation.w = 0.9922098148020028;
    goal_msg->poses.back().pose.orientation.x = 0.0;
    goal_msg->poses.back().pose.orientation.y = 0.0;
    goal_msg->poses.back().pose.orientation.z = -0.12457802137847303;
    goal_msg->poses.back().header.frame_id = "map";
    goal_msg->poses.back().header.stamp = this->now();

    goal_msg->poses.push_back(geometry_msgs::msg::PoseStamped());
    goal_msg->poses.back().pose.position.x = 4.174689401679919;
    goal_msg->poses.back().pose.position.y = -1.8766319679599446;
    goal_msg->poses.back().pose.position.z = 0.0;
    goal_msg->poses.back().pose.orientation.w = 0.9994551859763745;
    goal_msg->poses.back().pose.orientation.x = 0.0;
    goal_msg->poses.back().pose.orientation.y = 0.0;
    goal_msg->poses.back().pose.orientation.z = 0.03300501817800784;
    goal_msg->poses.back().header.frame_id = "map";
    goal_msg->poses.back().header.stamp = this->now();

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Waypoints>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FollowWaypoints::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FollowWaypoints::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FollowWaypoints::result_callback, this, _1);
    this->client_ptr_->async_send_goal(*goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Waypoints>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<GoalHandleWaypoints::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleWaypoints::SharedPtr,
    const std::shared_ptr<const Waypoints::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Current: ";
    ss << feedback->current_waypoint << " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleWaypoints::WrappedResult & result)
  {
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
    ss << "Result received: ";
    for (auto number : result.result->missed_waypoints) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class FollowWaypoints

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_follower_cpp::FollowWaypoints)