#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "planning_bridge_msgs/srv/start_navigation.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

/**
 * @brief A simple node with anaction client that requests the navigation to a goal pose, passed by a service request
 * 
 */
class Nav2Pose : public rclcpp::Node {
public:
  using ActionNav2Pose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav2Pose = rclcpp_action::ClientGoalHandle<ActionNav2Pose>;

  /**
   * @brief Creates the "NavigateToPose" action client and initialize the custom StartNavigation service server
   * 
   * @param options default node options passed by ROS2
   */
  explicit Nav2Pose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("nav_2_pose_client", options) {
    action_nav_pose_ptr_ =
        rclcpp_action::create_client<ActionNav2Pose>(this, "navigate_to_pose");

    start_navigation_service_ptr_ = create_service<
        planning_bridge_msgs::srv::StartNavigation>(
        "start_navigation",
        [this](
            const std::shared_ptr<planning_bridge_msgs::srv::StartNavigation::Request>
                request,  
            std::shared_ptr<planning_bridge_msgs::srv::StartNavigation::Response>
                response) {
          goal_->pose.position.x = request->x;
          goal_->pose.position.y = request->y;
          goal_->pose.position.z = 1.5;
          goal_->pose.orientation.x = 0.0;
          goal_->pose.orientation.y = 0.0;
          goal_->pose.orientation.z = 0.0;
          goal_->pose.orientation.w = 1.0;
          goal_->header.frame_id = "map";
          goal_->header.stamp = now();
          send_goal();
          response->success = true;
        });
  }

  /**
   * @brief Sends the goal to the action server
   * 
   */
  void send_goal() {
    using namespace std::placeholders;

    if (!action_nav_pose_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto goal_msg = std::make_unique<ActionNav2Pose::Goal>();
    goal_msg->pose = *goal_;

    auto send_goal_options =
        rclcpp_action::Client<ActionNav2Pose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&Nav2Pose::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&Nav2Pose::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&Nav2Pose::result_callback, this, _1);
    action_nav_pose_ptr_->async_send_goal(*goal_msg, send_goal_options);
  }

private:
  /**
   * @brief Action client pointer
   */
  rclcpp_action::Client<ActionNav2Pose>::SharedPtr action_nav_pose_ptr_;
  /**
   * @brief Service server pointer
   */
  rclcpp::Service<planning_bridge_msgs::srv::StartNavigation>::SharedPtr start_navigation_service_ptr_;
  /**
   * @brief Goal pose pointer recevied by the service request
   */
  std::shared_ptr<geometry_msgs::msg::PoseStamped> goal_ = std::make_shared<geometry_msgs::msg::PoseStamped>();

  /**
   * @brief Callback function called when the action server responds to the goal request
   * 
   * @param future Informations about the goal
   */
  void goal_response_callback(
      std::shared_future<GoalHandleNav2Pose::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  /**
   * @brief Callback function called when the action is being executed, providing feedback (not used, but required)
   * @param feedback 
   */
  void feedback_callback(GoalHandleNav2Pose::SharedPtr, const std::shared_ptr<const ActionNav2Pose::Feedback> feedback) {}

  /**
   * @brief Callback function called when the action is completed
   * 
   * @param result Information codes about the result
   */
  void result_callback(const GoalHandleNav2Pose::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      return;
    }
  }
}; // class Nav2Pose

/**
 * @brief Starts the node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2Pose>();

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}