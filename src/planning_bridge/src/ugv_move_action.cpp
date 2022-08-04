#include <functional>
#include <future>
#include <memory>
#include <algorithm>
#include <memory>
#include <fstream>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/executor.hpp"
// #include "rclcpp/rclcpp_lifecycle.hpp"

#include "geometry_msgs/msg/pose.hpp"
// #include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "waypoint_msgs/srv/start_navigation.hpp"

// utils
#include "json.hpp"

#define GOAL_INDEX 2
#define JSON_NAME "waypoints_dict.json"

using json = nlohmann::json;
using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient {
public:
  MoveAction() : plansys2::ActionExecutorClient("ugv_move", 250ms) {
    progress_ = 0.0;

    waypoints_file_path_ = current_file_path_.substr(0,
    current_file_path_.find_last_of("/")); waypoints_file_path_ =
    waypoints_file_path_.substr(0,
    waypoints_file_path_.find_last_of("/")).append("/config/" JSON_NAME);
    waypoints_file_ = std::ifstream(waypoints_file_path_);

    // check if waypoints_file_ exists
    std::cout << "Reading waypoints from " << waypoints_file_path_ <<
    std::endl; if(!waypoints_file_.is_open()) {
      RCLCPP_ERROR(get_logger(), "waypoints_dict.json not found");
      return;
    } else {
      labels_ = json::parse(waypoints_file_);
      waypoints_file_.close();
    }

    // tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    //     "tf", 10, [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    //       // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reading tf topic");
    //       if (msg->transforms[0].header.frame_id == "map") {
    //         this->last_pose_->position.x =
    //             msg->transforms[0].transform.translation.x;
    //         this->last_pose_->position.y =
    //             msg->transforms[0].transform.translation.y;
    //         this->last_pose_->position.z =
    //             msg->transforms[0].transform.translation.z;
    //         this->last_pose_->orientation.x =
    //             msg->transforms[0].transform.rotation.x;
    //         this->last_pose_->orientation.y =
    //             msg->transforms[0].transform.rotation.y;
    //         this->last_pose_->orientation.z =
    //             msg->transforms[0].transform.rotation.z;
    //         this->last_pose_->orientation.w =
    //             msg->transforms[0].transform.rotation.w;
    //       }
    //     });
  }

private:
  void do_work() {

    room_goal_ = current_arguments_[GOAL_INDEX];
    goal_position_ = labels_.at(room_goal_).at("center").get<std::vector<double>>();

    if(!is_initial_distance_set_){
      auto request = std::make_shared<waypoint_msgs::srv::StartNavigation::Request>();
      request->x = goal_position_[0];
      request->y = goal_position_[1];
      request->z = goal_position_[2];
      auto future_pose = this->start_navigation_client_->async_send_request(
        request,
        [this](rclcpp::Client<waypoint_msgs::srv::StartNavigation>::SharedFuture future) {
          auto result = future.get();
          initial_distance_ = result->initial_distance;
          this->is_initial_distance_set_ = true;
          RCLCPP_INFO(this->get_logger(), "Setting initial distnace to %f", initial_distance_);
          continue_work();
        });
    }
  }

  void continue_work(){
    current_distance_ = std::sqrt(std::pow(goal_position_[0] -
    last_pose_->position.x, 2) + std::pow(goal_position_[1] -
    last_pose_->position.y, 2)); progress_ = 1.0 - (current_distance_ /
    initial_distance_);

    if (progress_ < 0.99) {
      send_feedback(progress_, "Move running");
    } else {
      finish(true, 1.0, "Move completed");

      progress_ = 0.0;
      is_initial_distance_set_ = false;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Moving ... [" << std::min(100.0, progress_ * 100.0) << "%]" << std::flush;
  }

  float progress_;
  geometry_msgs::msg::Pose::SharedPtr last_pose_ =
  std::make_shared<geometry_msgs::msg::Pose>();
  // rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  json labels_;
  std::string room_goal_;
  std::vector<double> goal_position_;
  double initial_distance_;
  bool is_initial_distance_set_ = false;
  double current_distance_;
  std::string current_file_path_ = __FILE__;
  std::string waypoints_file_path_;
  std::ifstream waypoints_file_;
  rclcpp::Client<waypoint_msgs::srv::StartNavigation>::SharedPtr start_navigation_client_ = this->create_client<waypoint_msgs::srv::StartNavigation>("start_navigation");
};

namespace nav_2_pose_cpp {
class Nav2Pose : public rclcpp::Node {
public:
  using ActionNav2Pose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav2Pose = rclcpp_action::ClientGoalHandle<ActionNav2Pose>;
  using PoseInt = long unsigned int;

  explicit Nav2Pose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("nav_2_pose_client", options) {
    this->action_nav_pose_ptr_ =
        rclcpp_action::create_client<ActionNav2Pose>(this, "navigate_to_pose");

    this->start_navigation_service_ptr_ = this->create_service<
        waypoint_msgs::srv::StartNavigation>(
        "start_navigation",
        [this](
            const std::shared_ptr<waypoint_msgs::srv::StartNavigation::Request>
                request,
            std::shared_ptr<waypoint_msgs::srv::StartNavigation::Response>
                response) {
          RCLCPP_INFO(this->get_logger(), "Received request");
          goal_->pose.position.x = request->x;
          goal_->pose.position.y = request->y;
          goal_->pose.position.z = request->z;
          goal_->pose.orientation.x = 0.0;
          goal_->pose.orientation.y = 0.0;
          goal_->pose.orientation.z = 0.0;
          goal_->pose.orientation.w = 1.0;
          goal_->header.frame_id = "map";
          goal_->header.stamp = this->now();
          response->initial_distance = 0;
          this->send_goal();
        });
  }

  void send_goal() {
    using namespace std::placeholders;

    if (!action_nav_pose_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal");

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
    this->action_nav_pose_ptr_->async_send_goal(*goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ActionNav2Pose>::SharedPtr action_nav_pose_ptr_;
  rclcpp::Service<waypoint_msgs::srv::StartNavigation>::SharedPtr
      start_navigation_service_ptr_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> goal_ =
      std::make_shared<geometry_msgs::msg::PoseStamped>();

  void goal_response_callback(
      std::shared_future<GoalHandleNav2Pose::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleNav2Pose::SharedPtr,
      const std::shared_ptr<const ActionNav2Pose::Feedback> feedback) {
    // if first time, set initial_distance_
    // else return current_distance_
    RCLCPP_INFO(this->get_logger(), "%f", feedback->distance_remaining);
  }

  void result_callback(const GoalHandleNav2Pose::WrappedResult &result) {
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
  }
}; // class Nav2Pose

} // namespace nav_2_pose_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto move = std::make_shared<MoveAction>();
  rclcpp::executors::MultiThreadedExecutor executor;

  auto navigate = std::make_shared<nav_2_pose_cpp::Nav2Pose>();

  executor.add_node(move->get_node_base_interface());
  executor.add_node(navigate->get_node_base_interface());

  move->set_parameter(rclcpp::Parameter("action_name", "ugv_move"));
  move->trigger_transition(
  lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  // print move currrent state
  RCLCPP_INFO(move->get_logger(), "Current state: %s",
              move->get_current_state().label().c_str());

  executor.spin();

  // rclcpp::spin(move->get_node_base_interface());
  // rclcpp::spin(navigate->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}