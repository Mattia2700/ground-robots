#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "waypoint_msgs/srv/current_pose.hpp"

#include <memory>

namespace current_pose_server {
class CurrentPose : public rclcpp::Node {
public:
  explicit CurrentPose() : Node("current_pose") {
    this->tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "tf", 10, [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reading tf topic");
          if (msg->transforms[0].header.frame_id == "map") {
            this->last_pose_->position.x = msg->transforms[0].transform.translation.x;
            this->last_pose_->position.y = msg->transforms[0].transform.translation.y;
            this->last_pose_->position.z = msg->transforms[0].transform.translation.z;
            this->last_pose_->orientation.x = msg->transforms[0].transform.rotation.x;
            this->last_pose_->orientation.y = msg->transforms[0].transform.rotation.y;
            this->last_pose_->orientation.z = msg->transforms[0].transform.rotation.z;
            this->last_pose_->orientation.w = msg->transforms[0].transform.rotation.w;
          }
        });
    this->service_ = this->create_service<waypoint_msgs::srv::CurrentPose>(
        "get_current_pose",
        [this](const std::shared_ptr<waypoint_msgs::srv::CurrentPose::Request> request, std::shared_ptr<waypoint_msgs::srv::CurrentPose::Response> response) {
          RCLCPP_INFO(this->get_logger(), "Received request");
          response->current_pose = *this->last_pose_;
          RCLCPP_INFO(this->get_logger(), "Sending response with position (%f,%f,%f) and orientation (%f,%f,%f,%f)",
                      response->current_pose.position.x, response->current_pose.position.y, response->current_pose.position.z,
                      response->current_pose.orientation.x, response->current_pose.orientation.y, response->current_pose.orientation.z, response->current_pose.orientation.w);
        });
  }

private:
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Service<waypoint_msgs::srv::CurrentPose>::SharedPtr service_;
  std::shared_ptr<geometry_msgs::msg::Pose> last_pose_ =
      std::make_shared<geometry_msgs::msg::Pose>();
};
} // namespace current_pose_server

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to return pose.");

  rclcpp::spin(std::make_shared<current_pose_server::CurrentPose>());
  rclcpp::shutdown();
  return 0;
}