#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <signal.h>

class PreApproachNode : public rclcpp::Node {
public:
  PreApproachNode() : Node("pre_approach_node") {

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Sets the obstacle";
    this->declare_parameter<std::double_t>("obstacle", 0.0, param_desc);

    auto param_desc1 = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc1.description = "Sets the degrees";
    this->declare_parameter<int>("degrees", 0, param_desc1);

    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PreApproachNode::scan_callback, this,
                  std::placeholders::_1));
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PreApproachNode::odom_callback, this,
                  std::placeholders::_1));

    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    this->get_parameter("obstacle", obs);
    this->get_parameter("degrees", deg);

    RCLCPP_INFO(this->get_logger(), "Target: %f; Curr: %f", obs,
                msg->ranges[540]);

    geometry_msgs::msg::Twist cmd_vel_msg;

    if (is_rotating_ == false) {
      if (std::abs(msg->ranges[540] - obs) < 0.05) {
        cmd_vel_msg.linear.x = 0.0;
        is_rotating_ = true;
      } else {
        cmd_vel_msg.linear.x = 0.5;
        cmd_vel_msg.angular.z = 0.0;
      }
    }

    target_yaw_ = deg * M_PI / 180.0;

    if (is_rotating_ == true) {

      RCLCPP_INFO(this->get_logger(), "Curr: %f; Targ: %f; Err: %f",
                  current_yaw_, target_yaw_,
                  std::abs(current_yaw_ - target_yaw_));

      initial_yaw_ = current_yaw_;

      if (std::abs(current_yaw_ - target_yaw_) > 0.05) {
        cmd_vel_msg.angular.z = -0.2;
        cmd_vel_msg.linear.x = 0.0;
        is_rotating_ = true;
      } else {
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_publisher_->publish(cmd_vel_msg);
        rclcpp::shutdown();
      }
    }
    cmd_vel_publisher_->publish(cmd_vel_msg);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  std::double_t obs;
  int deg;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  bool is_rotating_ = false;
  double initial_yaw_;
  double target_yaw_;
  double current_yaw_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproachNode>());
  rclcpp::shutdown();
  return 0;
}