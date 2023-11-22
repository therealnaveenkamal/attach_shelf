#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer() : Node("approach_service_server") {

    auto clock = this->get_clock();
    tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    approach_service_ = this->create_service<attach_shelf::srv::GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachServiceServer::handle_approach_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ApproachServiceServer::scan_callback, this,
                  std::placeholders::_1));

    msgvalue.resize(NUM_LASER_READINGS,
                    0.0); // Adjust the size based on your laser configuration
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&ApproachServiceServer::timer_callback, this));

    elevator_publisher =
        this->create_publisher<std_msgs::msg::Empty>("/elevator_up", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  std::vector<double> msgvalue;
  static const size_t NUM_LASER_READINGS = 1080;
  std::vector<int> globalLegPos;
  double angleLegs = 0.0;
  double xDistance = 0.0;
  double yDistance = 0.0;
  bool ready = false;
  bool move_extra = false;
  bool elevated = false;
  int extratime = 8;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  tf2::Quaternion cart_quat_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TransformStamped transform_stamped;

  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr approach_service_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_publisher;

  bool handle_approach_request(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {

    RCLCPP_INFO(this->get_logger(), "------%d", globalLegPos.size());
    if (globalLegPos.size() != 2) {
      response->complete = false;
    }

    if (request->attach_to_shelf) {
      RCLCPP_INFO(this->get_logger(), "------%d", msgvalue.size());

      RCLCPP_INFO(get_logger(), "Printing leg positions:");
      for (const auto &value : globalLegPos) {
        RCLCPP_INFO(get_logger(), "Value: %d", value);
      }

      RCLCPP_INFO(this->get_logger(), "------%d", msgvalue.size());
      ready = true;
      response->complete = true;

    } else {
      response->complete = false;
    }

    return true;
  }

  void timer_callback() {
    if (move_extra && !elevated) {
      if (extratime >= 0) {
        RCLCPP_INFO(this->get_logger(), "Move 30 cm INIT: %d", extratime);
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_msg.linear.x = 0.1;
        cmd_vel_publisher_->publish(cmd_vel_msg);
        extratime--;
      } else {
        RCLCPP_INFO(this->get_logger(), "Elevation Start");
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_publisher_->publish(cmd_vel_msg);

        std_msgs::msg::Empty msg;
        elevator_publisher->publish(msg);
        elevated = true;
      }
    }
  }

   void publish_cart_frame_transform() {

        geometry_msgs::msg::TransformStamped transformStamped;
      geometry_msgs::msg::Twist cmd_vel_msg;

      try {
        transformStamped = tf_buffer->lookupTransform(
            "robot_odom", "robot_front_laser_base_link", tf2::TimePointZero);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s",
                    "robot_odom", "robot_front_laser_base_link");
        return; // Return early in case of a transform exception
      }

      // Recalculate the transformed coordinates inside the loop
      tf2::Vector3 transformed_coords(
          transformStamped.transform.translation.x + xDistance,
          transformStamped.transform.translation.y + yDistance,
          transformStamped.transform.translation.z);

      double distance_to_target = transformed_coords.length();

      // Broadcast the transform for "cart_frame"
      transform_stamped.header.stamp = transformStamped.header.stamp;
      transform_stamped.header.frame_id = "robot_front_laser_base_link";
      transform_stamped.child_frame_id = "cart_frame";
      transform_stamped.transform.translation.x = xDistance;
      transform_stamped.transform.translation.y = yDistance;
      transform_stamped.transform.translation.z = 0.0;
      transform_stamped.transform.rotation.x = cart_quat_.x();
      transform_stamped.transform.rotation.y = cart_quat_.y();
      transform_stamped.transform.rotation.z = cart_quat_.z();
      transform_stamped.transform.rotation.w = cart_quat_.w();
      broadcaster->sendTransform(transform_stamped);

    if (ready) {

      tf2::Vector3 direction_to_target = transformed_coords.normalized();
      float tfDistance = sqrt(pow(xDistance, 2) +
                              pow(yDistance, 2));
      RCLCPP_INFO(this->get_logger(), "DIST: %f", tfDistance);

      if (tfDistance >= 0.4) {
        cmd_vel_msg.linear.x = 0.1;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(cmd_vel_msg);
      } else {
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(cmd_vel_msg);

        ready = false;
        move_extra = true;
      }

    }
  }


  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    for (size_t i = 0; i < msg->intensities.size(); ++i) {
      msgvalue[i] = msg->ranges[i];
    }

    double threshold = 7000;
    int j = 0;
    std::vector<int> legPositions;

    float tempIntensity = 0.0;

    // RCLCPP_INFO(this->get_logger(), "------%d", msg->intensities.size());

    for (int i = 1; i < msg->intensities.size() - 1; ++i) {

      if (msg->intensities[i] > threshold &&
          msg->intensities[i + 1] > threshold) {
        tempIntensity = 0.0;
        if (i + 10 < msg->intensities.size()) {
          for (j = i; j <= i + 10; ++j) {
            tempIntensity += msg->intensities[j];
          }
          if (tempIntensity >= 60000) {
            legPositions.push_back(i + 5);
            // RCLCPP_INFO(this->get_logger(), "POS: %d, %f", i +
            // 5,tempIntensity);
            i = j;
          }
        }
      }
    }

    // RCLCPP_INFO(this->get_logger(), "------");

    globalLegPos = legPositions;

    if (legPositions.size() == 2) {
      double distance =
          (msg->ranges[legPositions[0]] + msg->ranges[legPositions[1]]) / 2.0;
      angleLegs =
          (msg->angle_min +
           ((legPositions[0] + legPositions[1]) / 2.0) * msg->angle_increment);

      xDistance = distance * cos(angleLegs);
      yDistance = distance * sin(angleLegs);

      cart_quat_.setRPY(0.0, 0.0, angleLegs);

      RCLCPP_INFO(this->get_logger(), "%d, %d, (%f, %f)", legPositions[0],
                  legPositions[1], xDistance, yDistance);

      publish_cart_frame_transform();
    } else {
      RCLCPP_INFO(this->get_logger(), "TWO LEGS NOT FOUND");
      publish_cart_frame_transform();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}
