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

    msgvalue.resize(NUM_LASER_READINGS, 0.0);
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
  double xDistance_once = 0.0;
  double yDistance_once = 0.0;
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

  void handle_approach_request(
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

/*
void publish_cart_frame_transform() {
    geometry_msgs::msg::TransformStamped transformStamped;
    geometry_msgs::msg::Twist cmd_vel_msg;

      if (ready) {

      try {
        // Get the fixed transform from robot_odom to front_laser_link
        transformStamped = tf_buffer->lookupTransform(
            "robot_odom", "robot_base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s",
                    "robot_odom", "robot_front_laser_base_link");
        return; // Return early in case of a transform exception
    }

    // Calculate the fixed distance between robot_odom and front_laser_link
    tf2::Vector3 odom_to_laser_distance(
        transformStamped.transform.translation.x,
        transformStamped.transform.translation.y,
        transformStamped.transform.translation.z);

    // Recalculate the transformed coordinates inside the loop
    tf2::Vector3 base_coords(xDistance, yDistance, 0.0);

    tf2::Vector3 base_coords_var(yDistance, xDistance, 0.0);

    // Calculate the updated position of cart_frame with respect to robot_odom
    tf2::Vector3 cart_frame_coords = odom_to_laser_distance + base_coords;

    // Broadcast the transform for "cart_frame"
    transform_stamped.header.stamp = transformStamped.header.stamp;
    transform_stamped.header.frame_id = "robot_odom";
    transform_stamped.child_frame_id = "cart_frame";
    transform_stamped.transform.translation.x = xDistance;
    transform_stamped.transform.translation.y = yDistance;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = cart_quat_.x();
    transform_stamped.transform.rotation.y = cart_quat_.y();
    transform_stamped.transform.rotation.z = cart_quat_.z();
    transform_stamped.transform.rotation.w = cart_quat_.w();
    broadcaster->sendTransform(transform_stamped);
    
        // Calculate the distance between cart_frame and front_laser_link
        tf2::Vector3 front_laser_coords = odom_to_laser_distance + base_coords_var;

        double tfDistance = sqrt(pow(xDistance, 2) + pow(yDistance, 2));

        double distance_to_target = base_coords_var.length();

        RCLCPP_INFO(this->get_logger(), "Distance to front_laser_link: %f", tfDistance);
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

} */

void publish_cart_frame_transform() {
  geometry_msgs::msg::Twist cmd_vel_msg;

  try {
    auto odom_to_base_transform = tf_buffer->lookupTransform(
        "robot_odom", "robot_base_link", tf2::TimePointZero);
    auto base_to_laser_transform = tf_buffer->lookupTransform(
        "robot_base_link", "robot_front_laser_base_link", tf2::TimePointZero);
    tf2::Vector3 base_coords(xDistance, yDistance, 0.0);
    tf2::Quaternion laser_quat;
    tf2::fromMsg(base_to_laser_transform.transform.rotation, laser_quat);

    tf2::Vector3 laser_coords = tf2::quatRotate(laser_quat, base_coords);

    tf2::Quaternion odom_quat;
    tf2::fromMsg(odom_to_base_transform.transform.rotation, odom_quat);

    tf2::Vector3 odom_coords =
        tf2::quatRotate(odom_quat, laser_coords);
    odom_coords += tf2::Vector3(odom_to_base_transform.transform.translation.x,
                                odom_to_base_transform.transform.translation.y,
                                odom_to_base_transform.transform.translation.z);


    if (ready) {

    if(xDistance_once==0.0){
         xDistance_once = odom_coords.x();
    }

    if(yDistance_once==0.0){
         yDistance_once = odom_coords.y();
    }

    transform_stamped.header.stamp = odom_to_base_transform.header.stamp;
    transform_stamped.header.frame_id = "robot_odom";
    transform_stamped.child_frame_id = "cart_frame";
    transform_stamped.transform.translation.x = xDistance_once;
    transform_stamped.transform.translation.y = yDistance_once;
    transform_stamped.transform.translation.z = odom_coords.z();
    transform_stamped.transform.rotation.x = cart_quat_.x();
    transform_stamped.transform.rotation.y = cart_quat_.y();
    transform_stamped.transform.rotation.z = cart_quat_.z();
    transform_stamped.transform.rotation.w = cart_quat_.w();
    broadcaster->sendTransform(transform_stamped);

        double tfDistance = sqrt(pow(xDistance, 2) + pow(yDistance, 2));
        RCLCPP_INFO(this->get_logger(), "Dist: %f", tfDistance);

      if (tfDistance >= 0.4) {
        cmd_vel_msg.linear.x = 0.1;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(cmd_vel_msg);
      } else {
        xDistance_once = odom_coords.x();
        yDistance_once = odom_coords.y();
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(cmd_vel_msg);

        ready = false;
        move_extra = true;
      }
    }
    else if (ready == false && move_extra == true){

   

    }

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
    return;
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
            i = j;
          }
        }
      }
    }

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
