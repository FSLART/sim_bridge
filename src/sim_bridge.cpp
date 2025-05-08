#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <eufs_msgs/msg/wheel_speeds_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <lart_msgs/msg/dynamics_cmd.hpp>
#include <lart_msgs/msg/dynamics.hpp>
#include <lart_msgs/msg/cone_array.hpp>
#include "lart_common.h"

#define BLUE_CONE 0
#define YELLOW_CONE 1
#define ORANGE_CONE 2
#define BIG_ORANGE_CONE 3
#define UNKNOWN_CONE 4

using namespace std::chrono_literals;

class SimBridge : public rclcpp::Node
{
public:
  SimBridge() : Node("sim_bridge")
  {
    cone_array_sub_ = this->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
      "/ground_truth/cones", 10,
      std::bind(&SimBridge::cone_array_callback, this, std::placeholders::_1));

    cone_array_pub = this->create_publisher<lart_msgs::msg::ConeArray>("/mapping/cones", 10);

    dynamics_sub_ = this->create_subscription<lart_msgs::msg::DynamicsCMD>(
      "/pc_origin/dynamics", 10,
      std::bind(&SimBridge::dynamics_callback, this, std::placeholders::_1));

    ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);

    sensor_speed_sub = this->create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(
      "/ground_truth/wheel_speeds", 10,
      std::bind(&SimBridge::sensor_speed_callback, this, std::placeholders::_1));

    dynamics_pub_ = this->create_publisher<lart_msgs::msg::Dynamics>("/acu_origin/dynamics", 10);
  }

private:
  void cone_array_callback(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg)
  {
    lart_msgs::msg::ConeArray combined_cones;
    combined_cones.header = msg->header;

    auto convert_cones = [&](const std::vector<eufs_msgs::msg::ConeWithCovariance>& source, int class_type) {
      for (const auto& cone : source) {
        lart_msgs::msg::Cone converted;
        converted.header = msg->header;
        converted.position = cone.point;
        converted.class_type.data = class_type;
        combined_cones.cones.push_back(converted);
      }
    };

    convert_cones(msg->blue_cones, BLUE_CONE);
    convert_cones(msg->yellow_cones, YELLOW_CONE);
    convert_cones(msg->orange_cones, ORANGE_CONE);
    convert_cones(msg->big_orange_cones, BIG_ORANGE_CONE);
    convert_cones(msg->unknown_color_cones, UNKNOWN_CONE);

    RCLCPP_INFO(this->get_logger(), "Total cones converted: %lu", combined_cones.cones.size());

    cone_array_pub->publish(combined_cones);
  }

  void dynamics_callback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg)
  {
    ackermann_msgs::msg::AckermannDriveStamped ack_msg;
    ack_msg.header.stamp = this->now();
    ack_msg.header.frame_id = "base_link"; 
    ack_msg.drive.steering_angle = msg->steering_angle;
    ack_msg.drive.speed = RPM_TO_MS(msg->rpm);

    ackermann_pub_->publish(ack_msg);

    RCLCPP_INFO(this->get_logger(), "DynamicsCMD → Ackermann: angle=%.2f, rpm=%d → speed=%.2f m/s",
                ack_msg.drive.steering_angle, msg->rpm, ack_msg.drive.speed);
  }

  void sensor_speed_callback(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg)
{
  lart_msgs::msg::Dynamics dynamics_msg;

  // dynamics_msg.wheel_speed_lf = msg->speeds.lf_speed;
  // dynamics_msg.wheel_speed_rf = msg->speeds.rf_speed;
  // dynamics_msg.wheel_speed_lr = msg->speeds.lb_speed;
  // dynamics_msg.wheel_speed_rr = msg->speeds.rb_speed;

  float speed = ((msg->speeds.lb_speed + msg->speeds.rb_speed)/2) / 37.8188;
  dynamics_msg.rpm = MS_TO_RPM(speed);

  dynamics_pub_->publish(dynamics_msg);

  RCLCPP_INFO(this->get_logger(), "Sensor rpm: %d",dynamics_msg.rpm);

  // RCLCPP_INFO(this->get_logger(), "Published Dynamics: steer=%.2f, lf=%.2f, rf=%.2f, lr=%.2f, rr=%.2f",
  //             dynamics_msg.steering_angle,
  //             dynamics_msg.wheel_speed_lf,
  //             dynamics_msg.wheel_speed_rf,
  //             dynamics_msg.wheel_speed_lr,
  //             dynamics_msg.wheel_speed_rr);
}

  float rpm_to_mps(uint16_t rpm)
  {
    return static_cast<float>(rpm) / 37.8188f;
  }

  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cone_array_sub_;
  rclcpp::Publisher<lart_msgs::msg::ConeArray>::SharedPtr cone_array_pub;
  rclcpp::Subscription<lart_msgs::msg::DynamicsCMD>::SharedPtr dynamics_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
  rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr sensor_speed_sub;
  rclcpp::Publisher<lart_msgs::msg::Dynamics>::SharedPtr dynamics_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimBridge>());
  rclcpp::shutdown();
  return 0;
}
