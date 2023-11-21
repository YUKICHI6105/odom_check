#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>
// #include <odom_check/msg/odom.hpp>
#include <can_utils.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <cstdint>
#include <cmath>

#include <chrono>

using namespace std::chrono_literals;

template<class T>
struct Odom{
  T vx;
  T vy;
  T vz;
  T ax;
  T ay;
  T az;
  T yaw;
  T pitch;
  T roll;
};

class OdomCheckNode : public rclcpp::Node
{
  private:
    rclcpp::Subscription<can_plugins2::msg::Frame>::SharedPtr sub_;
    // rclcpp::Publisher<odom_check::msg::Odom>::SharedPtr pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::TimerBase::SharedPtr timer_;
    Odom<double> odom_msgD_;
    Odom<uint16_t> odom_msg16_;
    // can_utils::CanFrameParser parser_;
    // void can_callback(const can_plugins2::msg::Frame::SharedPtr msg);
    
    void can_callback(const can_plugins2::msg::Frame msg)
    {
      if(msg.id == 0x555){
        std::memcpy(&odom_msg16_.roll, msg.data.data() + 0, sizeof(uint16_t));
        std::memcpy(&odom_msg16_.pitch, msg.data.data() + 2, sizeof(uint16_t));
        std::memcpy(&odom_msg16_.yaw, msg.data.data() + 4, sizeof(uint16_t));
        std::memcpy(&odom_msg16_.vx, msg.data.data() + 6, sizeof(uint16_t));
      }else if(msg.id == 0x556){
        std::memcpy(&odom_msg16_.ax, msg.data.data() + 0, sizeof(uint16_t));
        std::memcpy(&odom_msg16_.ay, msg.data.data() + 2, sizeof(uint16_t));
        std::memcpy(&odom_msg16_.az, msg.data.data() + 4, sizeof(uint16_t));
        std::memcpy(&odom_msg16_.vy, msg.data.data() + 6, sizeof(uint16_t));
      // }else if(msg.id == 0x557){
      //   std::memcpy(&odom_msg.x, msg.data.data() + 0, sizeof(float));
      //   std::memcpy(&odom_msg.y, msg.data.data() + 4, sizeof(float));
      // //parser_.parse(msg, odom_msg);
      }
      odom_msgD_.roll = odom_msg16_.roll * 6.28 / 360;
      odom_msgD_.pitch = odom_msg16_.pitch * 6.28 / 360;
      odom_msgD_.yaw = odom_msg16_.yaw * 6.28 / 360;
      odom_msgD_.vx = odom_msg16_.vx * 45 / 1000 * 3.14 / 2048;
      odom_msgD_.vy = odom_msg16_.vy * 45 / 1000 * 3.14 / 2048;
      // odom_msgD_.vz = odom_msg16_.vz * 45 / 1000 * 3.14 / 2048;
      odom_msgD_.ax = odom_msg16_.ax * 1.0;
      odom_msgD_.ay = odom_msg16_.ay * 1.0;
      odom_msgD_.az = odom_msg16_.az * 1.0;
      // imu.angular_velocity.x
    }
  public:
    OdomCheckNode() : Node("odom_check_node")
    {
      sub_ = this->create_subscription<can_plugins2::msg::Frame>(
        "can_rx", 10, std::bind(&OdomCheckNode::can_callback, this, std::placeholders::_1));
      // pub_ = this->create_publisher<odom_check::msg::Odom>("odom_check", 10);
      pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
      pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    }
    void timer_callback();
};

void OdomCheckNode::timer_callback(){
  // auto odom_msg = odom_check::msg::Odom();
  auto odometry = nav_msgs::msg::Odometry();
  auto imu = sensor_msgs::msg::Imu();

  // 速度
  odometry.twist.twist.linear.x = odom_msgD_.vx;
  odometry.twist.twist.linear.y = odom_msgD_.vy;
  // odometry.twist.twist.linear.z = odom_msgD_.vz;

  // 角速度
  imu.angular_velocity.x = odom_msgD_.roll;
  imu.angular_velocity.y = odom_msgD_.pitch;
  imu.angular_velocity.z = odom_msgD_.yaw;

  // 加速度
  imu.linear_acceleration.x = odom_msgD_.ax;
  imu.linear_acceleration.y = odom_msgD_.ay;
  imu.linear_acceleration.z = odom_msgD_.az;
  
  // pub_->publish(odom_msg);
  pub_odom_->publish(odometry);
  pub_imu_->publish(imu);
}


int main(int argc, char *argv[])
{
  //printf("hello world odom_check package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomCheckNode>());
  rclcpp::shutdown();
  return 0;
}
