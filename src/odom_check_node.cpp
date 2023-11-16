#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <odom_check/msg/odom.hpp>
#include <can_utils.hpp>
#include <cstdint>
#include <cmath>

class OdomCheckNode : public rclcpp::Node
{
  private:
    rclcpp::Subscription<can_plugins2::msg::Frame>::SharedPtr sub_;
    rclcpp::Publisher<odom_check::msg::Odom>::SharedPtr pub_;
    // can_utils::CanFrameParser parser_;
    // void can_callback(const can_plugins2::msg::Frame::SharedPtr msg);
    
    void can_callback(const can_plugins2::msg::Frame msg)
    {
      auto odom_msg = odom_check::msg::Odom();
      if(msg.id == 0x555){
        std::memcpy(&odom_msg.roll, msg.data.data() + 0, sizeof(float));
        std::memcpy(&odom_msg.pitch, msg.data.data() + 4, sizeof(float));
      }else if(msg.id == 0x556){
        std::memcpy(&odom_msg.yaw, msg.data.data() + 0, sizeof(float));
      }else if(msg.id == 0x557){
        std::memcpy(&odom_msg.x, msg.data.data() + 0, sizeof(float));
        std::memcpy(&odom_msg.y, msg.data.data() + 4, sizeof(float));
      //parser_.parse(msg, odom_msg);
      }
      pub_->publish(odom_msg);
    }
  public:
    OdomCheckNode() : Node("odom_check_node")
    {
      sub_ = this->create_subscription<can_plugins2::msg::Frame>(
        "can_rx", 10, std::bind(&OdomCheckNode::can_callback, this, std::placeholders::_1));
      pub_ = this->create_publisher<odom_check::msg::Odom>("odom_check", 10);
    }
};


int main(int argc, char *argv[])
{
  //printf("hello world odom_check package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomCheckNode>());
  rclcpp::shutdown();
  return 0;
}
