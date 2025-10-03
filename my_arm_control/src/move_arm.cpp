#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber"),
    publisher(TOPIC)   // initialize member
  {
    // Init DDS
    ChannelFactory::Instance()->Init(0);
    publisher.InitChannel();

    // ROS subscriber
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::JointState& msg)
  {
    auto position = msg.position;
    unitree_arm::msg::dds_::ArmString_ msg_arm{};

    float angle0 = position[2] * (180.0f / 3.14159265f);
    float angle1 = position[0] * (180.0f / 3.14159265f);
    float angle2 = position[1] * (180.0f / 3.14159265f);
    float angle3 = position[3] * (180.0f / 3.14159265f);
    float angle4 = position[4] * (180.0f / 3.14159265f);
    float angle5 = position[5] * (180.0f / 3.14159265f);
    float angle6 = 0.0f;

    RCLCPP_INFO(this->get_logger(), "Angle (%f)", angle0);

    // msg_arm.data_() =
    //   "{\"seq\":4,"
    //   "\"address\":1,"
    //   "\"funcode\":1,"
    //   "\"data\":{"
    //     "\"id\":" + std::to_string(id) +
    //     ",\"angle\":" + std::to_string(angle0) +
    //     ",\"delay_ms\":0}"
    //   "}";

    msg_arm.data_() = "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,\"angle0\":"+std::to_string(angle0)+",\"angle1\":"+std::to_string(angle1)+",\"angle2\":"+std::to_string(angle2)+",\"angle3\":"+std::to_string(angle3)+",\"angle4\":"+std::to_string(angle4)+",\"angle5\":"+std::to_string(angle5)+",\"angle6\":"+std::to_string(angle6)+"}}";

    // msg_arm.data_() =
    // "{\"seq\":4,"
    // "\"address\":1,"
    // "\"funcode\":2,"
    // "\"data\":{"
    //   "\"mode\":1,"
    //   "\"angle0\":" + std::to_string(10) + ","
    //   "\"angle1\":" + std::to_string(10) + ","
    //   "\"angle2\":" + std::to_string(10) + ","
    //   "\"angle3\":" + std::to_string(10) + ","
    //   "\"angle4\":" + std::to_string(10) + ","
    //   "\"angle5\":" + std::to_string(10) +
    // "}}";
    publisher.Write(msg_arm);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

  // members for Unitree SDK
  ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
