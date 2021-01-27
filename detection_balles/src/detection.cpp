#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/int32.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "image_raw", 10, std::bind(&MinimalSubscriber::image_callback, this, _1));
    }

  private:
    void image_callback(const std_msgs::msg::Int32::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%i'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
