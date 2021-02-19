#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"


using std::placeholders::_1;

class FakeImu: public rclcpp::Node
{
  public:
    FakeImu(): Node("node_fake_imu")
    {
    	publisher_imu = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    	
      subscription_truth = this->create_subscription<nav_msgs::msg::Odometry>("/truth", rclcpp::SensorDataQoS(), std::bind(&FakeImu::callback_truth, this, _1));

    }

  private:

    void callback_truth(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
    	sensor_msgs::msg::Imu msg_imu;
      msg_imu.header = msg->header;
      msg_imu.orientation = msg->pose.pose.orientation;

      publisher_imu->publish(msg_imu);
    
    }


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_truth;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeImu>());
  rclcpp::shutdown();
  return 0;
}
