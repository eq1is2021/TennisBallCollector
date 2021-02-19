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


rclcpp::Time last_msg_cmd;
rclcpp::Time last_msg_cmd_catcher;

geometry_msgs::msg::Twist msg_mvt_catcher;
geometry_msgs::msg::Twist msg_mvt_vide;
geometry_msgs::msg::Twist msg_mvt;

using std::placeholders::_1;

//Add secu

double current_yaw = 0.;
double objective_yaw = 0.;

double sawtooth(double x)
{
	return 2.*atan(tan(x/2.));
}

class Ctrl : public rclcpp::Node
{
  public:
    Ctrl()
    : Node("node_yaw_ctrl")
    {
    	publisher_mvt = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    	
      	subscription_ctrl = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_yaw", 10, std::bind(&Ctrl::callback_cmd, this, _1));
      	subscription_ctrl_cage = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_catcher", 10, std::bind(&Ctrl::callback_cmd_catcher, this, _1));
    	subscription_yaw = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data",  rclcpp::SensorDataQoS(), std::bind(&Ctrl::callback_yaw, this, _1));
    	timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Ctrl::timer_callback, this));
    }

  private:

    void timer_callback()
    {
      if((rclcpp::Clock().now() - last_msg_cmd_catcher).seconds() < 0.2)
      {
      	//std::cout <<"Mode catcher"<< std::endl;
		publisher_mvt->publish(msg_mvt_catcher);
      }
      else if((rclcpp::Clock().now() - last_msg_cmd).seconds() < 0.2)
      {
      	//std::cout <<"Mode yaw"<< std::endl;
		publisher_mvt->publish(msg_mvt);
      }
      else
      {
      	//std::cout <<"No cmd"<< std::endl;
      	msg_mvt_vide.linear.x = 0.;
		msg_mvt_vide.angular.z = 0.75*atan(sawtooth(objective_yaw- current_yaw));
      	publisher_mvt->publish(msg_mvt_vide);
      }
    }

    void callback_yaw(const sensor_msgs::msg::Imu::SharedPtr msg) const
    {
    	tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    	tf2::Matrix3x3 m(q);
    	double roll, pitch, yaw;
    	m.getRPY(roll, pitch, yaw);

    	current_yaw = sawtooth(yaw - M_PI/2.);
    
    }

    void callback_cmd(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
    	objective_yaw = msg->angular.z;
    	msg_mvt.linear.x = std::max(-0.5,std::min(0.5,msg->linear.x));
    	msg_mvt.angular.z = 0.75*atan(sawtooth(objective_yaw- current_yaw));
    	//std::cout << current_yaw*180./M_PI <<" "<<sawtooth(msg->angular.z - current_yaw)*180./M_PI << std::endl;
 		last_msg_cmd = rclcpp::Clock().now();
    }

    void callback_cmd_catcher(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
    	msg_mvt.linear.x = std::max(-0.5,std::min(0.5,msg->linear.x));
    	msg_mvt_catcher.angular.z = msg->angular.z;
		last_msg_cmd_catcher = rclcpp::Clock().now();
    }


    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_ctrl;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_ctrl_cage;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_yaw;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_mvt;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ctrl>());
  rclcpp::shutdown();
  return 0;
}
