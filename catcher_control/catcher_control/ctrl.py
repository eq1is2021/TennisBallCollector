#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Wrench

class WrenchSubPub(Node):

    def __init__(self):
        super().__init__('wrench_pub')
        self.isUp = True
        self.publisher_ = self.create_publisher(Wrench, '/catcher_force', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(Bool,'/catcher_up', self.listener_callback,10)

    def listener_callback(self, msg):
        self.isUp = msg.data

    def timer_callback(self):
        msg = Wrench()
        msg.force.x = 0.0
        msg.force.y = 0.0
        msg.force.z = (2.0 * (float)(self.isUp == True)) + (-1.0 * (float)(self.isUp == False))
        msg.torque.x = 0.0
        msg.torque.y = 0.0
        msg.torque.z = 0.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node_ = WrenchSubPub()
    rclpy.spin(node_)

    node_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()