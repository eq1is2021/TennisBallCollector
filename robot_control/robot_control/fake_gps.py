import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

from rclpy.qos import qos_profile_sensor_data


terrain_x=30
terrain_y=16


class GPS(Node):

    def __init__(self):
        
        super().__init__('gps_node')


        self.subscription = self.create_subscription(
            NavSatFix,
            'gps',
            self.gps_callback,
            qos_profile_sensor_data)


        self.publisher_ = self.create_publisher(
            Twist,
            'gnss_twist',
            10)


    def gps_callback(self, msg):
        lat = msg.latitude*math.pi/180.
        lon = msg.longitude*math.pi/180.
        print("ok")
        
        R = 6371009
        position = Twist()
                           
        position.linear.x, position.linear.y = R*lat*math.cos(lon) + 15., -R*lon + 8.
        
        position.linear.z = 0.
        position.angular.x = 0.
        position.angular.y = 0.
        position.angular.z = 0.
        
        self.publisher_.publish(position)
        
  
def main(args=None):
    rclpy.init(args=args)

    gps = GPS()

    rclpy.spin(gps)

    gps.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()