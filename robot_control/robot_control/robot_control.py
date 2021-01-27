import rclpy
from rclpy.node import Node
import math
import numpy as np

#Import des messages
from std_msgs.msg import String
# from std_msgs.msg import String
# from std_msgs.msg import String

def test_choppe_balle(pos_robot,pos_balle):
    x_diff=pos_balle[0]-pos_robot[0]
    y_diff=pos_balle[1]-pos_robot[1]
    dist=np.linalg.norm([x_diff, y_diff])
    angle_target=math.atan2(y_diff,x_diff)
    angle_diff=pos_robot[2]-angle_target
    #Si le robot est près de la balle (distance inférieur à d)
    if (angle_diff < a and dist < b):
        return True
    else:
        return False

def get_dist_angle(pos_robot,pos_target):
    x_diff=pos_target[0]-pos_robot[0]
    y_diff=pos_target[1]-pos_robot[1]
    dist=np.linalg.norm([x_diff, y_diff])
    angle_target=math.atan2(y_diff,x_diff)
    angle_diff=pos_robot[2]-angle_target
    return(dist,angle_diff)

class Robot_Control(Node):

    def __init__(self):
        super().__init__('robot_control')

        #Création des variables internes
        self.pos_robot = [0,0,0]
        self.pos_balle = [0,0]
        self.robot_detec=False
        self.balle_detec=False
        self.has_ball=False

        #Création du suscriber de la position robot
        self.subscription = self.create_subscription(
            String,
            'robot_position',
            self.robot_position_callback,
            10)

        #Création du suscriber des positions des balles
        self.subscription = self.create_subscription(
            String,
            'ball_position',
            self.ball_position_callback,
            10)

        self.subscription = self.create_subscription(
            String,
            'ball_inside',
            self.ball_inside_callback,
            10)

        #Création du publisher de commande des roues
        self.publisher_ = self.create_publisher(
            String,
            'robot_command',
            10)

        #Création de la boucle de contrôle
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.control_callback)

    def robot_position_callback(self, msg):
        # self.pos_robot=[msg.data,msg.data,msg.data]
        self.robot_detec=True

    def ball_position_callback(self, msg):
        # self.pos_balle=[msg.data,msg.data,msg.data]
        self.balle_detec=True

    def ball_position_callback(self, msg):
        # self.has_ball=msg.data

    def control_callback(self):
        if not self.has_ball: #Si le robot n'a pas de balle
            if (self.robot_detec and self.balle_detec):
                # msg = String()
                # msg.data = 'Hello World: %d' % self.i
                # self.publisher_.publish(msg)
            else:
        else:

def main(args=None):
    rclpy.init(args=args)

    robot_control = Robot_Control()

    rclpy.spin(robot_control)

    robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()