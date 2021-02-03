import rclpy
from rclpy.node import Node
import math
import numpy as np

#Message imports
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool

terrain_x=60
terrain_y=40
wall_center=20
wall_length=20
goal_zone_left_x=4
goal_zone_left_y=36
goal_zone_right_x=56
goal_zone_right_y=4
goal_left_x=0
goal_left_y=40
goal_right_x=0
goal_right_y=40

def get_dist_angle(pos_robot,pos_target):
    x_diff=pos_target[0]-pos_robot[0]
    y_diff=pos_target[1]-pos_robot[1]
    dist=np.linalg.norm([x_diff, y_diff])
    angle_target=math.atan2(y_diff,x_diff)
    angle_diff=pos_robot[2]-angle_target
    return(dist,angle_diff)

def get_passage_wall(pos_robot):
    len_space=terrain_y-wall_center-wall_length/2
    if pos_robot[1]<terrain_y/2:
        return(terrain_x/2,len_space/2)
    else:
        return(terrain_x/2,terrain_y-len_space/2)


def test_lr(pos):
    if pos < terrain_x/2:
        return "left"
    else:
        return "right"

def command_objective(pos_robot,pos_target):
    dist,angle_diff = get_dist_angle(pos_robot,pos_target)
    cmd = Twist()
    if abs(angle_diff) > np.radians(20):
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = -0.1*angle_diff
    else:
        cmd.linear.x = 0.5
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = -0.1*angle_diff
    return(cmd)

def test_goal_zone(pos_robot):
    x_diff_left=goal_zone_left_x[0]-pos_robot[0]
    y_diff_left=goal_zone_left_y[1]-pos_robot[1]
    dist_left=np.linalg.norm([x_diff_left, y_diff_left])
    x_diff_right=goal_zone_right_x[0]-pos_robot[0]
    y_diff_right=goal_zone_right_y[1]-pos_robot[1]
    dist_right=np.linalg.norm([x_diff_right, y_diff_right])
    if dist_left < 2 or dist_right < 2:
        return True
    else:
        return False

def test_ball_near(pos_robot,pos_balle,d):
    x_diff=pos_balle[0]-pos_robot[0]
    y_diff=pos_balle[1]-pos_robot[1]
    dist=np.linalg.norm([x_diff, y_diff])
    if dist < d && (test_lr(pos_robot[0])==test_lr(pos_balle[0])):
        return True
    else:
        return False

def test_choppe_balle(pos_robot,pos_balle):
    x_diff=pos_balle[0]-pos_robot[0]
    y_diff=pos_balle[1]-pos_robot[1]
    dist=np.linalg.norm([x_diff, y_diff])
    angle_target=math.atan2(y_diff,x_diff)
    angle_diff=pos_robot[2]-angle_target
    if (angle_diff < a and dist < b):
        return True
    else:
        return False

class Robot_Control(Node):

    def __init__(self):
        super().__init__('robot_control')

        #Internal variables
        self.pos_robot = [0,0,0]
        self.num_balle = 0
        self.pos_balle = np.zeros((10,3))
        self.robot_detec=False
        self.balle_detec=False
        self.balle_target_id=-1
        self.has_ball=False
        self.has_objective = False
        self.objective_x = 0
        self.objective_y = 0
        self.commande_robot = Twist()

        #Robot position suscriber
        self.subscription = self.create_subscription(
            Vector3,
            'robot_position',
            self.robot_position_callback,
            10)

        #Ball positions suscriber
        self.subscription = self.create_subscription(
            PoseArray,
            'ball_position',
            self.ball_position_callback,
            10)

        self.subscription = self.create_subscription(
            Bool,
            'ball_inside',
            self.ball_inside_callback,
            10)

        #Moving command publisher
        self.publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            10)

        #Control loop creation
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.control_callback)

    def robot_position_callback(self, msg):
        self.pos_robot=[msg.x,msg.y,msg.z]
        self.robot_detec=True

    def ball_position_callback(self, msg):
        self.num_balle = 0 #
        for i in range(10):
            self.pos_balle[i,0]=msg.poses[i].position.x
            self.pos_balle[i,1]=msg.poses[i].position.y
            if msg.poses[i].position.z > 1:
                self.num_balle 


    def ball_inside_callback(self, msg):
        #Ball inside - Bool message - In variable self.has_ball
        self.has_ball=msg.data

    def get_ball_target_id(self):
        for i in range(self.num_balle):
            coef_id = ((num_balle-i)/num_balle)*2/3
            dist_ball,angle_ball=get_dist_angle(self.pos_robot,self.pos_balle[i])
            dist_max=np.linalg.norm([terrain_x, terrain_y])
            coef_dist=((dist_max-dist_ball)/dist_max)*1/3

    def control_callback(self):
        self.has_objective = False
        self.objective_x,self.objective_y = 0,0
        if self.has_ball:
            for i in range(self.num_balle):
                if test_ball_near(self.pos_robot,self.pos_balle[i],d) && :
                    self.objective_x,self.objective_y=self.pos_balle[i,0],self.pos_balle[i,1]
                    self.has_objective = True
            if not self.has_objective:
                if loc_robot(self.pos_robot[0]) == "left":
                    if test_goal_zone(self.pos_robot):
                        self.objective_x,self.objective_y = goal_zone_left_x,goal_zone_left_y
                        self.has_objective = True
                    else:
                        self.objective_x,self.objective_y = goal_left_x,goal_left_y
                        self.has_objective = True
                else:
                    if test_goal_zone(self.pos_robot):
                        self.objective_x,self.objective_y = goal_zone_right_x,goal_zone_right_y
                        self.has_objective = True
                    else:
                        self.objective_x,self.objective_y = goal_right_x,goal_right_y
                        self.has_objective = True
        else:
            if self.num_balle > 0:
                if self.balle_target_id == -1:
                    self.balle_target_id=get_ball_target_id(self)
                if test_lr(self.pos_robot[0])==test_lr(self.pos_balle[self.balle_target_id,0]):
                    self.objective_x,self.objective_y=self.pos_balle[self.balle_target_id,0],self.pos_balle[self.balle_target_id,1]
                    self.has_objective = True
                else:
                    self.objective_x,self.objective_y=get_passage_wall(self.pos_robot)
                    self.has_objective = True
        if self.has_objective:
            self.commande_robot=command_objective(self.pos_robot,[self.objective_x,self.objective_y])



def main(args=None):
    rclpy.init(args=args)

    robot_control = Robot_Control()

    rclpy.spin(robot_control)

    robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()