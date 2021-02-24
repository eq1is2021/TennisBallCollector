import rclpy
from rclpy.node import Node
import math
import numpy as np

#Message imports
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

###Création des variables globales - taille du terrain, taille du mur, zone d'entrée dans les buts et buts eux-mêmes
terrain_x=30
terrain_y=16
wall_center_x=terrain_x/2
wall_center_y=terrain_y/2
wall_length=11.15
len_space=terrain_y-wall_center_y-wall_length/2
goal_zone_left_x=2.7
goal_zone_left_y=terrain_y-2.5
goal_zone_right_x=terrain_x-2.7
goal_zone_right_y=2.5
goal_left_x=1
goal_left_y=terrain_y-1
goal_right_x=terrain_x-1
goal_right_y=1

#Fonction renvoyant la distance et l'angle entre le robot et une position (en prenant en compte l'angle du robot)
def get_dist(pos_robot,pos_target):
    x_diff=pos_target[0]-pos_robot[0]
    y_diff=pos_target[1]-pos_robot[1]
    dist=np.linalg.norm([x_diff, y_diff])
    return(dist)

#Fonction renvoyant la position du passage du mur le plus proche du robot
def get_passage_wall(pos_robot):
    if pos_robot[1]<terrain_y/2:
        return(terrain_x/2,len_space/2)
    else:
        return(terrain_x/2,terrain_y-len_space/2)

#Simple test renvoyant si le robot se trouve dans la moitié gauche ou droite du terrain
def test_lr(pos):
    if pos < terrain_x/2:
        return "left"
    else:
        return "right"

#Fonction renvoyant la commande en cmd_vel à envoyer au robot pour qu'il se déplace vers une position cible
# def command_objective(pos_robot,pos_target):
#     dist,angle_diff = get_dist_angle(pos_robot,pos_target)
#     cmd = Twist()
#     if abs(angle_diff) > np.radians(20):
#         cmd.linear.x = 0
#         cmd.linear.y = 0
#         cmd.linear.z = 0
#         cmd.angular.x = 0
#         cmd.angular.y = 0
#         cmd.angular.z = -0.1*angle_diff
#     else:
#         cmd.linear.x = 0.5
#         cmd.linear.y = 0
#         cmd.linear.z = 0
#         cmd.angular.x = 0
#         cmd.angular.y = 0
#         cmd.angular.z = -0.1*angle_diff
#     return(cmd)

#Fonction de test pour voir si on est dans la zone d'entrée du but
def test_goal_zone(pos_robot):
    x_diff_left=goal_zone_left_x-pos_robot[0]
    y_diff_left=goal_zone_left_y-pos_robot[1]
    dist_left=np.linalg.norm([x_diff_left, y_diff_left])
    x_diff_right=goal_zone_right_x-pos_robot[0]
    y_diff_right=goal_zone_right_y-pos_robot[1]
    dist_right=np.linalg.norm([x_diff_right, y_diff_right])
    # print("dist_left: ", dist_left)
    # print("dist_right: ", dist_right)
    if dist_left < 2 or dist_right < 2:
        return True
    else:
        return False

def test_goal(pos_robot):
    # print("testing goal_robot")
    x_diff_left=goal_left_x-pos_robot[0]
    y_diff_left=goal_left_y-pos_robot[1]
    dist_left=np.linalg.norm([x_diff_left, y_diff_left])
    x_diff_right=goal_right_x-pos_robot[0]
    y_diff_right=goal_right_y-pos_robot[1]
    dist_right=np.linalg.norm([x_diff_right, y_diff_right])
    # print("dist_left: ", dist_left)
    # print("dist_right: ", dist_right)
    if dist_left < 1 or dist_right < 1:
        return True
    else:
        return False

#Fonction de test pour voir si une balle est proche du robot par rapport à la disrance d - et si elle se trouve du même côté que le robot lui même
def test_ball_near(pos_robot,pos_balle,d):
    x_diff=pos_balle[0]-pos_robot[0]
    y_diff=pos_balle[1]-pos_robot[1]
    dist=np.linalg.norm([x_diff, y_diff])
    if dist < d and (test_lr(pos_robot[0])==test_lr(pos_balle[0])) and pos_balle[2]==1:
        return True
    else:
        return False

def test_ball_wall(pos_balle):
    #Fonction de test pour voir si une balle est proche du mur. Return True si la balle est loin du mur et on peut la prendre, False si elle est proche du mur
    d_t=0.3
    if pos_balle[0]<d_t or pos_balle[0]>terrain_x-d_t:
        return False
    if pos_balle[1]<d_t or pos_balle[1]>terrain_y-d_t:
        return False
    if (wall_center_x-d_t<pos_balle[0]<wall_center_x+d_t) and (len_space-d_t<pos_balle[1]<len_space+wall_length+d_t):
        return False

    # 4 murs des goal zones à éviter
    if (pos_balle[0]<1+d_t) and (terrain_y-goal_zone_left_y-d_t<pos_balle[1]<terrain_y-goal_zone_left_y+d_t):
        return False
    if (goal_zone_left_x-dt<pos_balle[0]<goal_zone_left_x+d_t) and (pos_balle[1]<terrain_y-1-d_t):
        return False
    if (pos_balle[0]>terrain_x-1-d_t) and (goal_zone_left_y-d_t<pos_balle[1]<goal_zone_left_y+d_t):
        return False
    if (terrain_x-goal_zone_left_x-dt<pos_balle[0]<terrain_x-goal_zone_left_x+d_t) and (pos_balle[1]<1+d_t):
        return False

    return True

# def test_choppe_balle(pos_robot,pos_balle):
#     x_diff=pos_balle[0]-pos_robot[0]
#     y_diff=pos_balle[1]-pos_robot[1]
#     dist=np.linalg.norm([x_diff, y_diff])
#     angle_target=math.atan2(y_diff,x_diff)
#     angle_diff=pos_robot[2]-angle_target
#     if (angle_diff < a and dist < b):
#         return True
#     else:
#         return False

class Robot_Control(Node):

    def __init__(self):
        super().__init__('robot_control')

        #Variables internes
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
        self.goal_status = False

        self.subscription = self.create_subscription(
            Twist,
            'aruco_twist',
            self.robot_position_callback,
            10)

        self.subscription = self.create_subscription(
            PoseArray,
            'balles_labels',
            self.ball_position_callback,
            10)

        self.subscription = self.create_subscription(
            Bool,
            'catcher_up',
            self.ball_inside_callback,
            10)

        #Publisher de la commande de mouvement
        self.publisher_ = self.create_publisher(
            Point,
            'robot_objective',
            10)

        #Création boucle commande
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.control_callback)

    def robot_position_callback(self, msg):
        # On sauvegarde la position du robot dans pos robot, et on vérifie qu'on l'a enregistré via robot_detec
        self.pos_robot=[msg.linear.x,msg.linear.y,msg.angular.z]
        self.robot_detec=True

    def ball_position_callback(self, msg):
        #Callback des balles, on sauvegarde les variables reçues dans pos_balles et num_balles
        # print("ball_position_callback")
        self.num_balle = 0 #
        for i in range(10):
            self.pos_balle[i,0]=msg.poses[i].position.x
            self.pos_balle[i,1]=msg.poses[i].position.y
            self.pos_balle[i,2]=msg.poses[i].position.z
            if msg.poses[i].position.z > 0.5 :
                # Si la balle existe, on la compte
                self.num_balle += 1
                # print(self.num_balle)

    def ball_inside_callback(self, msg):
        #Booléen indiquant si on a une balle dans la cage ou pas
        self.has_ball=not(msg.data)

    def get_ball_target_id(self):
        print("calling get_ball_target_id")
        # Renvoie l'id de la balle à aller chercher
        # On initialise une absence de balle cible
        self.balle_target_id = -1
        weight = 0
        # On parcours toutes les balles
        for i in range(10):
            # Si la balle existe
            # print(self.pos_balle[i][2])
            if self.pos_balle[i][2]==1 and test_ball_wall(self.pos_balle[i]):
                # print("testing ball: ",i)
                # Poids de l'id
                weight_id = ((10-i)/10)*2/3
                dist_ball=get_dist(self.pos_robot,self.pos_balle[i])
                # print("dist_ball: ",dist_ball)
                dist_max=np.linalg.norm([terrain_x, terrain_y])
                # Poids de la distance
                weight_dist=((dist_max-dist_ball)/dist_max)*1/3
                # Poids total de la balle
                weight_ball = weight_id+weight_dist
                # print("weight: ",weight)
                # print("weight_ball: ",weight_ball)
                # Si le poids est supérieur au poids enregistré, on l'enregistre
                if weight_ball > weight:
                    weight = weight_ball
                    self.balle_target_id = i
                    self.get_logger().info("self.balle_target_id: ")
                    self.get_logger().info(str(self.balle_target_id))
                    print("self.balle_target_id: ",self.balle_target_id)
                # En sortie de boucle, on a donc l'id de la balle avec le poids le plus élevé

    def control_callback(self):
        # print(self.pos_balle)
        # On initialise une absence d'objectif
        self.has_objective = False
        self.objective_x,self.objective_y,self.objective_z = 0.,0.,0.
        # Si le robot a une balle dans son panier
        if self.has_ball:
            print("Has Ball")
            # Partie commentée: une seule balle à la fois, on ne peut pas en prendre plusieurs
            # # On regarde si il y a une balle tout près
            # for i in range(10):
            #     if test_ball_near(self.pos_robot,self.pos_balle[i],d):
            #         print("Ball Near")
            #         self.objective_x,self.objective_y=self.pos_balle[i,0],self.pos_balle[i,1]
            #         self.has_objective = True
            #         # S'il y en a une, on va la chercher
            # # Si le robot n'a pas de balle tout près à chercher
            # if not self.has_objective:
            # S'il est à gauche
            if test_lr(self.pos_robot[0]) == "left":
                print("Robot - Left")
                if test_goal(self.pos_robot):
                    print("Inside Goal Zone - Stop")
                    self.objective_x,self.objective_y = 0,0
                    self.objective_z = 2
                    self.has_objective = True
                else:
                    if test_goal_zone(self.pos_robot) or self.goal_status: # S'il est dans la zone d'entrée du but, ou s'il est en train d'aller vers le but, il va vers le but
                        print("Moving Toward Goal")
                        self.objective_x,self.objective_y = goal_left_x,goal_left_y
                        self.objective_z = 1
                        self.goal_status = True
                        self.has_objective = True
                    else: #Sinon, il se dirige vers la zone d'entrée de but
                        print("Moving Toward Goal Entry Zone")
                        self.objective_x,self.objective_y = goal_zone_left_x,goal_zone_left_y
                        self.objective_z = 1
                        self.has_objective = True
            else:
                print("Robot - Right")
                if test_goal(self.pos_robot):
                    print("Inside Goal Zone")
                    self.objective_x,self.objective_y = 0,0
                    self.objective_z = 2
                    self.has_objective = True
                else:
                    if test_goal_zone(self.pos_robot) or self.goal_status:
                        print("Moving Toward Goal")
                        self.objective_x,self.objective_y = goal_right_x,goal_right_y
                        self.objective_z = 1
                        self.has_objective = True
                        self.goal_status = True
                    else:
                        print("Moving Toward Goal Entry Zone")
                        self.objective_x,self.objective_y = goal_zone_right_x,goal_zone_right_y
                        self.objective_z = 1
                        self.has_objective = True
        else:
            # Si le robot n'a pas de balle, il ne doit pas aller vers le but de toute façon.
            self.goal_status = False
            print("No Ball")
            if test_goal(self.pos_robot):
                print("Inside Goal")
                if test_lr(self.pos_robot[0]) == "left":
                    print("Going Outside - Left")
                    self.objective_x,self.objective_y = goal_zone_left_x,goal_zone_left_y
                    self.objective_z = 1
                    self.has_objective = True
                else:
                    print("Going Outside - Right")
                    self.objective_x,self.objective_y = goal_zone_right_x,goal_zone_right_y
                    self.objective_z = 1
                    self.has_objective = True
            else:
                if self.num_balle > 0:
                    if self.balle_target_id == -1:
                        # On prends la balle avec le poids le plus élevé
                        self.get_ball_target_id()
                    print("Target Ball: ",self.balle_target_id)
                    # Si le robot et la balle sont du même côté:
                    # print("self.pos_robot[0]: ",self.pos_robot[0])
                    # print("self.pos_balle[self.balle_target_id][0]: ",self.pos_balle[self.balle_target_id][0])
                    if test_lr(self.pos_robot[0])==test_lr(self.pos_balle[self.balle_target_id][0]):
                        print("Robot and Ball in the same side")
                        self.objective_x,self.objective_y=self.pos_balle[self.balle_target_id][0],self.pos_balle[self.balle_target_id][1]
                        self.has_objective = True
                        # Le robot va chercher la balle
                    else:
                        # Sinon, le robot va vers le passage dans le mur le plus proche
                        print("Robot and ball in different sides")
                        self.objective_x,self.objective_y=get_passage_wall(self.pos_robot)
                        self.objective_z = 1
                        self.has_objective = True
        # Enfin, on envoie la commande du robot
        # self.commande_robot=command_objective(self.pos_robot,[self.objective_x,self.objective_y])
        objectif = Point()
        objectif.x, objectif.y, objectif.z = float(self.objective_x),float(self.objective_y),float(self.objective_z)
        if(self.has_objective == False):
            objectif.z = float(2.)
        self.publisher_.publish(objectif)

def main(args=None):
    rclpy.init(args=args)

    robot_control = Robot_Control()

    rclpy.spin(robot_control)

    robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()