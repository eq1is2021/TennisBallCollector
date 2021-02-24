import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import math
from rclpy.qos import qos_profile_sensor_data

class Labeliser(Node):

    def __init__(self):
        super().__init__('node_labelisation_balles')
        self.publisher_pose = self.create_publisher(PoseArray, 'balles_labels', 10)
        self.balles = self.create_subscription(PoseArray, 'balles_coords', self.balles_callback, 10)
        #https://answers.ros.org/question/361030/ros2-image-subscriber/
        self.sub_images = self.create_subscription(Image, '/zenith_camera/image_raw', self.image_callback, qos_profile_sensor_data)

        self.bridge = CvBridge()

        self.count = 0 #nb de balles déjà vues

        #array des balles 
        #on sait qu'il y a 10 balles
        #chaque balles a un état (variable poses.position.z)
        #-0 : balle pas encore vue (ou balle disparue du plateau)
        #-1 : balle trouvé et vue
        #-2 : balle perdue
        #-3 : balle dans le robot 
        #-4 : balle déposée (dans une zone orange)
        self.array_balles = PoseArray()  

        self.publisher_pose  # prevent unused variable warning
        print("Node Labelisation des balles : Noeud lancé")
        print("Node Labelisation des balles : Une fenêtre doit s'afficher")

    # Define a callback for the Image message
    def image_callback(self, img_msg):
        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))


        #affichage du label sur l'image
        font = cv2.FONT_HERSHEY_SIMPLEX
        for i in range(len(self.array_balles.poses)):
            pose = self.array_balles.poses[i]
            if(pose.position.z == 1.):
                cv2.putText(cv_image,str(i),(int(pose.position.x),int(pose.position.y)), font, 1,(0,255,0),2,cv2.LINE_AA)
            elif(pose.position.z == 2.):
                cv2.putText(cv_image,str(i),(int(pose.position.x),int(pose.position.y)), font, 1,(0,0,255),2,cv2.LINE_AA)
            elif(pose.position.z == 3.):
                cv2.putText(cv_image,str(i),(int(pose.position.x),int(pose.position.y)), font, 1,(0,255,255),2,cv2.LINE_AA)
            elif(pose.position.z == 4.):
                cv2.putText(cv_image,str(i),(int(pose.position.x),int(pose.position.y)), font, 1,(255,0,255),2,cv2.LINE_AA)

        # Show the converted image
        cv2.imshow('image',cv_image)
        cv2.waitKey(1)

    def balles_callback(self, msg):
        """
        if len(msg.poses) > 0:
            if self.count == 0:
                if len(msg.poses) == 1:
                    self.array_balles.poses[0] = msg.poses[0]
                    self.array_balles.poses[0].position.z = 1.
                    self.count +=1
            else:
                for j in range(self.count):
                    dist = []
                    for i in range(len(msg.poses)):
                        if msg.poses[i].position.z == 0.:
                            dist.append(self.dist(self.array_balles.poses[j], msg.poses[i]))
                        else:
                            dist.append(float("inf"))

                    mi = min(dist)
                    if(mi < 100 ):
                        index = dist.index(mi)
                        self.array_balles.poses[j].position = msg.poses[index].position
                        self.array_balles.poses[j].position.z = 1.
                        msg.poses[index].position.z = 1.
                    else:
                        #balle perdue
                        self.array_balles.poses[j].position.z = 2.


                if len(msg.poses) > self.count:
                    for i in range(len(msg.poses)):
                        if msg.poses[i].position.z == 0.:
                            self.array_balles.poses[self.count] = msg.poses[i]
                            self.array_balles.poses[self.count].position.z = 1.
                    self.count+= 1

            self.send_balles() 
        """
        #on ajoute une balle pour l'initialisation
        if(len(msg.poses) > 0 and len(self.array_balles.poses) == 0):
            pose = Pose() 
            pose.position = msg.poses[0].position
            pose.position.z = 2.
            self.array_balles.poses.append(pose)

        for j in range(len(self.array_balles.poses)):
            if self.array_balles.poses[j].position.z == 1. :
                self.array_balles.poses[j].position.z = 2.

        #on parcours toutes les balles vues
        for i in range(len(msg.poses)):
            if msg.poses[i].position.z == 0.:
                dist = []
                #on parcours la liste des balles vues à l'état précedent pour trouver la plus proche
                for j in range(len(self.array_balles.poses)):
                    if self.array_balles.poses[j].position.z == 2. :
                        dist.append(self.dist(self.array_balles.poses[j], msg.poses[i]))
                    else:
                        dist.append(float("inf"))
                mi = min(dist)
                if(mi < 100 ):
                    #une balle est trouvée
                    index = dist.index(mi)
                    self.array_balles.poses[index].position = msg.poses[i].position
                    self.array_balles.poses[index].position.z = 1.
                else :
                    #balle pas trouvée, on l'ajoute dans la liste
                    pose = Pose() 
                    pose.position = msg.poses[i].position
                    pose.position.z = 2.
                    self.array_balles.poses.append(pose)
        self.send_balles() 


    def send_balles(self):
        msg = PoseArray() 
        x_offset = 30
        y_offset = 683
        scale = 0.02481389578163772
        for i in range(len(self.array_balles.poses)):
            if(self.array_balles.poses[i].position.z == 1.):
                pose = Pose() 
                pose.position.x = (self.array_balles.poses[i].position.x-x_offset)*scale
                pose.position.y = (-self.array_balles.poses[i].position.y+y_offset)*scale
                pose.position.z = self.array_balles.poses[i].position.z
                msg.poses.append(pose)


        while(len(msg.poses) < 10):
            pose = Pose() 
            pose.position.x = 0.
            pose.position.y = 0.
            pose.position.z = 0.
            msg.poses.append(pose)

        self.publisher_pose.publish(msg) 


    def dist(self, p1, p2): #Pose, Pose
        return math.sqrt((p1.position.x-p2.position.x)**2 + (p1.position.y-p2.position.y)**2)
        

def main(args=None):
    rclpy.init(args=args)
    print("Node Labelisation des balles : Initialisation du node de tracking/labelisation")
    labelise = Labeliser()
    rclpy.spin(labelise)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    labelise.destroy_node()
    rclpy.shutdown()
    
    print("Node Labelisation des balles : Fin du programme de labelisation")

    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()
