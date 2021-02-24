#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseArray
from sensor_msgs.msg import Imu
from numpy import array, cos, sin, arctan2, sqrt, sign, cross, arange, meshgrid, pi, zeros
from numpy.linalg import norm
from transforms3d.euler import quat2euler
import matplotlib.pyplot as plt
import time

def sawtooth(x):
    return (x + pi) % (2 * pi) - pi

def model_wall(p, p1, p2):
    def is_left(a, b, c):
        return -sign((c[0, 0] - a[0, 0]) * (b[1, 0] - a[1, 0]) - (c[1, 0] - a[1, 0]) * (b[0, 0] - a[0, 0]))

    def is_in_box(a, b, c):
        if a[0, 0] == b[0, 0]:
            dir = "vertical"
        else:
            dir = "horizontal"
        
        if dir == "horizontal":
            xmin, xmax = min(a[0, 0], b[0, 0]), max(a[0, 0], b[0, 0])
            return (xmin <= c[0, 0] <= xmax)
        else:
            ymin, ymax = min(a[1, 0], b[1, 0]), max(a[1, 0], b[1, 0])
            return (ymin <= c[1, 0] <= ymax)

    dx, dy = p2[0, 0] - p1[0, 0], p2[1, 0] - p1[1, 0]
    ng, nd = array([[-dy], [dx]]), array([[dy], [-dx]])
    ng, nd = ng/norm(ng), nd/norm(nd)
    BA = p1 - p
    u = p2 - p1
    d = norm(cross(BA.flatten(), u.flatten())) / norm(u)

    if is_in_box(p1, p2, p) and (is_left(p, p1, p2) == 1):
        res = (1/d**3) * ng #+ ((p-p1) / norm(p-p1) ** 3 + (p-p2) / norm(p-p2) ** 3)
    elif is_in_box(p1, p2, p) and (is_left(p, p1, p2) == -1):
        res = (1/d**3) * nd #+ ((p-p1) / norm(p-p1) ** 3 + (p-p2) / norm(p-p2) ** 3)
    else:
        res = array([[0], [0]]) #+ (p-p1) / norm(p-p1) ** 3 + (p-p2) / norm(p-p2) ** 3

    return res 

def model_box(p, xmin, xmax, ymin, ymax, M):
    if (xmin < p[0, 0] < xmax) and (ymin < p[1, 0] < ymax):
        return M
    else:
        return array([[0], [0]])

def model_objective(p, p_obj):
    N1, N2 = p[0, 0] - p_obj[0, 0], p[1, 0] - p_obj[1, 0]
    obj_x = - 50 * N1 / (N1**2 + N2**2)**(3/2)
    obj_y = - 50 * N2 / (N1**2 + N2**2)**(3/2)

    return array([[obj_x], [obj_y]])

def model_player(p, p_j):
    N1, N2 = p[0, 0] - p_j[0, 0], p[1, 0] - p_j[1, 0]
    j_x = 100 * N1 / (N1**2 + N2**2)**(4/2)
    j_y = 100 * N2 / (N1**2 + N2**2)**(4/2)

    return array([[j_x], [j_y]])

def model_const(p, p_obj):
    if (p_obj[0, 0] < 15) and (p[0, 0] > 15):
        return -1
    elif (p_obj[0, 0] > 15) and (p[0, 0] < 15):
        return 1
    else:
        return 0

class FieldSubPub(Node):

    def __init__(self):
        super().__init__('field_pub')
        self.position = array([[0.], [0.]])
        self.angle = 0.
        self.objective = array([[0.], [0.]])
        self.objective_status = 2
        self.players = [array([[5.], [5.]]), array([[25.], [5.]])]
        self.avg_speed = 2.0

        self.cst_V = self.const_V(self.position)
        self.current_V = self.cst_V + self.var_V(self.position)
        self.Mx, self.My = arange(0, 30, 0.5), arange(0, 16, 0.3)
        self.X, self.Y = meshgrid(self.Mx, self.My)
        self.cst_V_array = self.array_const_V()
        var = self.array_var_V()
        self.current_V_array = [self.cst_V_array[0] + var[0], self.cst_V_array[1] + var[1]]

        self.publisher_ = self.create_publisher(Twist, '/cmd_yaw', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription1 = self.create_subscription(Point,'/robot_objective', self.objective_callback,10)
        self.subscription2 = self.create_subscription(PoseArray,'/joueurs_coords', self.players_callback,10)
        self.subscription3 = self.create_subscription(Twist,'/gnss_twist', self.pos_callback,10)
        self.subscription4 = self.create_subscription(Imu,'/imu/data', self.ang_callback,10)

    def objective_callback(self, msg):
        self.objective [0, 0], self.objective[1, 0] = msg.x, msg.y
        self.objective_status = msg.z

    def players_callback(self, msg):
        if msg.poses[0].position.z == 1:
            self.players[0][0, 0], self.players[0][1, 0] = msg.poses[0].position.x, msg.poses[0].position.y
        if msg.poses[1].position.z == 1:
            self.players[1][0, 0], self.players[1][1, 0] = msg.poses[1].position.x, msg.poses[1].position.y

    def pos_callback(self, msg):
        self.position[0, 0], self.position[1 ,0] = msg.linear.x, msg.linear.y

    def ang_callback(self, msg):
        self.angle = sawtooth(quat2euler([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])[2] - pi/2.)

    def timer_callback(self):
        self.current_V = self.const_V(self.position) + self.var_V(self.position)
        cmd_msg = Twist()
        if self.objective_status in [0, 1]:
            d = sqrt((self.objective[0, 0] - self.position[0, 0])**2 + (self.objective[1, 0] - self.position[1, 0])**2)
            if d < 1.:
                cmd_msg.linear.x = self.avg_speed * d**2
            elif d < 0.3:
                cmd_msg.linear.x = 0.
            else:
                cmd_msg.linear.x = self.avg_speed
            cmd_msg.angular.z = arctan2((float)(self.current_V[1, 0]), (float)(self.current_V[0, 0]))
        else:
            cmd_msg.linear.x = self.avg_speed
            cmd_msg.angular.z = self.angle
        self.publisher_.publish(cmd_msg)
        self.draw_field()

    def const_V(self, p):
        #gestion des murs
        p1, p2 = array([[0], [0]]), array([[30], [0]])
        Nwall1, Nwall2 = model_wall(p, p1, p2)
        p3, p4 = array([[0], [16]]), array([[30], [16]])
        Nwall3, Nwall4 = model_wall(p, p3, p4)
        p5, p6 = array([[0], [0]]), array([[0], [16]])
        Nwall5, Nwall6 = model_wall(p, p5, p6)
        p7, p8 = array([[30], [0]]), array([[30], [16]])
        Nwall7, Nwall8 = model_wall(p, p7, p8)
        p9, p10 = array([[15], [2.5]]), array([[15], [13.5]])
        Nwall9, Nwall10 = model_wall(p, p9, p10)
        p11, p12 = array([[0], [13.5]]), array([[1.5], [13.5]])
        Nwall11, Nwall12 = model_wall(p, p11, p12)
        p13, p14 = array([[2.7], [14.5]]), array([[2.7], [16]])
        Nwall13, Nwall14 = model_wall(p, p13, p14)
        p15, p16 = array([[27.3], [0]]), array([[27.3], [1.5]])
        Nwall15, Nwall16 = model_wall(p, p15, p16)
        p17, p18 = array([[28.5], [2.5]]), array([[30], [2.5]])
        Nwall17, Nwall18 = model_wall(p, p17, p18)
        # gestion du filet
        net_x_bot, net_y_bot = model_box(p, 13, 17, 1, 8, array([[0], [-1]]))
        net_x_top, net_y_top = model_box(p, 13, 17, 8, 15, array([[0], [1]]))

        const_V_x = net_x_bot + net_x_top + Nwall1 + Nwall3 + Nwall5 + Nwall7 + Nwall9 + Nwall11 + Nwall13 + Nwall15 + Nwall17
        const_V_y = net_y_bot + net_y_top + Nwall2 + Nwall4 + Nwall6 + Nwall8 + Nwall10 + Nwall12 + Nwall14 + Nwall16 + Nwall18

        return array([[const_V_x], [const_V_y]])

    def var_V(self, p):
        if self.objective_status != 2:
            obj_x, obj_y = model_objective(p, self.objective)
            j_x, j_y = model_player(p, self.players[0]) + model_player(p, self.players[1])
            const = model_const(p, self.objective) 
        else:
            obj_x, obj_y = array([[0.], [0.]])
            j_x, j_y = model_player(p, self.players[0]) + model_player(p, self.players[1])
            const = 0.

        current_V_x = const + obj_x + j_x
        current_V_y = obj_y + j_y

        return array([[current_V_x], [current_V_y]])

    def array_const_V(self):
        n, m = self.X.shape
        VX, VY = zeros((n, m)), zeros((n, m))
        for i in range(n):
            for j in range(m):
                p = array([[self.X[i, j]], [self.Y[i, j]]])
                VX[i, j], VY[i, j] = self.const_V(p)
        return VX, VY

    def array_var_V(self):
        n, m = self.X.shape
        VX, VY = zeros((n, m)), zeros((n, m))
        for i in range(n):
            for j in range(m):
                p = array([[self.X[i, j]], [self.Y[i, j]]])
                VX[i, j], VY[i, j] = self.var_V(p)
        return VX, VY

    def draw_field(self):
        #var = self.array_var_V()
        #self.current_V_array = [self.cst_V_array[0] + var[0], self.cst_V_array[1] + var[1]]
        #R = sqrt(self.current_V_array[0] ** 2 + self.current_V_array[1] ** 2)
        plt.cla()
        plt.xlim((-2, 32))
        plt.ylim((-2, 18))
        #plt.quiver(self.Mx, self.My, self.current_V_array[0] / R, self.current_V_array[1] / R)
        # robot        
        plt.arrow(self.position[0, 0], self.position[1, 0], cos(self.angle), sin(self.angle), color='b')
        plt.plot(self.position[0, 0], self.position[1, 0], '.b')
        plt.arrow(self.position[0, 0], self.position[1, 0], cos(self.angle), sin(self.angle), color='b')
        #objective
        obj_color = 'g' if self.objective_status !=2 else 'k'
        current_dir = arctan2(self.current_V[1, 0] / sqrt(self.current_V[1, 0] ** 2 + self.current_V[0, 0] ** 2), self.current_V[0, 0] / sqrt(self.current_V[1, 0] ** 2 + self.current_V[0, 0] ** 2))[0]
        plt.arrow(self.position[0, 0], self.position[1, 0], cos(current_dir), sin(current_dir), color=obj_color)
        plt.plot(self.objective[0, 0], self.objective[1, 0], color=obj_color)
        text = '('+ str(round(self.objective[0, 0], 1)) + '|' + str(round(self.objective[1, 0], 1)) + ')'
        plt.text(self.objective[0, 0], self.objective[1, 0], text, fontsize=12, color = obj_color)
        #players
        plt.plot(self.players[0][0, 0], self.players[0][1, 0], '.r')
        plt.plot(self.players[1][0, 0], self.players[1][1, 0], '.r')
        plt.gcf().canvas.draw_idle()
        plt.gcf().canvas.start_event_loop(0.001)

def main(args=None):
    plt.ion()
    plt.figure()
    plt.xlim((-2, 32))
    plt.ylim((-2, 18))
    rclpy.init(args=args)

    node_ = FieldSubPub()
    rclpy.spin(node_)

    node_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
