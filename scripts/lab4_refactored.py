#!/usr/bin/python

import rospy
import json
import numpy as np
import matplotlib as plt

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
from math import floor, cos, sin


class SparkiMap(object):
    def __init__(self, cell_size_x, cell_size_y, M, N):
        self.M = M
        self.N = N
        self.cell_size_x = cell_size_x
        self.cell_size_y = cell_size_y
        self.world_grid = np.zeros(shape=(int(M / cell_size_x), int(N / cell_size_y), 3))

    def update_position(self, x, y):
        # for j in range(int(self.N/self.cell_size_y)):
        #     for i in range(int(M/self.cell_size_x)):
        #         if self.world_grid[i, j] == [0, 0, 0] or self.world_grid[i, j] == [255, 0, 0]:
        #             if floor(x * 100) == i and floor(y * 100)
        pass

    def show_map(self):
        plt.imshow(self.world_grid, interpolation='nearest')
        plt.show()


class SparkiController(object):
    IR_THRESHOLD = 300

    def __init__(self):
        # ROS Publishers
        self.motor_pub = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
        self.ping_pub = rospy.Publisher('/sparki/ping_command', Empty, queue_size=10)
        self.odom_pub = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
        self.servo_pub = rospy.Publisher('/sparki/set_servo', Int16, queue_size=10)

        # ROS Subscribers
        self.odom_sub = rospy.Subscriber('/sparki/odometry', Pose2D, self.update_odom_cb)
        self.state_sub = rospy.Subscriber('/sparki/state', String, self.update_state_cb)

        # ROS Timers
        self.line_follow_tmr = rospy.Timer(rospy.Duration(secs=1), self.line_follow_cb)

        # State Info
        self.present_pose = Pose2D()
        self.sensors_dict = {}

        # Sparki's Map
        self.map = SparkiMap(0.01, 0.01, .42, .60)

    def update_odom_cb(self, msg):
        """
        :type msg: Pose2D
        :param msg:
        :return:
        """
        self.present_pose = msg

    def update_state_cb(self, msg):
        """
        :type msg: String
        :param msg:
        :return:
        """
        self.sensors_dict = json.load(msg.data)

    def line_follow_cb(self, event):
        ir = self.sensors_dict["light_sensors"]
        motor_com = Float32MultiArray()
        motor_com.data = [0.25, 0.25]

        if ir[1] < self.IR_THRESHOLD:
            motor_com.data = [-0.25, 0.25]

        if ir[3] < self.IR_THRESHOLD:
            motor_com.data = [0.25, -0.25]

            # if ( (lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) )
        if ir[2] < self.IR_THRESHOLD < ir[1] and ir[3] > self.IR_THRESHOLD:
            motor_com.data = [0.25, 0.25]

        self.motor_pub.publish(motor_com)

    def us_to_robot(self, x_us):
        x_r = (x_us/100.0) * cos(self.sensors_dict['servo'] + self.present_pose.theta)
        y_r = (x_us/100.0) * sin(self.sensors_dict['servo'] + self.present_pose.theta)

        return x_r, y_r

    def robot_to_world(self, x_r, y_r):
        x_w = x_r + self.present_pose.x
        y_w = y_r + self.present_pose.y
        return x_w, y_w

    def cost(self, cell_index_from, cell_index_to):
        pass


if __name__ == "__main__":
    rospy.init_node('Sparki-Controller', anonymous=True)
    controller = SparkiController()
    rospy.spin()
