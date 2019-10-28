#!/usr/bin/python
from matplotlib import animation

import rospy
import json
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Pose, Pose2D, PoseArray, Point32, PointStamped
from sensor_msgs.msg import PointCloud, ChannelFloat32
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
from tf.transformations import quaternion_from_euler
from math import floor, cos, sin
from angles import normalize_angle


# class SparkiMap(object):
#     def __init__(self, cell_size_x, cell_size_y, M, N):
#         self.pc_pub = rospy.Publisher('/sparki/obstacle_pc', PoseArray, queue_size=10)
#         self.M = M
#         self.N = N
#         self.cell_size_x = cell_size_x
#         self.cell_size_y = cell_size_y
#         self.world_grid = np.zeros(shape=(int(M / cell_size_x), int(N / cell_size_y), 3))
#
#         self.fig = plt.figure()
#         self.img = plt.imshow(self.world_grid, interpolation='nearest')
#         self.sparki_x = 0.0
#         self.sparki_y = 0.0
#
#         #ani = animation.FuncAnimation(self.fig, self.update_fig, interval=50, blit=True)
#         plt.ion()
#         # plt.draw()
#         #plt.pause(0.1)
#         plt.show()
#
#
#
#     def update_position(self, x, y):
#         self.sparki_x = x
#         self.sparki_y = y
#         self.update_fig()
#
#     def update_fig(self):
#         # print("here")
#         # for j in range(int(self.N / self.cell_size_y)):
#         #     for i in range(int(self.M / self.cell_size_x)):
#         #         if ((self.world_grid[i, j] == [0, 0, 0]).all() or (self.world_grid[i, j] == [255, 0, 0]).all()):
#         #             if (floor(self.sparki_x) == i and floor(self.sparki_y) == j):
#         #                 self.world_grid[i, j] = [255, 0, 0]
#         #             else:
#         #                 self.world_grid[i, j] = [0, 0, 0]
#         # self.img.set_data(self.world_grid)
#         #
#         # plt.draw()
#         # plt.pause(0.01)
#         # print(self.sparki_x)
#         # print(self.sparki_y)
#         #return [self.img]
#         pc = PoseArray() # type: PoseArray
#         pc.header.stamp = rospy.Time.now()
#         pc.header.frame_id = "map"
#         p1 = Pose()
#         p1.position.x = 1.0
#         p1.position.y = 1.0
#         p1.position.z = 1.0
#         p1.orientation.w = 1
#
#         p2 = Pose()
#         p2.position.x = 2.0
#         p2.position.y = 2.0
#         p2.position.z = 1.0
#         p2.orientation.w = 1
#
#         pc.poses = [p1, p2]
#         self.pc_pub.publish(pc)


class SparkiController(object):
    IR_THRESHOLD = 300

    def __init__(self):
        # Sparki's Map
        # self.map = SparkiMap(0.01, 0.01, .42, .60)

        # ROS Publishers
        self.motor_pub = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
        self.ping_pub = rospy.Publisher('/sparki/ping_command', Empty, queue_size=10)
        self.odom_pub = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
        self.servo_pub = rospy.Publisher('/sparki/set_servo', Int16, queue_size=10)
        self.render_pub = rospy.Publisher('/sparki/render_sim', Empty, queue_size=10)
        self.pose_pub = rospy.Publisher('/sparki/point', Pose2D, queue_size=10)
        self.pc_pub = rospy.Publisher('/obstacle/point', PointStamped, queue_size=10)
        # ROS Subscribers
        self.odom_sub = rospy.Subscriber('/sparki/odometry', Pose2D, self.update_odom_cb)
        self.state_sub = rospy.Subscriber('/sparki/state', String, self.update_state_cb)

        # ROS Timers
        self.line_follow_tmr = rospy.Timer(rospy.Duration.from_sec(0.1), self.line_follow_cb)
        self.render_tmr = rospy.Timer(rospy.Duration.from_sec(0.1), self.render_cb)
        self.us_tmr = rospy.Timer(rospy.Duration.from_sec(0.1), self.update_obstacle)

        # State Info
        self.present_pose = Pose2D()
        self.sensors_dict = {}

    def render_cb(self, msg):
        self.render_pub.publish(Empty())

    def update_odom_cb(self, msg):
        """
        :type msg: Pose2D
        :param msg:
        :return:
        """
        self.present_pose = msg
        self.present_pose.theta = normalize_angle(self.present_pose.theta)
        # self.map.update_position(msg.x, msg.y)
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "map"
        point.point.x = msg.x
        point.point.y = msg.y
        point.point.z = 1
        self.point_pub.publish(point)

    def update_state_cb(self, msg):
        """
        :type msg: String
        :param msg:
        :return:
        """
        self.sensors_dict = json.loads(msg.data)
        # rospy.loginfo(self.sensors_dict)
        if self.sensors_dict["servo"] != 80.0:
            self.servo_pub.publish(Int16(80))
        self.ping_pub.publish(Empty())

    def line_follow_cb(self, event):
        if "light_sensors" in self.sensors_dict:
            ir = self.sensors_dict["light_sensors"]
            motor_com = Float32MultiArray()
            motor_com.data = [1.00, 1.00]

            if ir[1] < self.IR_THRESHOLD:
                motor_com.data = [-1.00, 1.00]

            if ir[3] < self.IR_THRESHOLD:
                motor_com.data = [1.00, -1.00]

            if ir[2] < self.IR_THRESHOLD < ir[1] and ir[3] > self.IR_THRESHOLD:
                motor_com.data = [1.00, 1.00]

            self.motor_pub.publish(motor_com)

    def update_obstacle(self, event):
        if "ping" in self.sensors_dict:
            x_us = self.sensors_dict["ping"]
            if x_us < 0:
                return
            x_w, y_w = self.us_to_world(x_us)
            obstacle = PointStamped()
            obstacle.header.frame_id = "map"
            obstacle.header.stamp = rospy.Time.now()
            obstacle.point.x = x_w
            obstacle.point.y = y_w
            obstacle.point.z = 0.0
            self.pc_pub.publish(obstacle)

        # self.map.add_obstacle(x_w, y_w)

    def us_to_world(self, x_us):
        x_r = x_us * cos(np.deg2rad(self.sensors_dict['servo']))
        y_r = x_us * sin(np.deg2rad(self.sensors_dict['servo']))
        x_w = x_r * cos(self.present_pose.theta) + self.present_pose.x
        y_w = y_r * sin(self.present_pose.theta) + self.present_pose.y
        return x_w, y_w

    def cost(self, cell_index_from, cell_index_to):
        pass


if __name__ == "__main__":
    rospy.init_node('Sparki_Controller', anonymous=True)
    controller = SparkiController()
    rospy.spin()
