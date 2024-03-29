#!/usr/bin/python
from matplotlib import animation

import rospy
import json
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Pose2D, TransformStamped, PoseStamped, Point32
from sensor_msgs.msg import PointCloud, ChannelFloat32
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
from tf.transformations import quaternion_from_euler
from math import cos, sin, trunc
from angles import normalize_angle
import tf2_ros

class SparkiController(object):
    IR_THRESHOLD = 300

    def __init__(self):
        # ROS Publishers
        self.motor_pub = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
        self.ping_pub = rospy.Publisher('/sparki/ping_command', Empty, queue_size=10)
        self.odom_pub = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
        self.servo_pub = rospy.Publisher('/sparki/set_servo', Int16, queue_size=10)
        self.render_pub = rospy.Publisher('/sparki/render_sim', Empty, queue_size=10)
        self.pose_pub = rospy.Publisher('/sparki/transform', TransformStamped, queue_size=10)
        self.obstacle_pc_pub = rospy.Publisher('/obstacle/pc', PointCloud, queue_size=10)
        self.obstacle_pose = rospy.Publisher('/obstacle/pose', PoseStamped, queue_size=10)

        # ROS Subscribers
        self.odom_sub = rospy.Subscriber('/sparki/odometry', Pose2D, self.update_odom_cb)
        self.state_sub = rospy.Subscriber('/sparki/state', String, self.update_state_cb)

        # ROS Timers
        # self.render_tmr = rospy.Timer(rospy.Duration.from_sec(0.1), self.render_cb)

        # State Info
        self.present_pose = Pose2D()
        self.sensors_dict = {}

        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.world_map = np.zeros(shape=(200, 200))
        self.pc = PointCloud()

    def update_odom_cb(self, msg):
        """
        :type msg: Pose2D
        :param msg:
        :return:
        """
        self.present_pose = msg
        self.present_pose.theta = normalize_angle(self.present_pose.theta)
        # self.map.update_position(msg.x, msg.y)
        pose = TransformStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.child_frame_id = "sparki"
        pose.transform.translation.x = msg.x
        pose.transform.translation.y = msg.y
        pose.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, self.present_pose.theta)
        pose.transform.rotation.x = q[0]
        pose.transform.rotation.y = q[1]
        pose.transform.rotation.z = q[2]
        pose.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(pose)
        self.pose_pub.publish(pose)

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
        if "ping" in self.sensors_dict:
            self.update_obstacle(self.sensors_dict["ping"])
        self.ping_pub.publish(Empty())
        self.render_pub.publish(Empty())

    def update_obstacle(self, x_us):
        if x_us < 0:
            return
        # x_w, y_w = self.us_to_world(x_us)
        obstacle = TransformStamped()
        obstacle.header.frame_id = "sparki"
        obstacle.header.stamp = rospy.Time.now()
        obstacle.child_frame_id = "obstacle"
        obstacle.transform.translation.x = x_us * cos(np.deg2rad(self.sensors_dict["servo"]))
        obstacle.transform.translation.y = x_us * sin(np.deg2rad(self.sensors_dict["servo"]))
        obstacle.transform.rotation.w = 1
        self.broadcaster.sendTransform(obstacle)

        try:
            trans = self.buffer.lookup_transform("map", "obstacle", rospy.Time())
            x = trunc(trans.transform.translation.x * 100)
            y = trunc(trans.transform.translation.y * 100)
            if self.world_map[x][y] != 1:
                self.world_map[x][y] = 1
                self.pc.header.stamp = rospy.Time.now()
                self.pc.header.frame_id = "map"

                self.pc.points.append(Point32(x/100.0, y/100.0, 0))
                self.pc.channels.append(ChannelFloat32())
                self.obstacle_pc_pub.publish(self.pc)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

    def us_to_world(self, x_us):
        x_r = x_us * cos(np.deg2rad(self.sensors_dict['servo']))
        y_r = x_us * sin(np.deg2rad(self.sensors_dict['servo']))
        x_w = x_r * cos(self.present_pose.theta) - y_r * sin(self.present_pose.theta) + self.present_pose.x
        y_w = x_r * sin(self.present_pose.theta) + y_r * cos(self.present_pose.theta) + self.present_pose.y
        return x_w, y_w

    def cell_to_integer_thing_for_part_4_question_1_a_function(self, x, y):
    	return y*200 + x + 1

    def integer_to_cell_thing_for_part_4_question_1_a_function(self, cell):
    	y = cell // 200
    	cell -= (y*200)
    	x = cell 
    	return (x, y)

    def cost(self, cell_index_from, cell_index_to):
    	if cell_index_to == cell_index_from: #Same cell, no cost
    		return 0
    	if not np.abs(cell_index_from - cell_index_to) == 1: #They are not adjacent
    		return 69420

    	pos_1   = self.integer_to_cell_thing_for_part_4_question_1_a_function(cell_index_from - 1)
    	pos_2   = self.integer_to_cell_thing_for_part_4_question_1_a_function(cell_index_to - 1)
    	start_x = pos_1[0]
    	start_y = pos_1[1]
    	end_x   = pos_2[0]
    	end_y   = pos_2[1]

    	if self.world_map[end_x, end_y] == 1 or self.world_map[start_x, start_y] == 1: #Destination cell is occupied
    		return 69420
    	else:
    		return 0


if __name__ == "__main__":
    rospy.init_node('Sparki_Controller', anonymous=True)
    controller = SparkiController()
    rospy.spin()
    print(controller.cost(204, 202))
