#!/usr/bin/python
from matplotlib import animation

import rospy
import json
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Pose2D, TransformStamped, PoseStamped
from sensor_msgs.msg import PointCloud, ChannelFloat32
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
from tf.transformations import quaternion_from_euler
from math import floor, cos, sin
from angles import normalize_angle
import tf2_ros

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
        self.pose_pub = rospy.Publisher('/sparki/transform', TransformStamped, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/obstacle/transform', TransformStamped, queue_size=10)
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
        self.obstacle_pub.publish(obstacle)
        self.broadcaster.sendTransform(obstacle)

        try:
            trans = self.buffer.lookup_transform("map", "obstacle", rospy.Time())
            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            self.obstacle_pose.publish(pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return



        # self.map.add_obstacle(x_w, y_w)

    def us_to_world(self, x_us):
        x_r = x_us * cos(np.deg2rad(self.sensors_dict['servo']))
        y_r = x_us * sin(np.deg2rad(self.sensors_dict['servo']))
        x_w = x_r * cos(self.present_pose.theta) - y_r * sin(self.present_pose.theta) + self.present_pose.x
        y_w = x_r * sin(self.present_pose.theta) + y_r * cos(self.present_pose.theta) + self.present_pose.y
        return x_w, y_w

    def cost(self, cell_index_from, cell_index_to):
        pass


if __name__ == "__main__":
    rospy.init_node('Sparki_Controller', anonymous=True)
    controller = SparkiController()
    rospy.spin()
