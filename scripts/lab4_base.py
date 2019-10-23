# Group members: Andrew Seiler, Nick Elsasser, Justin Yun, Daniel Koris, Brendan Ostrom
# CSCI 3302-101 - Intro to Robotics
# Lab 4
# Due 10/22/19

import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
import matplotlib.pyplot as plt
import numpy as np
from math import floor, cos, sin

# GLOBALS
pose2d_sparki_odometry = None  # Pose2D message object, contains x,y,theta members in meters and radians
# TODO: Track servo angle in radians
# TODO: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
# TODO: Create data structure to hold map representation

# TODO: Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None

subscriber_odometry = None
subscriber_state = None

sensor_dict = {}  # 'servo', 'light_sensors', 'ping'

SPARKI_SIZE = 5  # sparki is 5cmx5cm

# MxN world grid
# 1cm / 1cm grid size
cell_size_x = 0.01
cell_size_y = 0.01
M = .42  # 42 cm in height
N = .60  # 60 cm in width
world_grid = np.zeros(shape=(int(M / cell_size_x), int(N / cell_size_y), 3))

# CONSTANTS
IR_THRESHOLD = 300  # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_TIME = 0.1  # In seconds


def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry

    # TODO: Init your node to register it with the ROS core
    rospy.init_node('Sp4rk1-C0ntr0ll3r', anonymous=True)
    init()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        # TODO: Implement CYCLE TIME

        # TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))

        # update sparki position on map
        for j in range(int(N / cell_size_y)):
            for i in range(int(M / cell_size_x)):
                if (world_grid[i, j] == [0, 0, 0] or world_grid[i, j] == [255, 0, 0]):
                    if (floor(pose2d_sparki_odometry.x * 100) == i and floor(pose2d_sparki_odometry.y * 100)):
                        world_grid[i, j] = [255, 0, 0]
                    else:
                        world_grid[i, j] = [0, 0, 0]

        ir = sensor_dict["light_sensors"]
        motor_com = Float32MultiArray()

        if ir[1] < IR_THRESHOLD:
            motor_com.data = [-1.0, 1.0]

        if ir[3] < IR_THRESHOLD:
            motor_com.data = [1.0, -1.0]

            # if ( (lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) )
        if ir[2] < IR_THRESHOLD and ir[1] > IR_THRESHOLD and ir[3] > IR_THRESHOLD:
            motor_com.data = [1.0, 1.0]

        publisher_motor.publish(motor_com)
        # TODO: Implement loop closure here
        if False:
            rospy.loginfo("Loop Closure Triggered")

        # TODO: Implement CYCLE TIME
        r.sleep()


def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry

    # publishers
    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
    publisher_ping = rospy.Publisher('/sparki/ping_command', Int16, queue_size=10)
    publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
    publisher_servo = rospy.Publisher('/sparki/set_servo', Int16, queue_size=10)

    # subscribers
    subscriber_odometry = rospy.Subscriber('/sparki/odometry', Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber('/sparki/state', String, callback_update_state)

    # initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    publisher_odom.publish(Pose2D(0.0, 0.0, 0.0))

    # set servo to an angle pointing inward to the map
    publisher_servo.publish(45)


def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    pose2d_sparki_odometry= data


def callback_update_state(data):
    # Creates a dictionary object from the JSON string received from the state topic
    global sensor_dict
    sensor_dict = json.loads(data.data)


def publish_odometry(x, y, theta):
    publisher_odom.publish(Pose2D(x, y, theta))


def convert_ultrasonic_to_robot_coords(x_us):
    # TODO: Using US sensor reading and servo angle, return value in robot-centric coordinates

    # x_r, y_r = 0.0, 0.0

    # not necessarily relative because it also factors in sparki's theta top make calculation easier
    x_r = (x_us / 100.0) * cos(sensor_dict['servo'] + pose2d_sparki_odometry.theta)
    y_r = (x_us / 100.0) * sin(sensor_dict['servo'] + pose2d_sparki_odometry.theta)

    return x_r, y_r


def convert_robot_coords_to_world(x_r, y_r):
    # TODO: Using odometry, convert robot-centric coordinates into world coordinates

    # x_w, y_w = 0.0, 0.0

    x_w = x_r + pose2d_sparki_odometry.x
    y_w = y_r + pose2d_sparki_odometry.y

    return x_w, y_w


def populate_map_from_ping(x_ping, y_ping):
    # TODO: Given world coordinates of an object detected via ping, fill in the corresponding part of the map

    # convert coords to pos
    col = floor(x_ping / cell_size_x)
    row = floor(y_ping / cell_size_y)

    world_grid[row, col] = [255, 255, 255]  # fill in tile

    pass


def display_map():
    # TODO: Display the map

    plt.imshow(world_grid, interpolation='nearest')
    plt.show()

    pass


def ij_to_cell_index(i, j):
    # TODO: Convert from i,j coordinates to a single integer that identifies a grid cell

    # assumptions:
    # i is col, j is row ?
    # i & j are in cm

    # row * num_cols + col
    cell_num = floor(j / SPARKI_SIZE) * int((N / cell_size_x) / SPARKI_SIZE) + floor(i / SPARKI_SIZE)

    # converts the coordinate system to sparki-size unit cells

    return cell_num


def cell_index_to_ij(cell_index):
    # TODO: Convert from cell_index to (i,j) coordinates

    # i, j = 0.0, 0.0

    i = floor(cell_index * SPARKI_SIZE / int((N / cell_size_x))
    j = floor(cell_index * SPARKI_SIZE % int((M / cell_size_y))

    # reverse the above function

    return i, j


def cost(cell_index_from, cell_index_to):
    # TODO: Return cost of traversing from one cell to another

    # use A* shortest path alg

    return 0


if __name__ == "__main__":
    main()


