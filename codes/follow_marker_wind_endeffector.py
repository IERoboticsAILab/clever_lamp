
#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import sys
import time
from dataclasses import dataclass
import socket
import math
import numpy as np
from simulation4 import closest_point_on_ellipsoid
from calibration import calibrate

# Ellipsoid parameters
center = (0, 0, 10)  # Center of the ellipsoid
semi_axes = (4, 4, 3)  # Semi-axes lengths (a, b, c)
plot_3d = True  # Set to True to visualize the result
#target_point = (2, 5, 0)  # Target point in space (marker position)






@dataclass
class desired_output:
    x: float
    y: float
    z: float
    w: float

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0

    def create(self, data, euler: list):
        " Use this function to change transformations to the data"
        self.x = data.pose.position.x * 100
        self.y = data.pose.position.y * 100
        self.z = data.pose.position.z * 100
        self.w = euler[2]
        return self

sock = None
last_position = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
base_coordinates = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}

def quaternion_to_euler(data: PoseStamped):
    # quaternion to euler z == the heading
    x = data.pose.orientation.x
    y = data.pose.orientation.y
    z = data.pose.orientation.z
    w = data.pose.orientation.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.atan2(t3, t4)

    return [X, Y, Z]

def has_position_changed(new_position, threshold=0.001):
    global last_position
    for axis in ['x', 'y', 'z', 'w']:
        if last_position[axis] is not None:
            if abs(new_position[axis] - last_position[axis]) > threshold:
                return True
    return False

def update_position(new_position, data):
    global last_position
    last_position = new_position
    return data

def callback(data):
    #time.sleep(0.001)
    host = rospy.get_param('~host')
    port = rospy.get_param('~port')
    name = rospy.get_param('~body')

    new_position = {
        'x': round(data.pose.position.x,4),
        'y': round(data.pose.position.y,4),
        'z': round(data.pose.position.z,4),
        'w': round(data.pose.orientation.w,4)
    }

    if has_position_changed(new_position):
        global last_position
        global base_coordinates
        data = update_position(new_position, data)
        desired_output_t = desired_output().create(data, quaternion_to_euler(data))
        print(f"deseired_output: {desired_output_t}")
        print(f"Position: {new_position}")
        print(f"Last Position: {last_position}")

        results = closest_point_on_ellipsoid(center, semi_axes, desired_output_t, plot_3d)

        # Output the results
        print(f"The closest point on the ellipsoid is {results['closest_point']}")
        print(f"The minimum distance from the point to the ellipsoid is {results['min_distance']:.2f}")
        print(f"Pitch: {results['pitch']:.2f}°, Roll: {results['roll']:.2f}°, Yaw: {results['yaw']:.2f}°")

        # Send the data to the robot
        bot.arm.set_ee_pose_components(x=results['closest_point'][0], y=results['closest_point'][1],
                                       z=results['closest_point'][2], roll=results['roll'],
                                       pitch=results['pitch'], yaw=results['yaw'])
        #bot.arm.set_ee_pose_components(x=results['closest_point'][0]-base_coordinates[0], y=results['closest_point'][1]-base_coordinates[1], z=results['closest_point'][2]-base_coordinates[2])
        #bot.arm.set_ee_cartesian_trajectory(x=results['closest_point'][0]-last_position['x'], y=results['closest_point'][1]-last_position['y'], z=results['closest_point'][2]-last_position['z'], roll=results['roll']-last_position['w'],)


def listener():
    rospy.init_node('optitrack_data', anonymous=True)
    name = rospy.get_param('~body')
    namespace = rospy.get_param('~namespace')
    rospy.Rate(120)
    rospy.Subscriber(f"/{namespace}/{name}/pose", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        global base_coordinates
        base_coordinates = calibrate()
        print(f"Base coordinates are: {base_coordinates}")

        bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")
        bot.arm.set_ee_pose_components(x=0.2, z=0.3, pitch=1.5)
        listener()
    finally:
        if sock is not None:
            sock.close()