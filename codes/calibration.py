
#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import sys
import math
import time
from dataclasses import dataclass
import numpy as np



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



def callbackk(data):
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
        return update_position(new_position, data)




def calibrate():
    global last_position
    last_position = {
        'x': None,
        'y': None,
        'z': None,
        'w': None
    }

    rospy.init_node('optitrack_data', anonymous=True)
    name = rospy.get_param('~body')
    namespace = rospy.get_param('~namespace')
    rospy.Rate(120)

    print("calibration sequence started")
    print("put marker on base of the robot and keep it still")
    rospy.Subscriber(f"/{namespace}/{name}/pose", PoseStamped, callbackk)
    print("calibration done")
    print(f"Base coordinates are: {last_position}")
    rospy.spin()

    return last_position