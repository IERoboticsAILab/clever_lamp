#!/usr/bin/env python3
import rospy
import math
import tf2_ros
import geometry_msgs.msg
from interbotix_xs_modules.arm import InterbotixManipulatorXS

def start_robot():
    global bot
    print("start robot")
    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper", init_node=False)
    bot.arm.set_ee_pose_components(x=0.2, y=0, z=0.3, pitch=1.5)
    print("done robot")

if __name__ == '__main__':
    rospy.init_node('clever_lamp')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    start_robot()
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("umh_5_new", "wx250s/base_link", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        #rospy.loginfo(f"trans: {trans}")
        #rospy.loginfo(f"root: {rot}")
        bot.arm.set_ee_pose_components(x=-trans.transform.translation.x, y=-trans.transform.translation.y, z=0.3, pitch=1.5, yaw=0)

        rate.sleep()




''' NEED TO ADD CONSTRAINS FOR CRAZY MOOVMENTS OF THE ROBOT
import rospy
import math
import tf2_ros
import geometry_msgs.msg
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import time

def start_robot():
    global bot
    print("start robot")
    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper", init_node=False)
    bot.arm.set_ee_pose_components(x=0.2, y=0, z=0.3, pitch=1.5)
    print("done robot")

if __name__ == '__main__':
    rospy.init_node('clever_lamp')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    hight = None
    horizontal = True
    treshold = 0.15

    rate = rospy.Rate(10.0)
    start_robot()
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("umh_5_new", "wx250s/base_link", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        rospy.loginfo(f"trans: {trans}")

        if hight is not None and hight - trans.transform.translation.z >= treshold:
            horizontal = not horizontal
        
        if horizontal:
            bot.arm.set_ee_pose_components(x=-trans.transform.translation.x, y=-trans.transform.translation.y, z=0.3, pitch=1.5, yaw=0)
            hight = trans.transform.translation.z
        else:
            bot.arm.set_ee_pose_components(x=0.3, y=-trans.transform.translation.y, z=abs(trans.transform.translation.x)+0.1, pitch=0, yaw=0)
            hight = trans.transform.translation.z

        rate.sleep()
'''