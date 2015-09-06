#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import sys
import tf
from naoqi import ALProxy
import time

PORT = 9559
robotIP = 'PEPPER68.local'
motionProxy = 0

motionProxy = ALProxy("ALMotion", robotIP, PORT)

def callback(data):
    names  = ["HeadYaw", "HeadPitch"]
    fractionMaxSpeed = 0.05
    quat = data.pose.orientation
    quaternion = (quat.x, quat.y, quat.z, quat.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    angles = [euler[1], 1.0 - euler[0]]
    rospy.loginfo('euler = %f %f %f', euler[0], euler[1], euler[2])
    rospy.loginfo('%f %f', angles[0], angles[1])
    global motionProxy
    motionProxy.setAngles(names, angles, fractionMaxSpeed)
    # rospy.sleep(0.1)

def listener():
    rospy.init_node('pepper_head')
    rospy.Subscriber('tango_pose', PoseStamped, callback, queue_size = 1)
    global motionProxy
    names  = ["HeadYaw", "HeadPitch"]
    fractionMaxSpeed = 0.05
    angles = [0, 0]
    motionProxy.setAngles(names, angles, fractionMaxSpeed)
    rospy.spin()

if __name__ == '__main__':
    listener()
