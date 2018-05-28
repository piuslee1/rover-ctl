#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from rover_ctl.msg import MotorCMD
from nav_msgs.msg import Odometry
import numpy as np
from pyquaternion import Quaternion

class Rover:
    def __init__(self):
        self.q = Quaternion(1, 0, 0, 0)

    def update(self, motor_cmd):
        print("HI")

def init():
    rospy.init_node("vehicle_sim", anonymous=True)
    pub = rospy.Publisher("/fusion/local_fusion/filtered", Odometry, queue_size=10)
    q = Quaternion(msg.pose.orientation.w,msg.pose.orientation.x,
            msg.pose.orientation.y,msg.pose.orientation.z)
    def callback(msg):
        print(msg)
        x = msg.data[1]
        y = msg.data[4]
        q = Quaternion(msg.pose.orientation.w,msg.pose.orientation.x,
                msg.pose.orientation.y,msg.pose.orientation.z)
        diff = q.rotate(np.array([1, 0, 0]))
        nextPose = PoseStamped()
        nextPose.pose.position.x = msg.pose.position.x + diff[0]
        nextPose.pose.position.y = msg.pose.position.y + diff[1]
        nextPose.pose.position.z = msg.pose.position.z + diff[2]
        nextPose.pose.orientation = msg.pose.orientation
        nextPose.header.frame_id = "map"
        rospy.loginfo(nextPose)
        pub.publish(nextPose)
    rospy.Subscriber("/motor_ctl", MotorCMD, callback)
    rospy.spin()

if __name__ == "__main__":
    init()
