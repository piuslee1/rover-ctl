#!/usr/bin/env python
import math, rospy, drive, drill, arm
from rover_ctl.msg import MotorCMD
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion

PUBLISH_ORDER = ["drive", "arm"]

class XBOX:
    def __init__(self):
        rospy.init_node("xbox_ctl", anonymous=True)
        rospy.Subscriber("/joy", Joy, self.callback)
        self.pub = rospy.Publisher("/motor_ctl", MotorCMD, queue_size=10)
        self.last_type = 0
        rospy.spin()

    def callback(self, msg):
        l_stick_x = msg.axes[3]
        l_stick_y = msg.axes[4]

        l_trig = msg.axes[2]
        r_trig = msg.axes[5]

        a_button = msg.buttons[0]
        b_button = msg.buttons[1]
        x_button = msg.buttons[2]
        y_button = msg.buttons[3]

        if PUBLISH_ORDER[self.last_type] == "drive":
            self.pub.publish(drive.makeMsg(msg))
        if PUBLISH_ORDER[self.last_type] == "drill":
            self.pub.publish(drill.makeMsg(msg))
        if PUBLISH_ORDER[self.last_type] == "arm":
            self.pub.publish(arm.makeMsg(msg))
        self.last_type = (self.last_type+1) % len(PUBLISH_ORDER)

if __name__ == "__main__":
    print("Started listening for xbox")
    x = XBOX()
