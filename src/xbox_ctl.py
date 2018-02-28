#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from serial import Serial

#addr = "00:14:03:05:F1:F5"
MAX_SPEED = 255

def callback(msg):
    turningRadius = -msg.axes[0]
    sign = -1 if turningRadius < 0 else 1
    turningRadius = sign*(4.1-4*abs(turningRadius))
    forwardVel = 255*msg.axes[1]
    turningSpeed = 1 if abs(turningRadius) > 0 else 0
    turningSpeed = (MAX_SPEED-abs(forwardVel))*turningSpeed/4
    msg = b'%f, %f, %f' % (turningRadius, turningSpeed, forwardVel)
    print(msg)
    #s.write(msg)

def init():
    rospy.init_node("xbox_ctl", anonymous=True)
    #s = Serial("/dev/rfcomm0", 9600)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()
    #s.close()

if __name__ == "__main__":
    init()
