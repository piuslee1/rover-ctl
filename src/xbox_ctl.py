#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from serial import Serial
import time

#addr = "00:14:03:05:F1:F5"
MAX_SPEED = 255
DEAD_ZONE = 0.1
MSG_RATE  = 10 #in Hz

MSG_PER = 1/MSG_RATE

last_message_send = 0

def deadzone(raw_value):
    if abs(raw_value) < DEAD_ZONE:
        return 0
    return raw_value

def callback(msg):
    if time.time() - last_message_send < MSG_PER:
        return
    last_message_send = time.time()
    
    turningRadius = -deadzone(msg.axes[0])
    sign = -1 if turningRadius < 0 else 1
    turningRadius = sign*(4.1-4*abs(turningRadius))
    forwardVel = 255*deadzone(msg.axes[1])
    turningSpeed = 1 if abs(turningRadius) > 0 else 0
    turningSpeed = (MAX_SPEED-abs(forwardVel))*turningSpeed/4
    msg = b'%f, %f, %f' % (turningRadius, turningSpeed, forwardVel)
    print(msg)
    #s.write(msg)

def init():
    rospy.init_node("xbox_ctl", anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()

if __name__ == "__main__":
    init()
