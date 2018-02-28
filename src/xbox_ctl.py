#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from serial import Serial
import time, math

#addr = "00:14:03:05:F1:F5"
MSG_RATE  = 3 #in Hz
MSG_PER = 1./MSG_RATE
last_message_send = 0
MAX_TURNING_RADIUS = 20

s = Serial("/dev/rfcomm0", 9600)

def callback(msg):
    global last_message_send
    if time.time() - last_message_send < MSG_PER:
        return
    last_message_send = time.time()

    turningRadius = -msg.axes[0]
    sign = -1 if turningRadius < 0 else 1
    turningRadius = sign*(MAX_TURNING_RADIUS*(1.01-abs(turningRadius)))

    forwardVel = 255*msg.axes[1]

    vel_mag = math.sqrt(msg.axes[0]**2 + msg.axes[1]**2)
    turningSpeed = (255*vel_mag-abs(forwardVel))/4

    #turningSpeed = 1 if abs(turningRadius) > 0 else 0
    msg = b'%f,%f,%f,\n' % (turningRadius, turningSpeed, forwardVel)
    print(msg)
    s.write(msg)

def init():
    rospy.init_node("xbox_ctl", anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()
    s.close()

if __name__ == "__main__":
    init()
