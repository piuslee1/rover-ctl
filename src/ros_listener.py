import rospy
from tkinter import *
import tkinter as tk
import serial, time
import messages
import math

from std_msgs.msg import Float64MultiArray


buffer_count = 0
CLEAR_BUFFER = 2

ser = serial.Serial("/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0", 115200)
time.sleep(1)


def dt_listener(msg):
    global buffer_count
    x = msg.data[0]
    y = msg.data[1]

    left_max = max(abs((y + x) / math.sqrt(x**2 + y**2)), 1e-9) 
    right_max = max(abs(y - x) / math.sqrt(x**2 + y**2), 1e-9) 
    left = (y + x) / left_max 
    right = (y - x) / right_max 

    speeds = [right, right, right, left, left, left]
    mess = messages.Message(messages.TARGET_SYSTEMS.DRIVE, speeds)
    ser.write(b'a' + mess.serialize())
    while ser.in_waiting:
        print(ser.readline())
    print("real hash: " +  str(mess.hash))
    if buffer_count > CLEAR_BUFFER:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    buffer_count += 1

rospy.init_node('dt_listener')
rospy.loginfo("started drive_train")
test_arm_sub = rospy.Subscriber('drive_train', Float64MultiArray, dt_listener)
rospy.spin()