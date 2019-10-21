import rospy
import serial, time
import messages
import math

from std_msgs.msg import Float64MultiArray


buffer_count = 0
CLEAR_BUFFER = 2

ser = serial.Serial("/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0", 115200)
time.sleep(1)

rospy.init_node('dt_listener')
rospy.loginfo("started drive_train")
rate = rospy.Rate(10)

def dt_listener(msg):
    global buffer_count
    x = msg.data[0]
    y = msg.data[1]
    if x**2 + y**2 == 0:
        return

    magnitude = max((abs(y) + abs(x)) / math.sqrt(x**2 + y**2), 1e-9) 
    left = int(255*(y + x) / magnitude)
    right = int(255*(y - x) / magnitude)

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
    rate.sleep()

test_arm_sub = rospy.Subscriber('drive_train', Float64MultiArray, dt_listener)
rospy.spin()
