#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

pub = rospy.Publisher("/killswitch", Bool, queue_size=1)
rospy.init_node("killswitch", anonymous=True)

while not rospy.is_shutdown():
    x = raw_input("Press anything to stahp; type stop to stop the program\n")
    if x == "stop":
        break
    msg = Bool()
    msg.data = True
    pub.publish(msg)
    x = raw_input("Press anything to start; type stop to stop the program\n")
    if x == "stop":
        break
    msg = Bool()
    msg.data = False
    pub.publish(msg)
