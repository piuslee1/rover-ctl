import rospy
from std_msgs.msg import Bool

pub = rospy.Publisher("/killswitch", Bool, queue_size=1)

while not rospy.is_shutdown():
    input("Press anything to stahp")
    msg = Bool()
    msg.data = True
    pub.publish(msg)
    rospy.spinOnce()
    input("Press anything to start")
    msg = Bool()
    msg.data = False
    pub.publish(msg)
    rospy.spinOnce()
