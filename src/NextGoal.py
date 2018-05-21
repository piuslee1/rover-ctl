import rospy
from geometry_msgs.msg import PoseStamped

class NextGoal:
    def __init__(self, poses):
        self.index = -1
        self.poses = poses
        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    def attach(self):
        self.index += 1
        self.pub.publish(poses[self.index])
        self.parent.switchTo("following")

    def detach(self):
        return
