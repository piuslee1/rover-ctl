import rospy
from geometry_msgs.msg import PoseStamped

class NextGoal:
    def __init__(self, poses):
        self.index = -1
        self.poses = poses
        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.parent = None

    def attach(self):
        self.index += 1
        if self.index == len(self.poses):
            self.parent.handleSignal("done")
        else:
            self.pub.publish(self.poses[self.index])
            self.parent.handleSignal("set")

    def detach(self):
        return

    def addNextGoal(self, goal):
        self.poses.insert(self.index, goal)
