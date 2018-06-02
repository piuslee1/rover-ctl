import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

class NextGoal:
    def __init__(self, poses):
        self.index = -1
        self.poses = poses
        self.pos_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.gps_pub = rospy.Publisher("/gps_goal", NavSatFix, queue_size=10)
        self.parent = None

    def attach(self):
        self.index += 1
        if self.index == len(self.poses):
            self.parent.handleSignal("done")
        else:
            if isinstance(self.poses[self.index], PoseStamped):
                self.pos_pub.publish(self.poses[self.index])
            else:
                self.gps_pub.publish(self.poses[self.index])
            self.parent.handleSignal("set")

    def detach(self):
        return

    def addNextGoal(self, goal):
        self.poses.insert(self.index, goal)
