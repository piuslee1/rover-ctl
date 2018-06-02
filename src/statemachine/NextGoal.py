import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

class NextGoal:
    def __init__(self, poses):
        self.index = -1
        self.poses = poses
        self.pos_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.gps_pub = rospy.Publisher("/gps_goal_fix", NavSatFix, queue_size=10)
        self.parent = None

    def attach(self):
        self.index += 1
        if self.index == len(self.poses):
            self.parent.handleSignal("done")
        else:
            pose = self.poses[self.index]
            pose.header.frame_id = "map"
            if isinstance(self.poses[self.index], PoseStamped):
                for i in range(5):
                    self.pos_pub.publish(pose)
            else:
                for i in range(5):
                    self.gps_pub.publish(pose)
            self.parent.handleSignal("set")

    def detach(self):
        return

    def addNextGoal(self, goal):
        self.poses.insert(self.index, goal)
