import rospy
from pyquaternion import Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from SearchState import SearchState

class Evalutor(SearchState):
    def __init__(self, confidence_thres, dist_thres, goalTracker):
        SearchState.__init__(confidence_thres)
        self.goalTracker = goalTracker
        self.dist_thres = dist_thres
        self.pose = None
        self.parent = None

    def attach(self):
        SearchState.attach(self)
        self.odom_sub = rospy.Subscriber("/fusion/local_fusion/filtered",
                Odometry, self.odomCallback)

    def detach(self):
        SearchState.detach(self)
        self.odom_sub.unregister()

    def odomCallback(self, data):
        self.pose = data.pose.pose

    def notfoundCallback(self, count):
        if count > 10:
            self.parent.handleSignal("lost")

    def foundCallback(self, orientation, angle, dist):
        if dist > self.dist_thres:
            if self.pose is None:
                return
            # Add goal to goal tracker
            rover_q = Quaternion(self.pose.orientation.w, self.pose.orientation.x,
                    self.pose.orientation.y, self.pose.orientation.z)
            newHeading = orientation.rotate(rover_q)
            offset = newHeading.rotate([dist, 0, 0])

            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = offset[0] + self.pose.position.x
            goal.pose.position.y = offset[1] + self.pose.position.y
            goal.pose.position.z = offset[2] + self.pose.position.z
            goal.pose.orientation.x = rover_q[0]
            goal.pose.orientation.y = rover_q[1]
            goal.pose.orientation.z = rover_q[2]
            goal.pose.orientation.w = rover_q[3]

            self.goalTracker.addNextGoal(goal)
            self.parent.handleSignal("far")
        else:
            self.parent.handleSignal("close")
