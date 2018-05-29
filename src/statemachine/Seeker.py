import rospy
from pyquaternion import Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from SearchState import SearchState
from ControlState import ControlState

class Seeker(SearchState, ControlState):
    def __init__(self, confidence_thres, 
            maxSpeedAtDist, maxSpeedAtAngle, minDriveSpeed, minTurningSpeed):
        ControlState.__init__(self, maxSpeedAtDist,
                maxSpeedAtAngle, minDriveSpeed, minTurningSpeed)
        SearchState.__init__(confidence_thres)
        self.goalTracker = goalTracker
        self.dist_thres = dist_thres
        self.ball_pose = None

    def attach(self):
        ControlState.attach(self)
        SearchState.attach(self)
        self.odom_sub = rospy.Subscriber("/fusion/local_fusion/filtered",
                Odometry, self.odomCallback)

    def detach(self):
        ControlState.detach(self)
        SearchState.detach(self)
        self.odom_sub.unregister()

    def odomCallback(self, data):
        if self.ball_pose is None:
            return
        pose = data.pose.pose
        # Add goal to goal tracker
        rover_q = Quaternion(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z)
        newHeading = self.ball_ori.rotate(rover_q)
        offset = newHeading.rotate([self.ball_dist, 0, 0])

        ball_pose = Pose()
        ball_pose.position.x = offset[0] + pose.position.x
        ball_pose.position.y = offset[1] + pose.position.y
        ball_pose.position.z = offset[2] + pose.position.z
        ball_pose.orientation.x = rover_q[0]
        ball_pose.orientation.y = rover_q[1]
        ball_pose.orientation.z = rover_q[2]
        ball_pose.orientation.w = rover_q[3]

        # Turn towards goal if not within margin
        desiredHeading = math.atan2(ball_pose.position.y - pose.position.y,
                ball_pose.position.x - pose.position.x)
        doTurn, motorcmd = self.turnTo(desiredHeading, pose)
        if doTurn:
            self.sendCommand(motorcmd)
        else:
            # Move towards goal if on target
            reached, motorcmd = self.drive(pose, ball_pose)
            if reached:
                self.parent.handleSignal("reached")
            else:
                seld.sendCommand(motorcmd)

    def notfoundCallback(self, count):
        if count > 100:
            self.parent.handleSignal("lost")

    def foundCallback(self, orientation, angle, dist):
        self.ball_angle = angle
        self.ball_ori = orientation
        self.ball_dist = dist
