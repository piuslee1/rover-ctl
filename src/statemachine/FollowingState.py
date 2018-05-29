#!/usr/bin/env python
import math, rospy, time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion
from ControlState import ControlState

# Using ROS pose for things

# Given a goal vector:
# Turn to desired direction
# Move to desired position
# Turn to final direction

MAX_MOTOR_SPEED = 255
#MAX_MOTOR_SPEED = 200
HEADING_DEAD_BAND = math.pi/8
POSITION_DEAD_BAND = 1.0 # m

class FollowingState (ControlState):
    def __init__(self, maxSpeedAtDist, maxSpeedAtAngle, minDriveSpeed, minTurningSpeed):
        ControlState.__init__(self, maxSpeedAtDist,
                maxSpeedAtAngle, minDriveSpeed, minTurningSpeed)
        self.state = "idle" # "aiming" "moving" "finetuning"
        self.goalPose = None
        self.path = None
        self.currentPose = None
        self.goalReached = True
        self.receivedPath = False

    def attach(self)
        ControlState.attach(self)
        self.goalPose = None
        self.path = None
        self.currentPose = None
        self.goalReached = True
        self.receivedPath = False
        self.path_sub = rospy.Subscriber("trajectory", Path, self.setPath)
        self.odom_sub = rospy.Subscriber("/fusion/local_fusion/filtered", Odometry, self.update)
        self.pub = rospy.Publisher("/motor_ctl", MotorCMD, queue_size=10)

    def detach(self):
        ControlState.detach(self)
        self.path_sub.unregister()
        self.odom_sub.unregister()

    def calcGoalAngle(self, roverPose):
        return math.atan2(self.goalPose.position.y - roverPose.position.y,
                self.goalPose.position.x - roverPose.position.x)

    def setPath(self, pathMsg):
        if self.receivedPath == False:
            self.path = pathMsg
        i = 0;
        j = 0;
        dist = 0;
        smallestDist = 99999;
        for k in self.path.poses:
            i += 1
            dist = math.sqrt(
                    (self.currentPose.position.x - k.pose.position.x)**2 +
                    (self.currentPose.position.y - k.pose.position.y)**2 +
                    (self.currentPose.position.z - k.pose.position.z)**2
                    )
            if dist < smallestDist:
                smallestDist = dist
                j = i
        self.setGoalCallback(self.path.poses[j+1].pose)

    def setGoalCallback(self, goalMsg):
        self.goalPose = goalMsg
        self.setState("aiming")

    def setState(self, state):
        print("Reached state %s" % state)
        self.state = state

    def update(self, msg):
        roverPose = msg.pose.pose
        self.currentPose = roverPose
        if self.goalPose is not None:
            print("Got path")
            goalHeading = self.calcGoalAngle(roverPose)
            if self.state == "aiming":
                # Turn to desired heading
                reached, motorctl = self.turnTo(goalHeading, roverPose)
                if reached:
                    self.setState("moving")
                else:
                    self.sendCommand(motorctl)
            elif self.state == "finetuning":
                reached, motorctl = self.turnTo(self.getHeading(self.goalPose), roverPose)
                if reached:
                    self.goalPose = None
                    self.setState("idle")
                    self.parent.handleSignal("reached")

                self.sendCommand(motorctl)
            elif self.state == "moving":
                # check if heading is still correct, if not set state to turning
                headingCorrect, motorctl = self.turnTo(goalHeading, roverPose)
                if not headingCorrect:
                    self.sendCommand(motorctl)
                else:
                    # drive
                    reached, motorctl = self.drive(roverPose, self.goalPose)
                    if reached:
                        self.setState("finetuning")
                    else:
                        self.sendCommand(motorctl)

if __name__ == "__main__":
    # maxSpeedAtDist, maxSpeedAtAngle, minDriveSpeed, minTurningSpeed
    r =  FollowingState(10, math.pi/2, 150, 100)
