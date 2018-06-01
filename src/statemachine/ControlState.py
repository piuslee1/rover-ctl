#!/usr/bin/env python
import math, rospy
from rover_ctl.msg import MotorCMD
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion

# Using ROS pose for things

# Given a goal vector:
# Turn to desired direction
# Move to desired position
# Turn to final direction

MAX_MOTOR_SPEED = 255
#MAX_MOTOR_SPEED = 200
HEADING_DEAD_BAND = math.pi/8
POSITION_DEAD_BAND = 2.0 # m

class ControlState:
    def __init__(self, maxSpeedAtDist, maxSpeedAtAngle, minDriveSpeed, minTurningSpeed):
        self.maxSpeedAtDist = maxSpeedAtDist
        self.maxSpeedAtAngle = maxSpeedAtAngle
        self.minDriveSpeed = minDriveSpeed
        self.minTurningSpeed = minTurningSpeed
        self.parent = None

    def attach(self):
        self.pub = rospy.Publisher("/motor_ctl", MotorCMD, queue_size=10)

    def detach(self):
        return

    # Get heading of pose
    def getHeading(self, pose):
        q = Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
        v1 = q.rotate([1, 0, 0])
        return math.atan2(v1[1], v1[0])

    def turnTo(self, angle, roverPose):
        current = self.getHeading(roverPose)
        dist = abs(current - angle)
        if dist < HEADING_DEAD_BAND:
            return True, [0]*6
        else:
            # Turn
            clipped = self.maxSpeedAtAngle if dist > self.maxSpeedAtAngle else dist
            turnSpeed = (MAX_MOTOR_SPEED-self.minTurningSpeed)*math.exp(-self.maxSpeedAtAngle+clipped) + self.minTurningSpeed
            mul = 1 # Left side drives faster
            if current - angle < 0:
                # Right side drives faster
                mul = -1
            return False, [mul*turnSpeed]*3 + [mul*-turnSpeed]*3

    def drive(self, roverPose, goalPose):
        current = self.getHeading(roverPose)
        dist = math.sqrt(
                (roverPose.position.x - goalPose.position.x)**2 +
                (roverPose.position.y - goalPose.position.y)**2 +
                (roverPose.position.z - goalPose.position.z)**2
                )
        if dist < POSITION_DEAD_BAND:
            return True, [0]*6
        else:
            # Turn
            clipped = self.maxSpeedAtDist if dist > self.maxSpeedAtDist else dist
            driveSpeed = (MAX_MOTOR_SPEED-self.minDriveSpeed)*math.exp(-self.maxSpeedAtDist+clipped) + self.minDriveSpeed
            return False, [driveSpeed]*6

    def sendCommand(self, motorSpeeds):
        msg = MotorCMD()
        msg.data = motorSpeeds
        print(msg)
        self.pub.publish(msg)
