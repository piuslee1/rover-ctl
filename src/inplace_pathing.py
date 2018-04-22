import math
from pyquaternion import Quaternion

# Using ROS pose for things

# Given a goal vector:
# Turn to desired direction
# Move to desired position
# Turn to final direction

MAX_MOTOR_SPEED = 255
HEADING_DEAD_BAND = math.pi/8
POSITION_DEAD_BAND = 0.2 # m

def getHeading(pose):
    q = Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
    v1 = q.rotate([1, 0, 0])
    return math.atan2(v1[1], v1[0])

def motorSpeedsToMsg(motorSpeeds):
    msg = b'{},' * 6 + b'\n'
    msg = msg.format(*motorSpeeds)
    return msg

class Rover:
    def __init__(self, maxSpeedAtDist, maxSpeedAtAngle, minDriveSpeed, minTurningSpeed):
        self.state = "idle" # "aiming" "moving" "finetuning"
        self.goal_pose = None
        self.maxSpeedAtDist = maxSpeedAtDist
        self.maxSpeedAtAngle = maxSpeedAtAngle
        self.minDriveSpeed = minDriveSpeed
        self.minTurningSpeed = minTurningSpeed

    def calcGoalAngle(self, roverPose):
        return math.atan2(self.goal_pose.position.y - roverPose.position.y,
                          self.goal_pose.position.x - roverPose.position.x)

    def turnTo(self, angle, roverPose):
        current = getHeading(roverPose)
        dist = abs(current - angle)
        if dist < HEADING_DEAD_BAND:
            return True, [0]*6
        else:
            # Turn
            clipped = self.maxSpeedAtAngle if dist > self.maxSpeedAtAngle else dist
            turnSpeed = (MAX_MOTOR_SPEED-self.minTurningSpeed)*math.exp(self.maxSpeedAtAngle-clipped) + self.minTurningSpeed
            mul = 1 # Left side drives faster
            if current - angle < 0:
                # Right side drives faster
                mul = -1
            return False, [mul*turnSpeed]*3 + [mul*-turnSpeed]*3

    def driveTo(self, dist, roverPose):
        current = getHeading(roverPose)
        dist = math.sqrt(
                (roverPose.position.x - self.goal_pose.position.x)**2,
                (roverPose.position.y - self.goal_pose.position.y)**2,
                (roverPose.position.z - self.goal_pose.position.z)**2,
            )
        if dist < POSITION_DEAD_BAND:
            return True, [0]*6
        else:
            # Turn
            clipped = self.maxSpeedAtAngle if dist > self.maxSpeedAtAngle else dist
            turnSpeed = (MAX_MOTOR_SPEED-self.minTurningSpeed)*math.exp(self.maxSpeedAtAngle-clipped) + self.minTurningSpeed
            mul = 1 # Left side drives faster
            if current - angle < 0:
                # Right side drives faster
                mul = -1
            return False, [mul*turnSpeed]*3 + [mul*-turnSpeed]*3

    def update(self, roverPose):
        if self.goal_pose is not None:
            if self.state = "aiming":
                # Turn to desired heading
                self.turnTo(desiredHeading)
            else if self.state = "finetuning":
                reached, motorctl = self.turnTo(heading)
                if reached:
                    self.goal_pose = None
                    self.state = "idle"

                serial.write(motorSpeedsToMsg(motorctl))
            else if self.state = "moving":
                # check if heading is still correct, if not set state to turning
                # else move the rover
