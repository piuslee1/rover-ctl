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

#MAX_MOTOR_SPEED = 255
MAX_MOTOR_SPEED = 200
HEADING_DEAD_BAND = math.pi/8
POSITION_DEAD_BAND = 1.0 # m

def getHeading(pose):
    q = Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
    v1 = q.rotate([1, 0, 0])
    return math.atan2(v1[1], v1[0])

class Rover:
    def __init__(self, maxSpeedAtDist, maxSpeedAtAngle, minDriveSpeed, minTurningSpeed):
        self.state = "idle" # "aiming" "moving" "finetuning"
        self.goal_pose = None
        self.maxSpeedAtDist = maxSpeedAtDist
        self.maxSpeedAtAngle = maxSpeedAtAngle
        self.minDriveSpeed = minDriveSpeed
        self.minTurningSpeed = minTurningSpeed
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.setGoalCallback)
        rospy.Subscriber("/fusion/local_fusion/filtered", Odometry, self.update)
        rospy.init_node("inplace_pathing", anonymous=True)
        self.pub = rospy.Publisher("/motor_ctl", MotorCMD, queue_size=10)
        rospy.spin()
        """
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update()
            rospy.spinOnce()
            r.sleep()
        """

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
            turnSpeed = (MAX_MOTOR_SPEED-self.minTurningSpeed)*math.exp(-self.maxSpeedAtAngle+clipped) + self.minTurningSpeed
            mul = 1 # Left side drives faster
            if current - angle < 0:
                # Right side drives faster
                mul = -1
            return False, [mul*turnSpeed]*3 + [mul*-turnSpeed]*3

    def drive(self, roverPose):
        current = getHeading(roverPose)
        dist = math.sqrt(
                (roverPose.position.x - self.goal_pose.position.x)**2 +
                (roverPose.position.y - self.goal_pose.position.y)**2 +
                (roverPose.position.z - self.goal_pose.position.z)**2
            )
        print(dist)
        if dist < POSITION_DEAD_BAND:
            return True, [0]*6
        else:
            # Turn
            clipped = self.maxSpeedAtDist if dist > self.maxSpeedAtDist else dist
            print(clipped)
            driveSpeed = (MAX_MOTOR_SPEED-self.minDriveSpeed)*math.exp(-self.maxSpeedAtDist+clipped) + self.minDriveSpeed
            return False, [driveSpeed]*6

    def sendCommand(self, motorSpeeds):
        msg = MotorCMD()
        msg.data = motorSpeeds
        print(msg)
        self.pub.publish(msg)

    def setGoalCallback(self, goalMsg):
        self.goal_pose = goalMsg.pose
        self.setState("aiming")

    def setState(self, state):
        print("Reached state %s" % state)
        self.state = state

    def update(self, msg):
        roverPose = msg.pose.pose
        if self.goal_pose is not None:
            goalHeading = self.calcGoalAngle(roverPose)
            if self.state == "aiming":
                # Turn to desired heading
                print(goalHeading)
                print(getHeading(roverPose))
                reached, motorctl = self.turnTo(goalHeading, roverPose)
                if reached:
                    self.setState("moving")
                else:
                    self.sendCommand(motorctl)
            elif self.state == "finetuning":
                reached, motorctl = self.turnTo(getHeading(self.goal_pose), roverPose)
                if reached:
                    self.goal_pose = None
                    self.setState("idle")

                self.sendCommand(motorctl)
            elif self.state == "moving":
                # check if heading is still correct, if not set state to turning
                headingCorrect, motorctl = self.turnTo(goalHeading, roverPose)
                if not headingCorrect:
                    self.sendCommand(motorctl)
                else:
                    # drive
                    reached, motorctl = self.drive(roverPose)
                    if reached:
                        self.setState("finetuning")
                    else:
                        self.sendCommand(motorctl)
        else:
            print("No goal")

if __name__ == "__main__":
    # maxSpeedAtDist, maxSpeedAtAngle, minDriveSpeed, minTurningSpeed
    r =  Rover(10, math.pi/2, 150, 100)
