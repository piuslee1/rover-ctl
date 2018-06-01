import math, rospy
from rover_ctl.msg import MotorCMD
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from pyquaternion import Quaternion

NUM_OF_JOINTS = 5
STATES = ["xy-plane", "xz-plane", "rotate-grasper"]
BINDINGS = {
    "a": "xy-plane",
    "b": "xz-plane",
    "x": "theta-phi-grasper",
}

def startsWith(string, tag):
    return string[:len(tag)] == tag

class Arm:
    def __init__(self, movespeed, rotspeed):
        self.movespeed = movespeed
        self.rotspeed = rotspeed
        self.pose = None
        self.goal = None
        self.state = 0
        self.joints = [None] * NUM_OF_JOINTS
        self.serial_sub = rospy.Subscriber("/arduino_serial", String, self.serialCallback)

    def serialCallback(self, msg):
        #arm#joint id, goal, current
        if not startsWith(msg, "#arm#"):
            return
        data = msg.split(",")
        self.joints[int(data[1])] = float(data[3])

    def calculatePosition(self):
        """
        returns False if missing joint data or if we fail to calculate position
        """
        for i in self.joints:
            if i is None:
                return False
        # Calculate kinematics
        return True

    def makeMsg(self, msg):
        if self.pose is None:
            return

        l_stick_x = msg.axes[3]
        l_stick_y = msg.axes[4]

        # start at 1, fully pulled = -1
        l_trig = msg.axes[2]
        r_trig = msg.axes[5]
        triggers = (l_trig-r_trig)/2

        buttons = {
            "a": msg.buttons[0],
            "b": msg.buttons[1],
            "x": msg.buttons[2],
            "y": msg.buttons[3],
        }

        # Set state via buttons
        for key in buttons:
            if buttons[key]:
                self.state = STATES.index(BINDINGS[key])

        # Stateful control of arm
        if STATES[self.state] == "xy-plane":
            self.goal = self.pose
            self.goal.position.x = self.pose.position.x + l_stick_x * self.movespeed
            self.goal.position.y = self.pose.position.y + l_stick_y * self.movespeed
            self.goal.position.z = self.pose.position.z + triggers * self.movespeed
        elif STATES[self.state] == "xz-plane":
            self.goal = self.pose
            self.goal.position.x = self.pose.position.x + l_stick_x * self.movespeed
            self.goal.position.y = self.pose.position.y + triggers * self.movespeed
            self.goal.position.z = self.pose.position.z + l_stick_y * self.movespeed
        elif STATES[self.state] == "rotate-grasper":
            self.goal = self.pose
            yaw = Quaternion(axis=[1,0,0], radians=self.rotspeed*l_stick_x)
            pitch = Quaternion(axis=[0,1,0], radians=self.rotspeed*l_stick_y)
            roll = Quaternion(axis=[0,0,1], radians=self.rotspeed*triggers)
            q = Quaternion(self.pose.orientation.w, self.pose.orientation.x,
                           self.pose.orientation.y, self.pose.orientation.z)
            self.goal.orientation.x = (q*yaw*pitch*roll)[1]
            self.goal.orientation.y = (q*yaw*pitch*roll)[2]
            self.goal.orientation.z = (q*yaw*pitch*roll)[3]
            self.goal.orientation.w = (q*yaw*pitch*roll)[0]

        # Calculate inverse kinematics
        cmd = MotorCMD()
        cmd.type = "arm"

        return cmd
