import rospy
from rover_ctl.msg import MotorCMD
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from pyquaternion import Quaternion
from solve_kinematics import KinematicSolver

NUM_OF_JOINTS = 5
IVK_STATES = ["xy-plane", "xz-plane", "rotate-grasper"]
STATES = ["manual", "xy-plane", "xz-plane", "rotate-grasper", "locked"]
META_STATES = ["up", "down", "left", "right"]
BINDINGS = {
    "a": "xy-plane",
    "b": "manual",
    "x": "theta-phi-grasper",
    "y": "locked",
}
META_BINDINGS = {
    "d_u": "up",
    "d_d": "down",
    "d_l": "left",
    "d_r": "right",
}

def startsWith(string, tag):
    return string[:len(tag)] == tag

def threshold(val, min_v, max_v):
    if val < min_v:
        return -1
    elif val > max_v:
        return 1
    else:
        return 0

class Arm:
    def __init__(self, movespeed, rotspeed):
        self.movespeed = movespeed
        self.rotspeed = rotspeed
        self.pose = None
        self.goal = None
        self.state = STATES.index("manual")
        self.meta_state = 0
        self.lockPose = None
        self.joints = [None] * NUM_OF_JOINTS
        self.serial_sub = rospy.Subscriber("/arduino_serial", String, self.serialCallback)
        self.solver = KinematicSolver()

    def serialCallback(self, msg):
        #arm#joint id, goal, current
        if not startsWith(str(msg), "#arm#"):
            return
        data = str(msg).split(",")
        self.joints[int(data[1])] = float(data[3])
        if STATES[self.state] != "locked":
            self.lockPose = self.joints

    def calculatePosition(self):
        """
        returns False if missing joint data or if we fail to calculate position
        """
        for i in self.joints:
            if i is None:
                return False
        # Calculate kinematics
        pos = self.solver.ee_translation(self.joints)
        q = Quaternion(matrix=self.solver.chain.forward_kinematics(self.joints))
        self.pose = Pose()
        self.pose.position.x = pos[0]
        self.pose.position.y = pos[1]
        self.pose.position.z = pos[2]
        self.pose.orientation.x = q[1]
        self.pose.orientation.y = q[2]
        self.pose.orientation.z = q[3]
        self.pose.orientation.w = q[0]
        return True

    def makeMsg(self, msg):
        #if self.pose is None:
            #return

        r_stick_x = msg.axes[3]
        r_stick_y = msg.axes[4]

        # start at 1, fully pulled = -1
        l_trig = msg.axes[2]
        r_trig = msg.axes[5]
        triggers = (l_trig-r_trig)/2

        dpad_x = msg.axes[6]
        dpad_y = msg.axes[7]

        buttons = {
            "a": msg.buttons[0],
            "b": msg.buttons[1],
            "x": msg.buttons[2],
            "y": msg.buttons[3],
            "l_b": msg.buttons[4],
            "r_b": msg.buttons[5],
            "setup": msg.buttons[7],
            "d_l": dpad_x > 0.1,
            "d_r": dpad_x < 0.1,
            "d_u": dpad_y > 0.1,
            "d_d": dpad_y < 0.1,
        }

        # Set state via buttons
        for key in BINDINGS:
            if buttons[key]:
                self.state = STATES.index(BINDINGS[key])

        for key in META_BINDINGS:
            if buttons[key]:
                self.meta_state = META_STATES.index(META_BINDINGS[key])

        # Stateful control of arm
        if STATES[self.state] == "xy-plane":
            self.goal = self.pose
            self.goal.position.x = self.pose.position.x + r_stick_x * self.movespeed
            self.goal.position.y = self.pose.position.y + r_stick_y * self.movespeed
            self.goal.position.z = self.pose.position.z + triggers * self.movespeed
        elif STATES[self.state] == "xz-plane":
            self.goal = self.pose
            self.goal.position.x = self.pose.position.x + r_stick_x * self.movespeed
            self.goal.position.y = self.pose.position.y + triggers * self.movespeed
            self.goal.position.z = self.pose.position.z + r_stick_y * self.movespeed
        elif STATES[self.state] == "rotate-grasper":
            self.goal = self.pose
            yaw = Quaternion(axis=[1,0,0], radians=self.rotspeed*r_stick_x)
            pitch = Quaternion(axis=[0,1,0], radians=self.rotspeed*r_stick_y)
            roll = Quaternion(axis=[0,0,1], radians=self.rotspeed*triggers)
            q = Quaternion(self.pose.orientation.w, self.pose.orientation.x,
                           self.pose.orientation.y, self.pose.orientation.z)
            self.goal.orientation.x = (q*yaw*pitch*roll)[1]
            self.goal.orientation.y = (q*yaw*pitch*roll)[2]
            self.goal.orientation.z = (q*yaw*pitch*roll)[3]
            self.goal.orientation.w = (q*yaw*pitch*roll)[0]
        elif STATES[self.state] == "locked":
            yield self.lockPose
        elif STATES[self.state] == "manual":
            wrist_rot = -dpad_x
            base = (buttons["r_b"]-buttons["l_b"])
            hand = dpad_y
            wrist = triggers * 50
            elbow = r_stick_y
            shoulder = r_stick_x
            cmd = MotorCMD()
            cmd.type = "manual"
            cmd.data = [wrist_rot,
                        hand,
                        threshold(wrist, -1, 1),
                        threshold(elbow, -0.1, 0.1),
                        threshold(shoulder, -0.1, 0.1),
                        base]
            yield cmd


        if STATES[self.state] in IVK_STATES:
            # Calculate inverse kinematics
            for i in self.solver.generate_path_to_point(
                    self.joints,
                    [self.pose.position.x, self.pose.position.y, self.pose.position.z],
                    [self.goal.position.x, self.goal.position.y, self.goal.position.z],
                    final_orientation=q*yaw*pitch*roll):
                cmd = MotorCMD()
                cmd.type = "arm"
                cmd.data = i
                yield cmd

        if buttons["setup"]:
            terr = MotorCMD()
            terr.type = "terr"
            yield terr
