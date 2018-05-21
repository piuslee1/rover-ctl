import math
from util import getHeading
from ControlState import ControlState

class InplaceSearchState(ControlState):
    def __init__(self, maxSpeedAtDist, maxSpeedAtAngle, minDriveSpeed, minTurningSpeed):
        ControlState.__init__(self, maxSpeedAtDist,
                maxSpeedAtAngle, minDriveSpeed, minTurningSpeed)

    def attach(self):
        ControlState.attach(self)
        self.startAngle = None
        self.odom_sub = rospy.Subscriber("/fusion/local_fusion/filtered", Odometry, self.update)

    def detach(self):
        ControlState.detach(self)
        self.odom_sub.unregister()

    def update(self, odom):
        if self.startAngle == None:
            self.startAngle = self.getHeading(odom.pose)
            return
        # Search for tennis ball
        # check if we have reached the end point
        doTurn, cmd = self.turnTo(self, self.startAngle+2*math.pi, odom.pose)
        if doTurn:
            self.sendCommand(cmd)
        else:
            self.parent.switchTo("aggrosearch")
