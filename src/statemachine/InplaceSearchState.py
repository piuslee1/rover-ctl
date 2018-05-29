import math, vision
from ControlState import ControlState
from SearchState import SearchState
from nav_msgs.msg import Odometry

class InplaceSearchState(ControlState, SearchState):
    def __init__(self, confidence_thres,
            maxSpeedAtDist, maxSpeedAtAngle, minDriveSpeed, minTurningSpeed):
        ControlState.__init__(self, maxSpeedAtDist,
                maxSpeedAtAngle, minDriveSpeed, minTurningSpeed)
        SearchState.__init__(self, confidence_thres)

    def attach(self):
        ControlState.attach(self)
        SearchState.attach(self)
        self.startAngle = None
        self.odom_sub = rospy.Subscriber("/fusion/local_fusion/filtered", Odometry, self.update)

    def detach(self):
        ControlState.detach(self)
        SearchState.detach(self)
        self.odom_sub.unregister()

    def update(self, odom):
        if self.startAngle == None:
            self.startAngle = self.getHeading(odom.pose)
            return
        # check if we have reached the end point
        doTurn, cmd = self.turnTo(self, self.startAngle+2*math.pi, odom.pose)
        if doTurn:
            self.sendCommand(cmd)
        else:
            self.parent.handleSignal("notfound")

    def foundCallback(self, orientation, angle, dist):
        self.parent.handleSignal("found")
