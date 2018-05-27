import math
from ControlState import ControlState

class SpiralSearchState(ControlState):
    def __init__(self, maxSpeedAtDist, maxSpeedAtAngle, minDriveSpeed, minTurningSpeed):
        ControlState.__init__(self, maxSpeedAtDist,
                maxSpeedAtAngle, minDriveSpeed, minTurningSpeed)

    def attach(self):
        ControlState.attach(self)
        self.odom_sub = rospy.Subscriber("/fusion/local_fusion/filtered", Odometry, self.update)

    def detach(self):
        ControlState.detach(self)
        self.odom_sub.unregister()

    def update(self, odom):
        # Search for tennis ball
        # check if we have reached the end point
