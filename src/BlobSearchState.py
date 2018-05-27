#!/usr/bin/env python

import math, rospy
import numpy as np
from ControlState import ControlState
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

class BlobSearchState(ControlState):
    def __init__(self, maxSpeedAtDist, maxSpeedAtAngle, minDriveSpeed, minTurningSpeed):
        ControlState.__init__(self, maxSpeedAtDist,
                maxSpeedAtAngle, minDriveSpeed, minTurningSpeed)

    def attach(self):
        ControlState.attach(self)
        self.odom_sub = rospy.Subscriber("/fusion/local_fusion/filtered", Odometry, self.update)
        self.map_sub = rospy.Subscriber("/occupancy_grid", OccupancyGrid, self.mapCallback)

    def detach(self):
        ControlState.detach(self)
        self.odom_sub.unregister()

    def mapCallback(self, map_msg):
        self.map = np.reshape(map_msg.data,
                (map_msg.info.height, map_msg.info.width))

        print(map_msg)

    def update(self, odom):
        # Search for tennis ball
        # check if we have reached the end point
        print(odom)

if __name__ == "__main__":
    a = BlobSearchState(10, math.pi/2, 150, 100)
    a.attach()
    rospy.init_node("statemachine", anonymous=True)
    rospy.spin()
