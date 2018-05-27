#!/usr/bin/env python
from __future__ import print_function

import math, rospy
import numpy as np
from ControlState import ControlState
from blobfinder import *
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

class BlobSearchState(ControlState):
    def __init__(self, maxSpeedAtDist, maxSpeedAtAngle, minDriveSpeed, minTurningSpeed):
        ControlState.__init__(self, maxSpeedAtDist,
                maxSpeedAtAngle, minDriveSpeed, minTurningSpeed)
        self.received_map = False
        np.set_printoptions(linewidth=100)

    def attach(self):
        ControlState.attach(self)
        #self.odom_sub = rospy.Subscriber("/fusion/local_fusion/filtered", Odometry, self.update)
        self.odom_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.update)
        self.map_sub = rospy.Subscriber("/occupancy_grid", OccupancyGrid, self.mapCallback)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=2)

    def detach(self):
        ControlState.detach(self)
        self.odom_sub.unregister()

    def mapCallback(self, map_msg):
        self.map = np.reshape(map_msg.data,
                (map_msg.info.height, map_msg.info.width))
        self.map_pose = map_msg.info.origin
        self.res = map_msg.info.resolution
        self.received_map = True

    def update(self, odom):
        # Search for tennis ball
        # check if we have reached the end point
        stamped = odom.pose
        if self.received_map:
            x = int((stamped.pose.position.x - self.map_pose.position.x) / self.res)
            y = int((stamped.pose.position.y - self.map_pose.position.y) / self.res)
            leaves, blob = find_blob((x, y),
                    self.map, 0, grid_neighbors, lambda v: (False))
            clk = make_clockwise((x,y))
            path = Path()
            path.header.frame_id = "map"
            for leaf in traverse_leaves(leaves, clk):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.y = leaf[0] * self.res + self.map_pose.position.y
                pose.pose.position.x = leaf[1] * self.res + self.map_pose.position.x
                path.poses.append(pose)
            self.path_pub.publish(path)

if __name__ == "__main__":
    a = BlobSearchState(10, math.pi/2, 150, 100)
    a.attach()
    rospy.init_node("statemachine", anonymous=True)
    rospy.spin()
