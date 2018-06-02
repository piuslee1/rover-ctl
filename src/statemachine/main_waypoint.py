#!/usr/bin/env python
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from NextGoal import NextGoal

from StateMachine import StateMachine
from FollowingState import FollowingState

from End import EndSuccess

import rospy

CONFIDENCE_THRESHOLD = 20 # number of tennis ball pixels
DIST_THRESHOLD = 5
MAX_SPEED_AT_DIST = 10 # distance at which rover should be traveling at max speed
MAX_SPEED_AT_ANGLE = math.pi/2 # angular distance at which rover should be turning at max speed
MIN_DRIVE_SPEED = 150
MIN_TURNING_SPEED = 180

pose1 = NavSatFix()
pose1.latitude = 32.882112
pose1.longitude = -117.2343985
pose2 = PoseStamped()
pose2.pose.position.x = 2
pose3 = PoseStamped()
pose3.pose.position.x = 2
pose3.pose.position.y = 2

goalTracker = NextGoal([pose1, pose2, pose3])

waypoint_states = {
    "nextGoal": goalTracker,
    "following": FollowingState(MAX_SPEED_AT_DIST, MAX_SPEED_AT_ANGLE,
        MIN_DRIVE_SPEED, MIN_TURNING_SPEED),
    "success": EndSuccess(),
}

waypoint_transitions = {
    "nextGoal:set": "following",
    "nextGoal:done": "success",
    "following:reached": "nextGoal",
}

waypoint = StateMachine(waypoint_states, waypoint_transitions, "nextGoal")

waypoint.attach()
rospy.spin()

