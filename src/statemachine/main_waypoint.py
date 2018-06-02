#!/usr/bin/env python
import math
from geometry_msgs.msg import PoseStamped
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

firstPose = PoseStamped()
firstPose.pose.position.x = 2
secondPose = PoseStamped()
secondPose.pose.position.x = 2
secondPose.pose.position.y = 2
goalTracker = NextGoal([firstPose, secondPose])

waypoint_states = {
    "nextGoal": goalTracker,
    "following": FollowingState(MAX_SPEED_AT_DIST, MAX_SPEED_AT_ANGLE,
        MIN_DRIVE_SPEED, MIN_TURNING_SPEED),
    "success": EndSuccess(),
}
print(waypoint_states)

waypoint_transitions = {
    "nextGoal:set": "following",
    "nextGoal:done": "success",
    "following:reached": "nextGoal",
}

waypoint = StateMachine(waypoint_states, waypoint_transitions, "nextGoal")

waypoint.attach()
rospy.spin()

