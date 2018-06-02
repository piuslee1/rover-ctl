#!/usr/bin/env python
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from NextGoal import NextGoal

from StateMachine import StateMachine
from FollowingState import FollowingState

from InplaceSearchState import InplaceSearchState

from BlobSearchState import BlobSearchState
from FollowingSearchState import FollowingSearchState

from Evaluator import Evaluator
from Seeker import Seeker

from End import EndSuccess

import rospy

CONFIDENCE_THRESHOLD = 20 # number of tennis ball pixels
DIST_THRESHOLD = 5
MAX_SPEED_AT_DIST = 10 # distance at which rover should be traveling at max speed
MAX_SPEED_AT_ANGLE = math.pi/2 # angular distance at which rover should be turning at max speed
MIN_DRIVE_SPEED = 150
MIN_TURNING_SPEED = 180

pose1 = NavSatFix()
#pose1.longitude = 
#pose1.latitude = 

pose2 = NavSatFix()
#pose2.longitude = 
#pose2.latitude = 

pose3 = NavSatFix()
#pose3.longitude = 
#pose3.latitude = 
goalTracker = NextGoal([pose1, pose2, pose3])

waypoint_states = {
    "nextGoal": goalTracker,
    "following": FollowingState(MAX_SPEED_AT_DIST, MAX_SPEED_AT_ANGLE,
        MIN_DRIVE_SPEED, MIN_TURNING_SPEED),
}

waypoint_transitions = {
    "nextGoal:set": "following",
    "nextGoal:done": "exit:end",
    "following:reached": "exit:searching",
}

searching_states = {
    "searching": InplaceSearchState(CONFIDENCE_THRESHOLD, MAX_SPEED_AT_DIST,
        MAX_SPEED_AT_ANGLE, MIN_DRIVE_SPEED, MIN_TURNING_SPEED),
}

searching_transitions = {
    "searching:found": "exit:seek",
    "searching:notfound": "exit:blobsearch",
}

blob_states = {
    "blobsearch": BlobSearchState(MAX_SPEED_AT_DIST, MAX_SPEED_AT_ANGLE,
        MIN_DRIVE_SPEED, MIN_TURNING_SPEED),
    "following": FollowingState(MAX_SPEED_AT_DIST, MAX_SPEED_AT_ANGLE,
        MIN_DRIVE_SPEED, MIN_TURNING_SPEED),
    # searching while following
    "followingSearch": FollowingSearchState(CONFIDENCE_THRESHOLD,
        MAX_SPEED_AT_DIST, MAX_SPEED_AT_ANGLE, MIN_DRIVE_SPEED, MIN_TURNING_SPEED), 
}

blob_transitions = {
    "blobsearch:done": "following",
    "following:reached": "followingSearch",
    "followingSearch:reached": "blobsearch",
    "followingSearch:found": "exit:seek",
}

seek_states = {
    "evaluator": Evaluator(CONFIDENCE_THRESHOLD, DIST_THRESHOLD, goalTracker),
    "seeker": Seeker(CONFIDENCE_THRESHOLD, MAX_SPEED_AT_DIST, MAX_SPEED_AT_ANGLE,
        MIN_DRIVE_SPEED, MIN_TURNING_SPEED),
}

seek_transitions = {
    "evaluator:far": "exit:waypoint",
    "evaluator:close": "seeker",
    "evaluator:lost": "exit:searching",
    "seeker:reached": "exit:waypoint",
    "seeker:lost": "exit:searching",
}

waypoint = StateMachine(waypoint_states, waypoint_transitions, "nextGoal")
searching = StateMachine(searching_states, searching_transitions, "evalulator")
blob = StateMachine(blob_states, blob_transitions, "blobsearch")
seek = StateMachine(seek_states, seek_transitions, "evalulator")

main_states = {
    "waypoint": waypoint,
    "searching": searching,
    "blobsearch": blob,
    "seek": seek,
    "end": EndSuccess(),
}

main_transitions = {
    "seek": "seek",
    "searching": "searching",
    "waypoint": "waypoint",
    "blobsearch": "blobsearch",
    "end": "end",
}

main = StateMachine(main_states, main_transitions, "waypoint")

main.attach()
rospy.spin()
