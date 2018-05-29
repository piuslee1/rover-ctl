#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
from NextGoal import NextGoal

from StateMachine import StateMachine
from FollowingState import FollowingState

from InplaceSearchState import InplaceSearchState

from BlobSearchState import BlobSearchState
from FollowingSearchState import FollowingSearchState

from Evaluator import Evaluator
from Seeker import Seeker

import rospy

CONFIDENCE_THRESHOLD = 20 # number of tennis ball pixels
DIST_THRESHOLD = 5
MAX_SPEED_AT_DIST = 10 # distance at which rover should be traveling at max speed
MAX_SPEED_AT_ANGLE = math.pi/2 # angular distance at which rover should be turning at max speed
MIN_DRIVE_SPEED = 150
MIN_TURNING_SPEED = 180

firstPose = PoseStamped()
secondPose = PoseStamped()
goalTracker = NextGoal([firstPose, secondPose]),

waypoint_states = [
    "nextGoal": goalTracker,
    "following": FollowingState(MAX_SPEED_AT_DIST, MAX_SPEED_AT_ANGLE,
        MIN_DRIVE_SPEED, MIN_TURNING_SPEED_SPEED),
]

waypoint_transitions = [
    "nextGoal:set": "following",
    "nextGoal:done": "waiting",
    "following:reached": "exit:searching",
]

searching_states = [
    "searching": InplaceSearchState(CONFIDENCE_THRESHOLD, MAX_SPEED_AT_DIST,
        MAX_SPEED_AT_ANGLE, MIN_DRIVE_SPEED, MIN_TURNING_SPEED_SPEED),
]

searching_transitions = [
    "searching:found": "exit:seek",
    "searching:notfound": "exit:blobsearch",
]

blob_states = [
    "blobsearch": BlobSearchState(MAX_SPEED_AT_DIST, MAX_SPEED_AT_ANGLE,
        MIN_DRIVE_SPEED, MIN_TURNING_SPEED_SPEED),
    "following": FollowingState(MAX_SPEED_AT_DIST, MAX_SPEED_AT_ANGLE,
        MIN_DRIVE_SPEED, MIN_TURNING_SPEED_SPEED),
    # searching while following
    "followingSearch": FollowingSearchState(CONFIDENCE_THRESHOLD,
        MAX_SPEED_AT_DIST, MAX_SPEED_AT_ANGLE, MIN_DRIVE_SPEED, MIN_TURNING_SPEED_SPEED), 
]

blob_transitions = [
    "blobsearch:done": "following",
    "following:reached": "followingSearch",
    "followingSearch:reached": "blobsearch",
    "followingSearch:found": "exit:seek",
]

seek_states = [
    "evaluator": Evaluator(CONFIDENCE_THRESHOLD, DIST_THRESHOLD, goalTracker),
    "seeker": Seeker(CONFIDENCE_THRESHOLD, MAX_SPEED_AT_DIST, MAX_SPEED_AT_ANGLE,
        MIN_DRIVE_SPEED, MIN_TURNING_SPEED_SPEED),
]

seek_transitions = [
    "evaluator:far": "exit:waypoint",
    "evaluator:close": "seeker",
    "evaluator:lost": "exit:searching",
    "seeker:reached": "exit:waypoint",
    "seeker:lost": "exit:searching",
]

waypoint = StateMachine(waypoint_states, waypoint_transitions, "nextGoal")
searching = StateMachine(searching_states, searching_transitions, "evalulator")
blob = StateMachine(blob_states, blob_transitions, "blobsearch")
seek = StateMachine(seek_states, seek_transitions, "evalulator")

main_states = [
    "waypoint": waypoint,
    "searching": searching,
    "blobsearch": blob,
    "seek": seek,
]

main_transitions = [
    "seek": "seek",
    "searching": "searching",
    "waypoint": "waypoint",
    "blobsearch": "blobsearch",
]

main = StateMachine(main_states, main_transitions, "waypoint")

main.attach()
rospy.spin()
