#!/usr/bin/env python
import math
from StateMachine import StateMachine

from Evaluator import Evaluator
from Seeker import Seeker

from End import EndSuccess
from End import EndFail

import rospy

CONFIDENCE_THRESHOLD = 20 # number of tennis ball pixels
DIST_THRESHOLD = 5
MAX_SPEED_AT_DIST = 10 # distance at which rover should be traveling at max speed
MAX_SPEED_AT_ANGLE = math.pi/2 # angular distance at which rover should be turning at max speed
MIN_DRIVE_SPEED = 150
MIN_TURNING_SPEED = 180

seek_states = {
    "evaluator": Evaluator(CONFIDENCE_THRESHOLD, DIST_THRESHOLD, goalTracker),
    "seeker": Seeker(CONFIDENCE_THRESHOLD, MAX_SPEED_AT_DIST, MAX_SPEED_AT_ANGLE,
        MIN_DRIVE_SPEED, MIN_TURNING_SPEED_SPEED),
    "failure": EndFail(),
    "success": EndSuccess(),
}

seek_transitions = {
    "evaluator:far": "failure",
    "evaluator:close": "seeker",
    "evaluator:lost": "success",
    "seeker:reached": "exit:waypoint",
    "seeker:lost": "failure",
}

seek = StateMachine(seek_states, seek_transitions, "evalulator")

seek.attach()
rospy.spin()

