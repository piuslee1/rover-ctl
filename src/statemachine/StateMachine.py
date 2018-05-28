import rospy
from FollowingState import FollowingState
from InplaceSearchState import InplaceSearchState
from geometry_msgs.msg import PoseStamped
from NextGoal import NextGoal

# States:
# publish gps location of goal, follow path
# turn in place and look for tennis ball
# explore for tennis ball
class StateMachine:
    # states: dict string->State
    def __init__(self, states, transitions, initial):
        self.current = states[initial]
        self.states = states
        self.transitions = transitions
        self.path = None
        self.blobSearch = False
        rospy.init_node("statemachine", anonymous=True)
        rospy.spin()

    def switchTo(self, state):
        self.current.detach()
        self.current = states[state]
        self.current.attach()
        self.current.parent = self

    def handleSignal(self, signal):
        nextState = self.transitions[self.current.name+":"+signal]
        self.switchTo(nextState)

if __name__ == "__main__":
    firstPose = PoseStamped()
    secondPose = PoseStamped()
    states = {
            "nextGoal": NextGoal([firstPose, secondPose]),
            "following": FollowingState(10, math.pi/2, 150, 100),
            "followingSearch": FollowingSearchState(10, math.pi/2, 150, 100),
            "searching": InplaceSearchState(10, math.pi/2, 150, 100),
            "blobsearch": BlobSearchState(10, math.pi/2, 150, 100),
    }
    stateTransitions = {
            # Waypoint following
            "nextGoal:set": "following",
            "nextGoal:done": "waiting",
            "following:reached": "searching",
            # Found goal
            "searching:found:far": "following",
            "searching:found:close": "nextGoal",
            "searching:notfound": "aggroSearch",
            # Blob search loop
            "blobsearch:done": "following",
            "following:reachedPath": "followingSearch",
            "followingSearch:reached": "blobsearch",
            # Found goal
            "followingSearch:found:far": "following",
            "followingSearch:found:close": "nextGoal",
    }
    s = StateMachine(states, stateTransitions, "following")
