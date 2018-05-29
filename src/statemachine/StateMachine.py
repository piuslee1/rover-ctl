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
        self.current = None
        self.states = states
        self.transitions = transitions
        self.path = None
        self.initial = initial
        # Add names
        for key in self.states:
            self.states[key].name = key
        rospy.init_node("statemachine", anonymous=True)

    def switchTo(self, state):
        print("Switching to %s mode" % state)
        if not self.current is None:
            self.current.detach()
        self.current = states[state]
        self.current.attach()
        self.current.parent = self

    def handleSignal(self, signal):
        print("Recieved %s signal from %s state" % (signal, self.current.name))
        nextState = self.transitions[self.current.name+":"+signal]
        if nextState[:4] == "exit":
            self.parent.handleSignal(nextState[5:])
            break
        self.switchTo(nextState)

    def attach(self):
        self.switchTo(initial)

    def detach(self):
        self.current.detach()
        self.current = None

if __name__ == "__main__":
    # Blob search State
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
            "following:reachedBall": "searching",
            "followingSearch:found:close": "nextGoal",
    }
    s = StateMachine(states, stateTransitions, "following")
