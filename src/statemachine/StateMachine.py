import rospy
from geometry_msgs.msg import PoseStamped

# States:
# publish gps location of goal, follow path
# turn in place and look for tennis ball
# explore for tennis ball
class StateMachine:
    # states: dict string->State
    def __init__(self, states, transitions, initial):
        self.current = None
        self.current_name = None
        self.states = states
        self.transitions = transitions
        self.path = None
        self.initial = initial
        self.parent = None
        # Add names
        rospy.init_node("statemachine", anonymous=True)

    def switchTo(self, state):
        print("Switching to %s mode" % state)
        if not self.current is None:
            self.current.detach()
        self.current_name = state
        self.current = self.states[state]
        self.current.parent = self
        self.current.attach()

    def handleSignal(self, signal):
        print("Recieved %s signal from %s state" % (signal, self.current_name))
        nextState = self.transitions[self.current_name+":"+signal]
        if nextState[:4] == "exit":
            self.parent.handleSignal(nextState[5:])
            return
        self.switchTo(nextState)

    def attach(self):
        self.switchTo(self.initial)

    def detach(self):
        self.current.detach()
        self.current = None
