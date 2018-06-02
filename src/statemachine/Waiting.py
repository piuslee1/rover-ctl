import rospy

class Waiting:
    def __init__(self):
        self.parent = None
        rospy.spin()
