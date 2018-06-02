import sys

class EndFail:
    def __init__(self):
        self.parent = None
    def attach(self):
        print("Failure")
        sys.exit(-1)

class EndSuccess:
    def __init__(self):
        self.parent = None
    def attach(self):
        print("Success")
        sys.exit(-1)
