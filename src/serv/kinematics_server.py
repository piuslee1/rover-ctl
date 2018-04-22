#!/usr/bin/env python2.7
import HTTPComputeNode
import sys
import json
import numpy as np
import time
import os
import signal
#from KinematicModel import KinematicModel
from threading import Thread
import rospy
from sensor_msgs.msg import Joy

# kinematics serving and control API
# GET: specified
#   -- /status: get the rover status (currently just the simulated arm)
#   -- (planned) /params: get rover constants like dimensions, etc. 
#   -- 
# POST: specified by the json payload attributes "action" and "data"
#   -- set_goal: set an xyz target location to move the manipulator to
#   -- reset: reset the arm to a safe default configuration
#   --

MSG_TIMEOUT = 5 #seconds

class ControlProcessor:
  def __init__(self,start_configuration,joint_map,precision=0.001):
#    self.model = KinematicModel(start_configuration,joint_map,precision)
    self.running = lambda: True
    self.publisher = None
    self.msg_timer = time.time()

  def set_publisher(self,pub):
      self.publisher = pub

  def processors(self):
    return {
      "get":{
        "processor":self.get_processor,
      },
      "post":{
        "processor":self.post_processor,
      },
    }

  def post_processor(self,content_type,payload,wfile):
    if "application/json" in content_type:
      payload = json.loads(payload)
      # now parse the payload
      if "set_goal" in payload['action']:
        print("setting goal to: {}".format(payload['data']))
#        self.model.set_goal(payload["data"])
      elif "reset" in payload['action']:
#        self.model.reset()
        print("resetting goal")
      elif "joystick-drive" in payload['action']:
        print("got some joystick data: {}".format(payload["data"]))
        if self.publisher:
            msg = Joy()
            msg.axes = payload["data"]["axes"]
#            msg.axes[0] = payload["data"]["axes"][0]
#            msg.axes[1] = payload["data"]["axes"][1]
            print(msg)
            self.msg_timer = time.time()
            self.publisher.publish(msg)
    else:
      wfile.write("content-type not recognized")

  def get_processor(self,path,content_type,wfile):
#    if path == "/status":
#      wfile.write(json.dumps(self.model.configuration))
    wfile.write("ikpy!")

  def start(self):
    thread = Thread(group=None,target=self.update_configuration)
    thread.start()
    return thread

  def update_configuration(self):
    last_print = 0
    while self.running():
#      if not self.model.configured():
#        self.model.update()
        if time.time() - self.msg_timer > MSG_TIMEOUT:
            null_msg = Joy()
            null_msg.axes = [0,0,0,0]
            self.publisher.publish(null_msg)
            self.msg_timer = time.time()
      # if time.time() - last_print > 1:
      #   last_print = time.time()
      #   print(self.model.end_affector)
    print("Processing thread closing...")

  def kill(self):
    self.running = lambda: False

def kill_server(x=None,y=None,z=None):
    print("{}: dying...".format(os.getpid()))
    exit(1)

def main():
  PORT = int(sys.argv[1])
  print("Process: {}".format(os.getpid()))


  start_configuration = {
    "base0":0,
    "joint0":np.pi/2,
    "joint1":-np.pi/2,
    "joint2":-np.pi/2,
    "finger00":-np.pi/3,"finger01":np.pi/3,
    "finger10":np.pi/3,"finger11":-np.pi/3,
  }

  joint_map = [
    "base0",
    "joint0",
    "joint1",
    "joint2"
  ]

  rospy.init_node("http_ctl",anonymous=True)
  pub = rospy.Publisher("/joy",Joy,queue_size=10)

  processor = ControlProcessor(start_configuration,joint_map)
  processor.set_publisher(pub)

  signal.signal(signal.SIGINT,kill_server)

  node = HTTPComputeNode.ControlNode(("",PORT),processor.processors())
  print(node.server_address)

  processor.running = node.running

  thread0 = processor.start()
  node.start()
  processor.kill() # kill the processing thread if the server goes down

if __name__ == "__main__":
    main()
