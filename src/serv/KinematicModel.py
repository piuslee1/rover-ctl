#!/usr/bin/env python2.7
import numpy as np
import time
from solve_kinematics import KinematicSolver
from collections import deque

TAU = 2*np.pi

class BasicActuator:
  def __init__(self,name,angle=0,speed=0.3):
    self.name = name
    self.speed = speed
    self.angle = angle
    self.goal = angle

  def actuate(self,angle):
    # print("current: {}\tgoal:{}".format(self.angle,self.goal))
    self.goal = angle % TAU

  def done(self):
    return self.goal == self.angle

  def update(self,delta):
    residual = self.goal - self.angle
    distance = abs(residual)
    direction = 1 if residual > 0 else -1

    if distance > np.pi:
      distance = TAU - distance
      direction = direction * -1

    movement = delta*float(self.speed)

    if movement >= distance:
      # print("distance:{}\tmovement: {}".format(distance,movement))
      self.angle = self.goal
    else:
      self.angle = (self.angle + movement * direction) % TAU


class SimulatedArmDriver:
  def __init__(self,start_configuration):
    self.actuators = [
      BasicActuator("base0",start_configuration[0]),
      BasicActuator("link0",start_configuration[1]),
      BasicActuator("link1",start_configuration[2]),
      BasicActuator("link2",start_configuration[3]),
    ]
    self.configuration = start_configuration + [0]
    self.last_update = time.time()

  def update(self):
    delta = time.time() - self.last_update
    self.last_update = time.time()
    for actuator in self.actuators:
      actuator.update(delta)
    self.configuration = [act.angle for act in self.actuators] + [0]

  def actuate(self,goal_angles):
    self.last_update = time.time()
    for i in range(0,len(self.actuators)):
      self.actuators[i].actuate(goal_angles[i])

  def done(self):
    return all([act.done() for act in self.actuators])

  def build(self):
    # return a kinematic solver for this driver
    # TODO: expand this class to be more general
    return KinematicSolver()


class KinematicModel:
  def __init__(self,start_configuration,joint_order,precision=0.001,step_size=10):
    self.goal = np.array([[0,0,0]])
    self.new_goal = True
    self.end_affector = np.array([[0,0,0]])
    self.configuration = start_configuration
    self.joint_order = joint_order
    self.precision = precision
    self.step_size = step_size
    self.path = []
    self.driver = SimulatedArmDriver([start_configuration[name] for name in joint_order])
    self.solver = self.driver.build()

  def set_goal(self,goal):
    # grab only the first three elements of whatever we got
    self.goal = np.array(goal[0:3]).reshape(1,3)
    self.new_goal = True

  def request_configuration(self,configuration):
    self.driver.actuate(configuration)

  def reset(self):
    # request a good default configuration
    self.request_configuration([0,np.pi/2,-np.pi/2,-np.pi/2])


  def configured(self):
    return (np.sum(np.square(self.end_affector - self.goal)) < self.precision**2) and self.driver.done()

  def update(self):
    self.driver.update()
    # update configuration to reflect current position
    for i in range(0,len(self.joint_order)):
      self.configuration[self.joint_order[i]] = self.driver.configuration[i]
    self.end_affector = self.solver.ee_translation(self.driver.configuration)


    # check if goal has changed
    if self.new_goal:
      # get a path towards the goal
      self.path = [wp for wp in self.solver.generate_path_to_point(self.driver.configuration,self.goal)]
      self.path.reverse()
      # override current actuator action for new path
      if len(self.path) > 0:
        self.driver.actuate(self.path.pop())
    else:
      # if all actuators have resolved their movement, feed them the next point along current path
      if self.driver.done() and len(self.path) > 0:
        self.driver.actuate(self.path.pop()) # TODO: make sure this is the right data type
    self.new_goal = False # if we had a new goal, it has now been processed

  def __build_joint_list__(self):
    return [self.configuration[joint] for joint in joint_order]