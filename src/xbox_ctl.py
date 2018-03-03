#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from serial import Serial
import time, math

#addr = "00:14:03:05:F1:F5"
MSG_RATE  = 15 #in Hz
MSG_PER = 1./MSG_RATE
last_message_send = 0
MAX_TURNING_RADIUS = 10

# Open serial. DO NOT REMOVE DELAY
#s = Serial("/dev/ttyACM0", 9600)
s = Serial("/dev/ttyACM0", 19200)
time.sleep(1)

class Motor:
    def __init__(self, id, x, y, maxSpeed, isFlipped):
        self.id = id
        self.x = x
        self.y = y
        self.maxSpeed = maxSpeed
        self.isFlipped = isFlipped

    def getTurningSpeed(self, turningRadius, turningSpeed):
        #return turningSpeed * (turningRadius - self.x) / (self.x**2+self.y**2)
        sign = 1 if self.x > 0 else -1
        return -sign*turningRadius*turningSpeed

    def getMotorSpeed(self, turningRadius, turningSpeed, maxTurningSpeed, forwardVel):
        maxTurningSpeed = 1 if maxTurningSpeed == 0 else maxTurningSpeed
        return forwardVel + self.getTurningSpeed(turningRadius, turningSpeed)

drivetrain = []
drivetrain.append(Motor(0,-0.4218, 0.53975, 255, False))
drivetrain.append(Motor(1,-0.4218, 0, 255, False))
drivetrain.append(Motor(2,-0.4080, -0.381, 255, False))
drivetrain.append(Motor(3,0.4080, -0.381, 255, False))
drivetrain.append(Motor(4,0.4218, 0, 255, False))
drivetrain.append(Motor(5,0.4218, 0.53975, 255, False))

wheelBaseWidth = 0.50

def callback(msg):
    global last_message_send
    if time.time() - last_message_send < MSG_PER:
        return
    last_message_send = time.time()

    turningRadius = -msg.axes[0]
    sign = -1 if turningRadius < 0 else 1
    #turningRadius = sign*(MAX_TURNING_RADIUS*(1-abs(turningRadius))+wheelBaseWidth)

    forwardVel = 255*msg.axes[1]

    #vel_mag = math.sqrt(msg.axes[0]**2 + msg.axes[1]**2)
    vel_mag = max(abs(msg.axes[0]), abs(msg.axes[1]))
    turningSpeed = (255*vel_mag-abs(forwardVel))
    if turningSpeed > 0:
        turningSpeed = turningSpeed - turningSpeed/4

    # Send abstract commands
    """
    msg = b'%f,%f,%f,\n' % (turningRadius, turningSpeed, forwardVel)
    print(msg)
    s.write(msg)
    """
    # Send raw commands
    msg = b'{},' * len(drivetrain) + b'\n'
    turningSpeeds = [0] * len(drivetrain)
    for i in range(len(drivetrain)):
        turningSpeeds[i] = abs(drivetrain[i].getTurningSpeed(turningRadius, turningSpeed))

    maxTurningSpeed = max(turningSpeeds)
    speeds = [0] * len(drivetrain)
    for i in range(len(drivetrain)):
        speeds[i] = int(drivetrain[i].getMotorSpeed(turningRadius, turningSpeed, maxTurningSpeed, forwardVel))
    msg = msg.format(*speeds)
    print(msg)
    s.write(msg)
    #print(s.readline())

def init():
    rospy.init_node("xbox_ctl", anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()
    s.close()

if __name__ == "__main__":
    init()
