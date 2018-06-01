from rover_ctl.msg import MotorCMD

LOW_DRILL_SPEED = -127
HIGH_DRILL_SPEED = -230
CLOSED_POS = 0
OPENED_POS = 255

isDrillOpen = False

def makeMsg(msg):
    l_trig = msg.axes[2]
    r_trig = msg.axes[5]

    a_button = msg.buttons[0]
    b_button = msg.buttons[1]
    x_button = msg.buttons[2]
    y_button = msg.buttons[3]
    # actuator
    actuator_speed = ((l_trig-1)/2 + (1-r_trig)/2) * 255
    # drill
    if x_button:
        drill_speed = LOW_DRILL_SPEED
    elif y_button:
        drill_speed = HIGH_DRILL_SPEED
    else:
        drill_speed = 0

    if a_button:
        isDrillOpen = True

    if b_button:
        isDrillOpen = False

    # sample servo
    if isDrillOpen:
        servo_pos = OPENED_POS
    else:
        servo_pos = CLOSED_POS

    drillCmd = MotorCMD()
    drillCmd.type = "drill"
    drillCmd.data = [int(actuator_speed), drill_speed, servo_pos]
    return drillCmd
