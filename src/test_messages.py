import serial
from messages import Message

serial_object = serial.Serial('/dev/ttyACM0',115200, timeout = 1)

Message(6, [20,0,0,0,0,0], serial_object)
