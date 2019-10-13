from tkinter import *
import tkinter as tk
import serial, time
import messages

buffer_count = 0
CLEAR_BUFFER = 2

ser = serial.Serial("/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0", 115200)
time.sleep(1)

root = tk.Tk()
maxs = 210

v = Scale(root, from_=-maxs, to=maxs)
v.set(0)
v.pack()
h = Scale(root, from_=-maxs, to=maxs)
h.set(0)
h.pack()

def make_msg(speeds):
    pass

def proc():
    global buffer_count
    speeds = [v.get(), 0, 0, h.get(), 0, 0]
    mess = messages.Message(messages.TARGET_SYSTEMS.DRIVE, speeds)
    #  cmd = "#0#{}\n".format(",".join([str(s) for s in speeds]))
    #  print(cmd)
    #  ser.write(bytes(cmd, "UTF-8"))
    ser.write(mess.serialize())
    #  print(ser.readline(ser.inWaiting()))
    while ser.in_waiting:
        print(ser.readline())
    if buffer_count > CLEAR_BUFFER:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    buffer_count += 1
    root.after(200, proc)

root.after(200, proc)
root.mainloop()
