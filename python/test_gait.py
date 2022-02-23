import delta_array_pb2
import numpy as np
from serial import Serial
from math import *
import time
from Prismatic_Delta import Prismatic_Delta

NUM_MOTORS = 12
delta_message = delta_array_pb2.DeltaMessage()
arduino = Serial('/dev/tty0', 57600)

def create_joint_positions(val):
    for i in range(12):
        if i<NUM_MOTORS:
            delta_message.joint_pos.append(val)
        else:
            delta_message.joint_pos.append(0.0)

while True:
    x = input("Position in cm? ")
    create_joint_positions(float(x)/100)

    print(delta_message)

    serialized = delta_message.SerializeToString()
    arduino.write(bytes(b'\xa6')  + serialized + bytes(b'\xa7'))
    reachedPos = str(arduino.readline())
    while reachedPos[0]!="~": 
        print(reachedPos)
        reachedPos = str(arduino.readline().decode())
    print(reachedPos)
    delta_message.Clear()