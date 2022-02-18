import linear_actuator_pb2
import numpy as np
from serial import Serial
from math import *
import time
import matplotlib.pyplot as plt
NUM_MOTORS = 12

arduino = Serial('/dev/ttyACM0', 57600)  


delta_message = linear_actuator_pb2.lin_actuator()
delta_message.id = 1



def f(x):
    return np.sin(x) + np.random.normal(scale=0.1, size=len(x))


x = []

for i in np.linspace(0, 2*np.pi, 24):
    x.append(5 + 4*np.sin(i))


def create_joint_positions(val):
    for i in range(12):
        if i<NUM_MOTORS:
            delta_message.joint_pos.append(val[i]/100)
        else:
            delta_message.joint_pos.append(0.0)

def rotate(l, n):
    return l[n:] + l[:n]

a = 0
while True:
    y = rotate(x, a)
    
    # plt.plot(y)
    # plt.show()
    a +=1 
    a = a%24

    # x = input("Position in cm? ")
    create_joint_positions(y)

#     print(delta_message)

    serialized = delta_message.SerializeToString()
    arduino.write(bytes('<', 'utf-8') + serialized + bytes('>', 'utf-8'))
    reachedPos = str(arduino.readline())
    while reachedPos[0]!="~": 
        print(reachedPos)
        reachedPos = str(arduino.readline().decode())
    print("reachedPos")
    delta_message.Clear()
    time.sleep(1)
