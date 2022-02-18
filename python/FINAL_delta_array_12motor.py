import linear_actuator_pb2
import numpy as np
from serial import Serial
from math import *
import time

NUM_MOTORS = 12

arduino = Serial('/dev/ttyACM0', 57600)  
delta_message = linear_actuator_pb2.lin_actuator()
delta_message.id = 1

class DeltaArray:
    def __init__(self, port, robot_id):
        self.arduino = Serial(port, 57600)
        self.delta_message = linear_actuator_pb2.lin_actuator()
        self.delta_message.id = robot_id
        self.min_joint_pos = 0.005
        self.max_joint_pos = 0.0988

        self.done_moving = False

    # GENERATE RESET and STOP commands in protobuf
    def reset(self):
        self.arduino.write()

    def stop(self):
        self.arduino.write()

    def move_joint_position(self, desired_joint_positions, robot_id):
        desired_joint_positions = np.clip(desired_joint_positions,self.minimum_joint_position,self.maximum_joint_position)
        # Add joint positions to delta_message protobuf
        _ = [delta_message.joint_pos.append(desired_joint_positions[i]) for i in range(12)]
        serialized = delta_message.SerializeToString()
        self.arduino.write(bytes(b'\xa6') + serialized + bytes(b'\xa7'))
        reachedPos = str(arduino.readline())
        while reachedPos[0]!="~": 
            print(reachedPos)
            reachedPos = str(arduino.readline().decode())
        delta_message.Clear()

    def close(self):
        self.arduino.close()

    def get_joint_positions(self):
        self.update_joint_positions_and_velocities()
        return self.current_joint_positions














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
