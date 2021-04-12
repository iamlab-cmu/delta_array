#!/usr/bin/env python

import time
import serial
import math
import numpy as np

class DeltaArray:
    def __init__(self, port):

        self.ser = serial.Serial(port, 57600)  # open serial port

        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.minimum_joint_position = 0.0015
        self.maximum_joint_position = 0.0988
        
        time.sleep(1.0)

    def reset(self):
        self.ser.write(b'<r>')
    
    def stop(self):
        self.ser.write(b'<s,1>')

    def start(self):
        self.ser.write(b'<s,0>')

    def move_joint_position(self, desired_joint_positions, durations):

        np.clip(desired_joint_positions,self.minimum_joint_position,self.maximum_joint_position)

        combined_array = np.hstack((np.array(durations).reshape(-1,1),np.array(desired_joint_positions)))
        num_trajectory_points = combined_array.shape[0]
        compressed_array = np.float32(combined_array.flatten())
        msg = '<p,' + str(num_trajectory_points) + ','
        for number in compressed_array:
            msg += str(round(number, 4)) + ','
        msg = msg[:-1] + '>'
        num_messages = int(np.ceil(len(msg) / 60.0))

        for i in range(num_messages):
            self.ser.write(msg[i*60:(i+1)*60].encode())
            time.sleep(0.1)

    def move_joint_velocity(self, desired_joint_velocities, durations):

        combined_array = np.hstack((np.array(durations).reshape(-1,1),np.array(desired_joint_velocities)))
        num_trajectory_points = combined_array.shape[0]
        compressed_array = np.float32(combined_array.flatten())
        
        msg = '<v,' + str(num_trajectory_points) + ','
        for number in compressed_array:
            msg += str(round(number, 9)) + ','
        msg = msg[:-1] + '>'
        num_messages = int(np.ceil(len(msg) / 60.0))

        for i in range(num_messages):
            self.ser.write(msg[i*60:(i+1)*60].encode())
            time.sleep(0.1)

    def close(self):
        self.ser.close()

    def update_joint_positions_and_velocities(self):
        while True:
            line = self.ser.readline()
            try:
                stringline = bytes.decode(line)
                if stringline[0] == 'j':
                    numbers = np.array(stringline[2:].strip().split(','))
                    for i in range(12):
                        self.current_joint_positions[i] = numbers[i*2]
                        self.current_joint_velocities[i] = numbers[i*2+1]
                    return False
                elif stringline[0] == 'd':
                    return True
            except:
                pass # Readline did not return a valid string.


    def get_joint_positions(self):
        self.ser.flushInput()
        self.update_joint_positions_and_velocities()
        return self.current_joint_positions

    def get_joint_velocities(self):
        self.ser.flushInput()
        self.update_joint_positions_and_velocities()
        return self.current_joint_velocities

    def wait_until_done_moving(self, timeout = 6):
        done_moving = False
        start_time = time.time()
        elapsed_time = 0.0

        while not done_moving and elapsed_time < timeout:
            done_moving = self.update_joint_positions_and_velocities()
            elapsed_time = time.time() - start_time

            