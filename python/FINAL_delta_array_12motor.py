from audioop import alaw2lin
import delta_array_pb2
import numpy as np
from serial import Serial
from math import *
import time
from Prismatic_Delta import Prismatic_Delta

NUM_MOTORS = 12
NUM_AGENTS = [1,3,4,5]
s_p = 1.5 #side length of the platform
s_b = 4.3 #side length of the base
l = 4.5 #length of leg attached to platform

Delta = Prismatic_Delta(s_p, s_b, l)

# arduino = Serial('/dev/ttyACM0', 57600)
# arduino = Serial('COM4', 57600)
# delta_message = linear_actuator_pb2.lin_actuator()
# delta_message.id = 1

class DeltaArrayAgent:
    def __init__(self, ser, robot_id):
        self.arduino = ser
        self.delta_message = delta_array_pb2.DeltaMessage()
        self.delta_message.id = robot_id
        self.delta_message.request_joint_pose = False
        self.delta_message.request_done_state = False
        self.delta_message.reset = False
        self.min_joint_pos = 0.005
        self.max_joint_pos = 0.0988

        self.done_moving = False
        self.current_joint_positions = [0.05]*12

    # GENERATE RESET and STOP commands in protobuf
    def reset(self):
        self.arduino.write()

    def stop(self):
        self.arduino.write()

    # def proto_clear(self):
    #     self.delta_message.Clear()
    #     self.delta_message.id = self.delta_message.id
    #     self.delta_message.request_joint_pose = self.delta_message.request_joint_pose
    #     self.delta_message.request_done_state = self.delta_message.request_done_state
    #     self.delta_message.reset = self.delta_message.reset


    def send_proto_cmd(self, ret_expected = False):
        serialized = self.delta_message.SerializeToString()
        self.arduino.write(bytes(b'\xa6') + serialized + bytes(b'\xa7'))
        if ret_expected:
            reachedPos = str(self.arduino.readline())
            # print(reachedPos.split(" "))
            reachedPos = reachedPos.strip().split(" ")
            if self.delta_message.id == int(reachedPos[0].split(':')[-1]):
                return [float(x) for x in reachedPos[1:-1]]
            else:
                print("ERROR, incorrect robot ID requested.")
                return [0.05]*12


    def move_joint_position(self, desired_joint_positions):
        desired_joint_positions = np.clip(desired_joint_positions,self.min_joint_pos,self.max_joint_pos)
        # Add joint positions to delta_message protobuf
        _ = [self.delta_message.joint_pos.append(desired_joint_positions[i]) for i in range(12)]
        self.send_proto_cmd()
        print(self.delta_message)
        del self.delta_message.joint_pos[:]

    def close(self):
        self.arduino.close()

    def get_joint_positions(self):
        _ = [self.delta_message.joint_pos.append(0.5) for i in range(12)]
        self.delta_message.request_done_state = True
        self.current_joint_positions = self.send_proto_cmd(True)
        del self.delta_message.joint_pos[:]
        self.delta_message.request_joint_pose = False
        self.delta_message.request_done_state = False
        self.delta_message.reset = False
        # print(self.current_joint_positions)


class DeltaArrayEnv:
    def __init__(self, port):
        self.ser = Serial(port, 57600)
        self.agents = {}
        for i in NUM_AGENTS:
            self.agents[i] = DeltaArrayAgent(self.ser, i)
        self.done_states = np.array([0]*12)
        self.generate_gaits()

    def move_over_trajectory(self, traj = "verticle"):
        if traj == "circle":
            thetas = np.linspace(0, 2*np.pi, 10)
            r = 1
            for theta in thetas:
                # for z in zvals:
                ee_pts = [r*np.cos(theta), r*np.sin(theta), 10.0]
                pts = Delta.IK(ee_pts)
                pts = np.array(pts) * 0.01
                # print(theta)
                jts = []
                for i in range(0, 4):
                    for j in range(3):
                        jts.append(pts[j])
                
                for i in NUM_AGENTS:
                    self.agents[i].move_joint_position(jts)

        elif traj=="vertical":
            # pts = Delta.IK([0,0,10])
            pts1 = [0.05,0.05,0.05]
            pts2 = [0.02,0.02,0.02]
            jts = []
            for i in range(0, 4):
                if i==3:
                    jts.extend(pts1)
                else:
                    jts.extend(pts2)
                
            # print(jts)
            for i in NUM_AGENTS:
                self.agents[i].move_joint_position(jts)
                
            for i in NUM_AGENTS:
                self.agents[i].get_joint_positions()

    def generate_gaits(self):
        upstep = Delta.IK([0,0,11])
        upstep = np.array(upstep) * 0.01

        meetstep_l = Delta.IK([0.5,0.5,11])
        meetstep_l = np.array(meetstep_l) * 0.01
        
        meetstep_r = Delta.IK([-0.5,-0.5,11])
        meetstep_r = np.array(meetstep_r) * 0.01
        
        downstep = Delta.IK([0,0,9])
        downstep = np.array(downstep) * 0.01
        
        alt_gait1 = np.array([[downstep, downstep],[upstep, upstep]]) #.|.|
        alt_gait2 = np.array([[meetstep_l, meetstep_l],[meetstep_r, meetstep_r]]) # /\
        alt_gait3 = np.array([[upstep, upstep],[downstep, downstep]]) #|.|.
        alt_gait4 = np.array([[meetstep_r, meetstep_r],[meetstep_l, meetstep_l]]) #\/

        # Gait from Right to Left
        right_gait = [alt_gait1, alt_gait2, alt_gait3, alt_gait4]
        self.right_gait_joints = []
        for gait in right_gait:
            jts = []
            for i in gait:
                for j in i:
                    jts.extend(j)
            self.right_gait_joints.append(jts)

        left_gait = [alt_gait3, alt_gait2, alt_gait1, alt_gait4]
        self.left_gait_joints = []
        for gait in left_gait:
            jts = []
            for i in gait:
                for j in i:
                    jts.extend(j)
            self.left_gait_joints.append(jts)
        
    def planar_translation(self):
        for gait in self.left_gait_joints:
            print(gait)
            for i in NUM_AGENTS:
                print(i)
                self.agents[i].move_joint_position(gait)
                time.sleep(0.5)
            for i in NUM_AGENTS:
                self.agents[i].get_joint_positions()
            time.sleep(0.3)

if __name__=="__main__":
    env = DeltaArrayEnv("COM7")
    # env = DeltaArrayEnv("/dev/tty7")
    # env.move_over_trajectory("vertical")
    env.planar_translation()