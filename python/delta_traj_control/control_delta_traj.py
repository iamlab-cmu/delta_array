import numpy as np
import matplotlib.pyplot as plt
import time

from pyparsing import delimitedList
from Prismatic_Delta import Prismatic_Delta
from get_coords import RoboCoords
import telnetlib
import delta_trajectory_pb2
from control_delta_arrays import DeltaArrayAgent

host = "192.168.0.182"
port = 80
timeout = 100

esp01 = telnetlib.Telnet(host, port, timeout)

DDA = DeltaArrayAgent(esp01, 1)
s_p = 1.5 #side length of the platform
s_b = 4.3 #side length of the base
l = 4.5 #length of leg attached to platform

Delta = Prismatic_Delta(s_p, s_b, l)

def create_joint_positions(val):
    a = []
    for i in range(4):
        for j in range(3):
            a.append(val[j])
    return a



if __name__=="__main__":
    # env = control_delta_arrays.DeltaArrayEnv("COM7")
    
    wave = 3*np.sin(np.linspace(-2*np.pi, 2*np.pi, 20))
    # plt.plot(wave)
    # plt.show()
    
    delta_message = delta_trajectory_pb2.DeltaMessage()
    jt_pos = delta_trajectory_pb2.JointPos()

    delta_message.id = 1
    delta_message.request_joint_pose = False
    delta_message.request_done_state = False
    delta_message.reset = False
    for i in range(10000):
        x = input("Go signal")
        # ee_pts = list(np.array(x.split(","), dtype=float))
        
        for j in range(20):
            ee_pts = [0,0,10+wave[j]]
            pts = Delta.IK(ee_pts)
            pts = np.array(pts) * 0.01
            pts = np.clip(pts,0.005,0.095)
            jts = create_joint_positions(pts)
            jt_pos.joint_pos.extend(jts)
            delta_message.trajectory.append(jt_pos)
            del jt_pos.joint_pos[:]

        serialized = delta_message.SerializeToString()
        for i in range(20):
            print(delta_message.trajectory[i])
        # print(serialized)
        # if b"\xa7~~" in serialized:
        #     print("HAKUNA")
            
        # if b"\xa6~~" in serialized:
        #     print("HAKUNA")
            
        esp01.write(b'\xa6~~' + serialized + b'\xa7~~\r\n')
        
        del delta_message.trajectory[:]
        print(delta_message)