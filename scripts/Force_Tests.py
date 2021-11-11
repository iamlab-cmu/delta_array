import pyfirmata
import time
import config
from sklearn.linear_model import LinearRegression
import numpy as np

from DeltaArray import DeltaArray
from Model import NN

def retract(da):
    da.reset()
    da.wait_until_done_moving()

def adjust_act_command(command):
    #Nx12 trajectory to adjust in cm
    #Actuators 3:6 are calibrated with linear regression
    traj = np.zeros_like(command)

    traj[:,3] = command[:,3]/100*.95976 + .00211
    traj[:,4] = command[:,4]/100*.97326
    traj[:,5] = command[:,5]/100*.9819 + .0014

    return traj

class Load_Cell():
    def __init__(self):
        try:
            self.board = pyfirmata.Arduino(config.NANO_PORT)
        except:
            raise ValueError('USB Device not found')

        self.analog_input = self.board.get_pin('a:0:i')  # set A0 to input
        self.analog_input.enable_reporting()
        it = pyfirmata.util.Iterator(self.board)
        it.start()

    def readSensor(self,load_cell_no):
        # analogRead is (0,1023) but here it is (0,1)
        b = config.LOAD_CELL_PARAMS["b"+str(load_cell_no)]
        m = config.LOAD_CELL_PARAMS["m"+str(load_cell_no)]

        sensor_value = self.analog_input.read()
        if sensor_value is None:
            return 0.0
        return (sensor_value*m + b) 

    def readSensorRaw(self):
        sensor_value = self.analog_input.read()
        if sensor_value is None:
            return 0.0
        else:
            return sensor_value

    def calibrate(self):
        vec = []
        for i in range(1,6):
            input("Place " + str(i) +" kg on weight")
            vec.append(self.readSensorRaw())

        y = np.arange(1,6).reshape((5,1))
        x = np.array(vec).reshape((5,1))

        reg = LinearRegression().fit(x,y)

        m = reg.coef_[0,0]
        b = reg.intercept_[0]


        print("m = ",m)
        print("b = ",b)

        print("error =",y-m*x - b)




class Force_Testing():
    def __init__(self, model, da, filename="force_test_data.txt", overwrite=False):
        self.da = da
        self.model = model

        self.filename = filename

        if overwrite:
            self.start_ee_position = []
            self.command_ee_position = []

            self.start_act_position = []
            self.command_act_position = []

            self.force = []
        else:
            Data = np.load(filename)
            self.start_ee_position = list(Data["start_ee_position"])
            self.command_ee_position = list(Data["command_ee_position"])

            self.start_act_position = list(Data["start_act_position"])
            self.command_act_position = list(Data["command_act_position"])

            self.force = list(Data["force"])


        self.load_cells = {}
        self.load_cell_dirs = config.LOAD_CELL_DIRS
        self.load_cell = Load_Cell()
            
    def goto_position(self,pos,timeout=3):
        act_cmd = np.zeros((1,12))
        act_cmd[0,3:6] = pos
        act_cmd = adjust_act_command(act_cmd)
        self.da.move_joint_position(act_cmd, [1.])
        return self.da.wait_until_done_moving(timeout=timeout)
    
    def get_pose_traj(self,xval):
        traj = np.linspace([xval,-1,2.2],[xval,1,2.2],11)
        ik,valid_mask = self.model.predict_ik(traj)

        traj = traj[valid_mask]
        return traj

    def debug_loop(self):
        last_inp = "f1"
        while True:
            inp = input("c a0,a1,a2: Command Position (a0,a1,a2)\ne e0,e1,e2: Command ee Position\nf i: Get Force Reading for Load Cell i\nr: Retract\n")
            #inp = input("\n")
            try:
                if inp == "":
                    inp = last_inp

                if inp[0] == "c":
                    act_in = list(map(float,inp[1:].split(",")))
                    self.goto_position(act_in)

                elif inp[0] == "e":
                    ee = list(map(float,inp[1:].split(",")))
                    act_in = self.model.predict_ik(np.expand_dims(ee,0))
                    print("Moving to position ",act_in)
                    self.goto_position(act_in,timeout=1.5)

                elif inp[0] == "r":
                    retract(self.da)

                elif inp[0] == "f":
                    cell_ind = int(inp[1:])
                    if 0 <= cell_ind and cell_ind < 5:
                        print(self.load_cell.readSensor(cell_ind),"kg")
                    else: 
                        print("Not a valid load cell index")
                    last_inp = inp

                else:
                    print("Not a valid command")
            except:
                print("There was an error with your last command")
                
            
                

    def test_loop(self):
        retract(self.da)
        traj = self.get_pose_traj(2)

        ind = 0
        while ind < len(traj):
            start_act_position = []
            command_act_position = []

            start_ee_position = []
            command_ee_position = []

            force = []

            pt = traj[ind] 

            pred_pt = self.model.predict_ik(np.expand_dims(pt,0))
            act_cmd = np.zeros((1,12))
            act_cmd[0,3:6] = pred_pt
            act_cmd = adjust_act_command(act_cmd)
            self.da.move_joint_position(act_cmd, [1.])
            self.da.wait_until_done_moving()

            input("Moving Delta Under Load Cells")

            for i in range(1,4):
                lc = self.load_cell
                lcd = self.load_cell_dirs[i]

                pt0 = np.expand_dims(pt,0) + lcd*.5
                pred0 = self.model.predict_ik(pt0)

                #command calibrated actuator to position
                act_cmd = np.zeros((1,12))
                act_cmd[0,3:6] = pred0 
                act_cmd = adjust_act_command(act_cmd)
                self.da.move_joint_position(act_cmd, [1.])
                self.da.wait_until_done_moving()
                inp = input("Press Enter When Ready Or S To Skip")
                if inp == "s":
                    continue
                
                inc = 1
                last_read = lc.readSensor(i)
                first_read = last_read
                pred = pred0

                while True:
                    # move Delta in direction lcd until force stops increasing
                    dir = lcd*inc
                    pred = self.model.predict_ik(np.expand_dims(pt+dir,0),pred)

                    #command calibrated actuator to position
                    act_cmd = np.zeros((1,12))
                    act_cmd[0,3:6] = pred 
                    act_cmd = adjust_act_command(act_cmd)
                    self.da.move_joint_position(act_cmd, [1.])
                    succ = self.da.wait_until_done_moving(timeout=1)
                    # if not succ and not contact:
                    #     print("Contact Made")
                    #     pred_0_contact = pred
                    #     pt_contact = pt+dir
                    #     contact = True


                    time.sleep(config.LOAD_CELL_LAG)
                    force_read = lc.readSensor(i)
                    print("Force Read",force_read)

                    
                    start_act_position.append(pred0)
                    command_act_position.append(pred)

                    start_ee_position.append(pt)
                    command_ee_position.append(pt + dir)
                    
                    force.append((force_read-first_read)*lcd)

                    last_read = force_read
                    if inc >= 3:
                        break
                    inc += .5
            
            act_cmd = np.zeros((1,12))
            act_cmd[0,3:6] = pred_pt - np.min(pred_pt)
            act_cmd = adjust_act_command(act_cmd)
            self.da.move_joint_position(act_cmd, [1.])
            self.da.wait_until_done_moving()

            while True:
                ans = input("Are you satisfied with the data at this point? y/n/r/bp\n")
                if ans.lower() == "y":
                    self.start_act_position.extend(start_act_position)
                    self.start_ee_position.extend(start_ee_position)
                    self.command_act_position.extend(command_act_position)
                    self.command_ee_position.extend(command_ee_position)
                    self.force.extend(force)
                    ind += 1
                    break 
                elif ans.lower() == "n":
                    ind += 1
                    break
                elif ans.lower() == "bp":
                    breakpoint()
                elif ans.lower() == "r":
                    print("Taking Datapoint Again")
                    break
            

            np.savez(self.filename,start_act_position=self.start_act_position,
                start_ee_position=self.start_ee_position,
                command_act_position=self.command_act_position,
                command_ee_position=self.command_ee_position,
                force = force)


fk_save_file = "./models/training_data_flat_fk.h5"
ik_save_file = "./models/training_data_flat_ik.h5"
model = NN(ik_save_file,fk_save_file,
			ik_save_file,fk_save_file,load=True)

da = DeltaArray(config.ACTUATOR_ARDUINO_PORT)

f = Force_Testing(model,da,filename="force_test_data_pp.npz", overwrite=True)
f.debug_loop()
#f.test_loop()











