import pyfirmata
import time
import config
from sklearn.linear_model import LinearRegression
import numpy as np

from DeltaArray import DeltaArray
from Model import NN

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
        weights = np.array([0,.1,.2,.3,.5])
        for weight in weights:
            input("Place " + str(weight) +" kg on weight")
            vec.append(self.readSensorRaw())

        y = weights.reshape((len(weights),1))
        x = np.array(vec).reshape((len(weights),1))

        reg = LinearRegression().fit(x,y)

        m = reg.coef_[0,0]
        b = reg.intercept_[0]


        print("m = ",m)
        print("b = ",b)

        print("error =",y-m*x - b)
        print("Mean Error:",np.mean(y-m*x - b))
        print("Max Error:",np.max(y-m*x - b))
    
    def test_calibration(self,cell_no):
        vec = []
        weights = np.array([0,.1,.2,.3,.5])
        for weight in weights:
            input("Place " + str(weight) +" kg on weight")
            vec.append(self.readSensor(cell_no))

        y = weights.reshape((len(weights),1))
        x = np.array(vec).reshape((len(weights),1))

        print("error =",y-x)
        print("Mean Error:",np.mean(y-x))
        print("Max Error:",np.max(y-x))




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

            self.forces = []
        else:
            Data = np.load(filename)
            self.start_ee_position = list(Data["start_ee_position"])
            self.command_ee_position = list(Data["command_ee_position"])

            self.start_act_position = list(Data["start_act_position"])
            self.command_act_position = list(Data["command_act_position"])

            self.forces = list(Data["forces"])


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
        z = config.TEST_HEIGHT
        traj = np.linspace([xval,-4,z],[xval,4,z],9)
        #traj = np.array([[0,0,z]])
        ik,valid_mask = self.model.predict_ik(traj,return_valid_mask=True)

        traj = traj[valid_mask]
        return traj

    def retract(self):
        self.goto_position(config.RETRACT_POS)

    def debug_loop(self):
        last_inp = "f1"
        while True:
            inp = input("c a0,a1,a2: Command Position (a0,a1,a2)\ne e0,e1,e2: Command ee Position\nf i: Get Force Reading for Load Cell i\nr: Retract\n"+"bp: Breakpoint\n")
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
                    self.goto_position([.75,.75,.75])

                elif inp[0] == "f":
                    cell_ind = int(inp[1:])
                    if 0 <= cell_ind and cell_ind < 5:
                        print(self.load_cell.readSensor(cell_ind),"kg")
                    else: 
                        print("Not a valid load cell index")
                    last_inp = inp
                elif inp == "bp":
                    breakpoint()
                else:
                    print("Not a valid command")
            except:
                print("There was an error with your last command")
                
            
                

    def test_loop(self,xval):
        self.retract()
        traj = self.get_pose_traj(xval)

        ind = 0
        while ind < len(traj):
            start_act_position = []
            command_act_position = []

            start_ee_position = []
            command_ee_position = []

            force = []

            center_pt = np.expand_dims(traj[ind],0)
            print("Center Point:",center_pt)

            center_pt_ik = self.model.predict_ik(center_pt)
            d = config.TEST_HEIGHT-config.REST_HEIGHT
            self.goto_position(center_pt_ik-[d,d,d])

            input("Move Load Cells Over Delta")
            self.goto_position(center_pt_ik)

            i = 0
            while i < 1:
                lc = self.load_cell
                lcd = self.load_cell_dirs[i]

                pt0 = center_pt + lcd*config.DIST2CELL
                pt0_ik = self.model.predict_ik(pt0)

                self.goto_position(pt0_ik)

                inc = 0
                while True:
                    inc += config.TEST_INC
                    test_pt = pt0 + lcd*inc
                    test_pt_ik = self.model.predict_ik(test_pt)
                    self.goto_position(test_pt_ik,timeout=1)

                    stop = False
                    redo = False
                    while True:
                        inp = input("k:keep going, l:stop,r:redo")
                        if inp == "k":
                            break
                        elif inp == "l":
                            stop = True
                            break
                        elif inp == "r":
                            redo = True
                            break
                    if redo:
                        break    
                    
                    if stop:
                        i += 1
                        break

                    time.sleep(config.LOAD_CELL_LAG)
                    force_read = lc.readSensor(i)
                    print("Force Read:",force_read)

                    start_act_position.append(pt0_ik)
                    command_act_position.append(test_pt_ik)

                    start_ee_position.append(pt0)
                    command_ee_position.append(test_pt)
                    
                    force.append(force_read*lcd)
                
                succ = self.goto_position(pt0_ik,timeout=5)
                if not succ:
                    print("Failed to Return")
                time.sleep(5)
            
            self.goto_position(self.goto_position(center_pt_ik-[d,d,d]))

            while True:
                ans = input("Are you satisfied with the data at this point? y/n/c/bp\n")
                if ans.lower() == "y":
                    self.start_act_position.extend(start_act_position)
                    self.start_ee_position.extend(start_ee_position)
                    self.command_act_position.extend(command_act_position)
                    self.command_ee_position.extend(command_ee_position)
                    self.forces.extend(force)
                    ind += 1
                    break 
                elif ans.lower() == "n":
                    print("Taking Datapoint Again")
                    break
                elif ans.lower() == "bp":
                    breakpoint()
                    
            np.savez(self.filename,start_act_position=self.start_act_position,
                start_ee_position=self.start_ee_position,
                command_act_position=self.command_act_position,
                command_ee_position=self.command_ee_position,
                forces = self.forces)


fk_save_file = "./models/training_data_flat_fk.h5"
ik_save_file = "./models/training_data_flat_ik.h5"
model = NN(ik_save_file,fk_save_file,
			ik_save_file,fk_save_file,load=True)

da = DeltaArray(config.ACTUATOR_ARDUINO_PORT)

f = Force_Testing(model,da,filename="force_test_1_data_tpu_z.npz", overwrite=False)
f.debug_loop()
#f.test_loop(4)












