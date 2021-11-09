import numpy as np

############### Actuator Config ####################
MAX_ACTUATOR_HEIGHT = .043 #m
MIN_ACTUATOR_HEIGHT = .005 #m

ACTUATOR_ARDUINO_PORT = "/dev/cu.usbmodem14201" 


##############  Neural Net Params ##################

BATCH_SIZE = 64
MEM_SIZE = 500000
TRAIN_PER_EP = 50
TRAINING_EPOCHS = 200

#Add random noise around initial guesses when training IK
IK_GUESS_STD = .2
IK_GUESSES_PER_SAMPLE = 10


############## Load Cell Params ###########
LOAD_CELL_PARAMS = {"m0":1.5043,"b0":-.2813,
	"m1":11.4562,"b1":.09495,
	"m2":11.691,"b2":.08828,
	"m3":11.99086,"b3":.09078,
	"m4":11.798,"b4":.10548}

# Map Load cell number to the direction of force the Delta is applying
# towards that load cell (from the Delta's frame of reference)
LOAD_CELL_DIRS = {
    0 : np.array([0,0,1]),
    1 : np.array([1,0,0]),
    2 : np.array([0,1,0]),
    3 : np.array([-1,0,0]),
    4 : np.array([0,-1,0])
}


NANO_PORT = "/dev/cu.usbserial-1410" 
INCREMENT = .1 #cm
LOAD_CELL_LAG = .25 #settling time for load cell in seconds
LOAD_CELL_HEIGHT = 1.9 #height above Delta