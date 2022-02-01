from DeltaArray import DeltaArray
import numpy as np
import time

da = DeltaArray('/dev/ttyACM0')

print(da.get_joint_positions())

da.reset()
da.wait_until_done_moving()
print(da.get_joint_positions())

for i in range(1,10):
	p = np.ones((1,12)) * 0.01 * i
	duration = [1.0]
	da.move_joint_position(p,duration)
	da.wait_until_done_moving()
	print(da.get_joint_positions())

da.reset()
da.wait_until_done_moving()
print(da.get_joint_positions())

da.close()