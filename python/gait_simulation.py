import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go

delta_array = np.array([[1,0,1,0,1,0,1,0],
                        [1,0,1,0,1,0,1,0],
                        [1,0,1,0,1,0,1,0],
                        [1,0,1,0,1,0,1,0],
                        [1,0,1,0,1,0,1,0],
                        [1,0,1,0,1,0,1,0],
                        [1,0,1,0,1,0,1,0],
                        [1,0,1,0,1,0,1,0]])

# print(delta_array[:,1::2])
# print(delta_array[:,0::2])

def plot_array(del_arr):
    plt.figure(figsize = (7,7))
    for i in range(1, del_arr.shape[0] + 1):
        for j in range(1, del_arr.shape[1] + 1):
            val = del_arr[j-1, i-1]
            if val==1:
                plt.scatter(i, j+0.25, c="red")
            elif val==0:
                plt.scatter(i,j-0.25,c="blue")
            else:
                plt.scatter(i + val,j + val,c="orange")
    plt.show()


for i in range(1):
    plot_array(delta_array)
    delta_array = np.where(delta_array == 0, -0.48, delta_array)
    delta_array = np.where(delta_array == 1, 0.52, delta_array)
    # print(delta_array)
    plot_array(delta_array)
    
    # delta_array = np.where(delta_array == 0, 0.48, delta_array)
    # delta_array = np.where(delta_array == 1, 0.52, delta_array)
    # print(delta_array)