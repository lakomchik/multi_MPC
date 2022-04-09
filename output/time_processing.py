from cProfile import label
from traceback import print_tb
import matplotlib.pyplot as plt
import numpy as np
from math import cos, fabs, sin, sqrt,atan2



mpc_coords = np.zeros([0,3],dtype=float)
pid_coords = np.zeros([0,3],dtype=float)
ref_coords = np.zeros([0,3],dtype=float)

def get_ref_coords(iter):
    v = 0.1
    ds = 0.006
    c = 2.

    s = iter*ds
    ref_coords = np.zeros([3],dtype=float)
    ref_speed = np.zeros([2],dtype= float)
    cur_x = c * sqrt(2) * cos(s)/(1 + pow(sin(s),2))
    cur_y = c * sqrt(2) * sin(s) * cos(s)/(1 + pow(sin(s),2))
    prev_x = c * sqrt(2) * cos(s-ds)/(1 + pow(sin(s-ds),2))
    prev_y = c * sqrt(2) * sin(s-ds) * cos(s-ds)/(1 + pow(sin(s-ds),2))
    theta = atan2(cur_y-prev_y,cur_x-prev_x)
    v_ref = sqrt( pow(cur_y-prev_y,2) + pow(cur_x-prev_x,2))*10
    ref_coords[0] = cur_x
    ref_coords[1] = cur_y
    ref_coords[2] = theta
    ref_speed[0] = v_ref
    ref_speed[1] = 0

    return ref_coords, ref_speed




iter_num = 200
for i in range(iter_num):
    ref_step_coords, ref_speed = get_ref_coords(i)
    ref_coords = np.append(ref_coords, [ref_step_coords], axis=0)


file_name = ("output.txt")

f = open(file_name)
for line in f:
    stroka = line.split()
    cur_coords = np.asarray(stroka,dtype=float)
    mpc_coords = np.append(mpc_coords, [cur_coords], axis=0)

file_name = ("pid_outpur.txt")

f = open(file_name)
for line in f:
    stroka = line.split()
    cur_coords = np.asarray(stroka,dtype=float)
    pid_coords = np.append(pid_coords, [cur_coords], axis=0)

pid_coords = np.delete(pid_coords,0,0)
print(pid_coords[0,:])
print(mpc_coords[0,:])
print(ref_coords[0])
pid_coords = pid_coords[:iter_num,:]
mpc_coords = mpc_coords[:iter_num,:]

delta_mpc = np.sqrt(np.sum(np.square(mpc_coords[:,:1] - ref_coords[:,:1] ),axis=1))
delta_pid = np.sqrt(np.sum(np.square(pid_coords[:,:1]  - ref_coords[:,:1] ),axis= 1))

max_mpc_deviation = delta_mpc[0]
max_pid_deviation = delta_pid[0]
mpc_iter =0
delta_iter = 0
for i in range(iter_num):
    if(delta_mpc[i] < 0.05 * max_mpc_deviation):
        mpc_iter = i
        break

for i in range(iter_num):
    if(delta_pid[i] < 0.05 * max_pid_deviation):
        pid_iter = i
        break

print("With max deviation "+str(max_mpc_deviation) + " m, MPC time: " + str(mpc_iter*0.1) + "  PID time: " + str(pid_iter*0.1))


'''plt.plot(ref_coords[:,0],ref_coords[:,1],"black",linewidth = 2, label = "Reference path")
plt.plot(pid_coords[:iter_num,0],pid_coords[:iter_num,1],"b--",linewidth=3.5,label = "PID controller")
plt.plot(mpc_coords[:iter_num,0],mpc_coords[:iter_num,1],"r--",linewidth=3.5, label = "MPC controller")
#print(gripper_coords[:,2])
plt.axis("equal")
plt.legend()
plt.xlabel("X, m")
plt.ylabel("Y, m")
plt.show()'''