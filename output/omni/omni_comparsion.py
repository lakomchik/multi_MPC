from cProfile import label
import matplotlib.pyplot as plt
import numpy as np
from math import cos, sin, sqrt,atan2, pi



mpc_coords = np.zeros([0,3],dtype=float)
pid_coords = np.zeros([0,3],dtype=float)
ref_coords = np.zeros([0,3],dtype=float)

def get_ref_coords(iter):
    v = 0.1
    phase = iter*pi/200
    ref_coords = np.zeros([3],dtype=float)
    ref_speed = np.zeros([3],dtype= float)
    cur_x = cos(phase)
    cur_y = sin(phase)
    theta = 6*float(phase)%(2*pi)
    print(theta)
    if theta > pi:
        theta = theta - 2*pi
   
    ref_coords[0] = cur_x
    ref_coords[1] = cur_y
    ref_coords[2] = theta
    print("REF VEC:")
   
    return ref_coords, ref_speed




iter_num = 400
for i in range(iter_num):
    ref_step_coords, ref_speed = get_ref_coords(i)
    ref_coords = np.append(ref_coords, [ref_step_coords], axis=0)


file_name = ("omni_mpc_4.txt")

f = open(file_name)
for line in f:
    stroka = line.split()
    cur_coords = np.asarray(stroka,dtype=float)
    mpc_coords = np.append(mpc_coords, [cur_coords], axis=0)

file_name = ("omni_pid_4.txt")

f = open(file_name)
for line in f:
    stroka = line.split()
    cur_coords = np.asarray(stroka,dtype=float)
    pid_coords = np.append(pid_coords, [cur_coords], axis=0)

sko = np.sqrt(np.sum(np.square(mpc_coords[:iter_num,:]-ref_coords),axis=0)/(ref_coords.shape[0]+1))
coeffs = file_name.split()
print('Srednekvadratichnoe otklonenie = '+str(sko))
print(coeffs)
plt.rcParams["figure.figsize"] = (7,10)
plot1 = plt.subplot2grid((4, 1), (0, 0), rowspan=3)
plot2 = plt.subplot2grid((4, 1), (3, 0), rowspan=1)
plot1.plot(ref_coords[:,0],ref_coords[:,1],"black",linewidth = 2, label = "Reference path")
plot1.plot(pid_coords[1:iter_num+1,0],pid_coords[1:iter_num+1,1],"b--",linewidth=3.5, label = "PID")
plot1.plot(mpc_coords[1:iter_num+1,0],mpc_coords[1:iter_num+1,1],"r--",linewidth=3.5, label = "MPC")
#print(gripper_coords[:,2])
plot1.axis("equal")
plot1.legend()
plot1.set_xlabel("X, M")
plot1.set_ylabel("Y, M")
time = np.arange(0,iter_num*0.1,0.1, dtype=float)
plot2.plot(time,ref_coords[:iter_num,2],"black", linewidth = 2, label = "Reference \u03F4")
plot2.plot(time,pid_coords[1:iter_num+1,2],"b--", linewidth = 3.5, label = "PID \u03F4")
plot2.plot(time,mpc_coords[1:iter_num+1,2],"r--", linewidth = 3.5, label = "MPC \u03F4")
plot2.legend()
plot2.set_xlabel("t, c")
plot2.set_ylabel("\u03F4, Rad")
plt.tight_layout()

plt.show()
