#!/usr/bin/python
# -*- coding: utf-8 -*

from cProfile import label
import matplotlib.pyplot as plt
import matplotlib as mtp
import numpy as np
from math import cos, sin, sqrt,atan2, pi


n_mpc_coords = np.zeros([0,5],dtype=float)
l_mpc_coords = np.zeros([0,5],dtype=float)
lqr_coords = np.zeros([0,5],dtype=float)


num = "3"
file_name = ("nonlinear_exp_" + num + ".txt")
f = open(file_name)
next(f)
for line in f:
    stroka = line.split()
    cur_coords = np.asarray(stroka,dtype=float)
    n_mpc_coords = np.append(n_mpc_coords, [cur_coords], axis=0)

file_name = ("linear_exp_" + num + ".txt")
f = open(file_name)
next(f)
for line in f:
    stroka = line.split()
    cur_coords = np.asarray(stroka,dtype=float)
    l_mpc_coords = np.append(l_mpc_coords, [cur_coords], axis=0)
file_name = ("lqr_exp_" + num + ".txt")
f = open(file_name)
next(f)
for line in f:
    stroka = line.split()
    cur_coords = np.asarray(stroka,dtype=float)
    lqr_coords = np.append(lqr_coords, [cur_coords], axis=0)
lqr_coords[:,0:2] = - lqr_coords[:,0:2]



names = ("x, м", "V, м/c", "\u03F4, Рад", "\u03C9, Рад/c", "Т, Н\u00D7м")

plt.rcParams["figure.figsize"] = (7,12)
fig, axs = plt.subplots(5)
iter_range = 400
time = np.arange(iter_range,dtype=float) * 0.025
for i in range(5):
    axs[i].plot(time,lqr_coords[1:iter_range+1,i],"g",linewidth=2., label = "LQR")
    axs[i].plot(time,n_mpc_coords[:iter_range,i],"r",linewidth=2., label = "NMPC")
    axs[i].plot(time,l_mpc_coords[:iter_range,i],"b",linewidth=2., label = "LMPC")
    
    axs[i].set(xlabel= "t,c", ylabel=names[i])
    axs[i].yaxis.set_major_formatter(mtp.ticker.FormatStrFormatter('%.2f'))
    #axs[i].legend()
axs[0].set_title(r'$\theta_0$' + " = -0.2 рад")
fig.legend(["LQR","NPMC","LMPC"])


plt.show()
