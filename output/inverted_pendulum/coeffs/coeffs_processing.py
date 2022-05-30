#!/usr/bin/python
# -*- coding: utf-8 -*

from cProfile import label
import matplotlib.pyplot as plt
import matplotlib as mtp
import numpy as np
from math import cos, sin, sqrt,atan2, pi


n_mpc_coords = np.zeros([0,5],dtype=float)
l_mpc_coords = np.zeros([0,5],dtype=float)

num = "6"
file_name = ("nonlinear_exp_" + num + ".txt")

f = open(file_name)
coeffs = f.readline()
coeffs = coeffs.split()
print(coeffs)
#next(f)
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


names = ("x, м", "V, м/c", "\u03F4, Рад", "\u03C9, Рад/c", "Т, Н\u00D7м")
plt.rcParams["figure.figsize"] = (7,8)

fig, axs = plt.subplots(5)
iter_range = 400
time = np.arange(iter_range,dtype=float) * 0.025
for i in range(5):
    axs[i].plot(time,n_mpc_coords[:iter_range,i],"r",linewidth=2., label = "NMPC")
    axs[i].plot(time,l_mpc_coords[:iter_range,i],"b",linewidth=2., label = "LMPC")
    axs[i].set(xlabel= "t,c", ylabel=names[i])
    axs[i].yaxis.set_major_formatter(mtp.ticker.FormatStrFormatter('%.2f'))
    #axs[i].legend()
title_params = ("$q_1$", "$q_2$", "$q_3$", "$q_4$", "$r_1$", "$r_2$")
title = ""
for i in range(5):
    title +=str(title_params[i])+" = " + str(coeffs[i]) + ", "


axs[0].set_title(title)
fig.legend(["NPMC","LMPC","LQR"])

plt.show()
