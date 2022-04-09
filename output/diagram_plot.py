from turtle import color
import numpy as np


import matplotlib.pyplot as plt
import numpy as np
from pyparsing import col


labels = ['0.2', '0.4', '0.6', '0.8', '1.0', '1.2','1.4', '1.6', '1.8', '2.0']
mpc_time = [1.5, 2.7, 3.4, 4.1, 4.6, 5.3, 5.9, 6.4, 6.9, 7.5]
pid_time = [2.3, 2.9, 3.5, 4., 4.6, 5.3, 5.8, 6.4, 7.1, 7.7]

x = np.arange(len(labels))  # the label locations
width = 0.35  # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(x - width/2, mpc_time, width, label='MPC', color = "red")
rects2 = ax.bar(x + width/2, pid_time, width, label='PID', color ="blue")

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Transient time, s')
ax.set_xlabel('Initial deviation, m')
ax.set_xticks(x, labels)
ax.legend()

ax.bar_label(rects1, padding=3)
ax.bar_label(rects2, padding=3)

fig.tight_layout()

plt.show()


