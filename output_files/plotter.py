import os, sys
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib import rc

plt.rcParams["grid.color"] = "lightgray"
plt.rcParams["grid.linewidth"] = 0.5
matplotlib.rc("font", family="serif", size=9)
matplotlib.rcParams["text.usetex"] = True
rc('text', usetex=True)

# Read data from the .dat file
smaple_time = 0.002
model_fileName = "sim_output.csv"
# target = np.genfromtxt("input_files/"  + target_fileName, delimiter=",", dtype=float, skip_header=1)
# data = np.genfromtxt("output_files/" + model_fileName, delimiter=",", dtype=float, skip_header=1)
df = pd.read_csv("output_files/" + model_fileName, delimiter=",")

num_samples = df.shape[0]

# Separate the columns into x, y, and z
t = np.linspace(0, num_samples*smaple_time, num_samples)

t = df['t'].values  
# X_des = df.loc[:, 'xtarg1':'xtarg4'].values  
# X_ctrl = df.loc[:, 'x1':'x4'].values  

# x1_dot = np.diff(X_ctrl[:,0])/smaple_time
# x1_dot = np.append(x1_dot, x1_dot[-1])

# x2_dot = np.diff(X_ctrl[:,2])/smaple_time
# x2_dot = np.append(x2_dot, x2_dot[-1])

# Create  plot
fig = plt.figure(figsize=(12, 7))
gs = GridSpec(3, 1, figure=fig, height_ratios=[1, 1, 1])
axs = []
axs.append(fig.add_subplot(gs[0, 0]))  # ax[0]
axs.append(axs[0].twinx())
axs.append(fig.add_subplot(gs[1, 0]))  # ax[1]
axs.append(axs[2].twinx())
axs.append(fig.add_subplot(gs[2, 0]))  # ax[1]

axs[0].plot(t, df["pred_p_x"].to_numpy(), c="black", label="$x^{kpm}_1$", linewidth=1.5)  
axs[1].plot(t, df["pred_v_x"].to_numpy(), c="red", label="$x^{kpm}_2$", linewidth=1.5)  
axs[2].plot(t, df["pred_p_z"].to_numpy(), c="black", label="$x^{kpm}_3$", linewidth=1.5) 
axs[3].plot(t, df["pred_v_z"].to_numpy(), c="red", label="$x^{kpm}_4$", linewidth=1.5)  

axs[0].plot(t, df["p_x"].to_numpy(), c="cyan", label="$x^{}_1$", linestyle="--", linewidth=2)  
axs[1].plot(t, df["v_x"].to_numpy(), c="blue", label="$x^{}_2$", linestyle="--", linewidth=2)  
axs[2].plot(t, df["p_z"].to_numpy(), c="cyan", label="$x^{}_3$", linestyle="--", linewidth=2) 
axs[3].plot(t, df["v_z"].to_numpy(), c="blue", label="$x^{}_4$", linestyle="--", linewidth=2)  

# axs[1].plot(t, x1_dot, c="green", label="$\dot{x}^{model}_1$", linestyle="--", linewidth=1) 
# axs[3].plot(t, x2_dot/2, c="green", label="$\dot{x^{model}_2}$", linestyle="--", linewidth=1.5)  

axs[0].set_title("States")
axs[0].set_ylabel("$x_1$ [mm]")
axs[1].set_ylabel("$x_2$ [mm/s]")
axs[0].minorticks_on()
axs[0].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
handles1, labels1 = axs[0].get_legend_handles_labels()
handles2, labels2 = axs[1].get_legend_handles_labels()
axs[1].legend(handles1 + handles2, labels1 + labels2, loc="best", ncol=2)

axs[2].set_title("States")
axs[2].set_ylabel("$x_3$ [mm]")
axs[3].set_ylabel("$x_4$ [mm/s]")
axs[2].minorticks_on()
axs[2].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
handles3, labels3 = axs[2].get_legend_handles_labels()
handles4, labels4 = axs[3].get_legend_handles_labels()
axs[3].legend(handles3 + handles4, labels3 + labels4, loc="best", ncol=2)

axs[4].plot(t, df["u"].to_numpy(), c="black", label="$u$", linewidth=1.5)  # Scatter plot with red circles

axs[4].set_title("Input")
axs[4].set_xlabel("Time [s]")
axs[4].set_ylabel("u [mm]")
axs[4].minorticks_on()
axs[4].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
axs[4].legend(loc="best", ncol=1)

plt.subplots_adjust(hspace=0.3)  # Set the spacing between subplots
plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(__file__), 'output_plot.pdf'), format='pdf')
plt.savefig(os.path.join(os.path.dirname(__file__), 'output_plot.png'), format='png', dpi=300)
plt.show(block=True)

