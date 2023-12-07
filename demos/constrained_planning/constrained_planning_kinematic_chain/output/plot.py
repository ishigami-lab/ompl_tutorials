# %%
from matplotlib import animation
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math

def plotChain(axis, joints_angle, joint_color, link_color):
  link_length = 1.0 / n_link
  theta = 0.0
  x_prev = 0.0
  y_prev = 0.0
  axis.add_patch(patches.Circle(xy=(0, 0), radius=0.05, facecolor=joint_color))
  axis.plot([-1, 1], [link_length, link_length], color="g", linewidth=3, linestyle="dotted")

  for ja in joints_angle:
    theta += ja
    x = x_prev + link_length * math.cos(theta)
    y = y_prev + link_length * math.sin(theta)
    axis.plot([x_prev, x], [y_prev, y], color=link_color, linewidth=4)
    axis.add_patch(patches.Circle(xy=(x, y), radius=0.05, facecolor=joint_color))

    x_prev = x
    y_prev = y

def update_anim(i):
  plt.cla()
  ax = fig.gca()
  ax.set_xlim(-1.2, 1.2)
  ax.set_ylim(-1.2, 1.2)
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_aspect('equal', adjustable='box')
  plotChain(ax, joints_history[-1,:], joint_color='violet', link_color='steelblue')
  plotChain(ax, joints_history[i,:], joint_color='r', link_color='b')

# %% [markdown]
# ### 問題設定


# %% [markdown]
# ### 出力結果（後処理なし）

# %%
# figure settings
fig = plt.figure(figsize=(6,6))

# load path data
joints_history = np.loadtxt('simplepath.txt')
history_num = joints_history.shape[0]
n_link = joints_history.shape[1]

# save animation
ani = animation.FuncAnimation(fig, update_anim, interval = 50, save_count=history_num)
ani.save('simplepath_animation.gif', writer='pillow')
plt.show()
# %%
