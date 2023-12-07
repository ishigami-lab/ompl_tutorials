# %%
from matplotlib import animation
import matplotlib.pyplot as plt
import numpy as np
import math

def plotSphere(axis, position, radius, color, **kwargs):
  u,v=np.mgrid[0:2*np.pi:40j, 0:np.pi:20j]
  x=radius*np.cos(u)*np.sin(v)+position[0]
  y=radius*np.sin(u)*np.sin(v)+position[1]
  z=radius*np.cos(v)+position[2]
  axis.plot_wireframe(x, y, z, color=color, **kwargs)

def plotChain(axis, joints_state, joint_color, link_color):
  joint_pos_list = np.split(joints_state, n_link)
  for i in range(len(joint_pos_list)):
    plotSphere(axis, joint_pos_list[i], 0.2, color=joint_color, linewidth=0.2)
    if(i >= 1):
      axis.plot(xs=[joint_pos_list[i-1][0], joint_pos_list[i][0]],
                ys=[joint_pos_list[i-1][1], joint_pos_list[i][1]],
                zs=[joint_pos_list[i-1][2], joint_pos_list[i][2]],
                color=link_color, linewidth=3)

def update_anim(i):
  plt.cla()
  ax = fig.gca(projection='3d')
  ax.set_box_aspect((1,1,1))
  ax.set_xlim3d(-3.5, 3.5)
  ax.set_ylim3d(-3.5, 3.5)
  ax.set_zlim3d(-3.5, 3.5)
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  plotChain(ax, joints_history[-1,:], joint_color='violet', link_color='steelblue')
  plotChain(ax, joints_history[i,:], joint_color='r', link_color='b')

# %% [markdown]
# ### 問題設定


# %% [markdown]
# ### 出力結果（後処理なし）

# %%
# figure settings
fig = plt.figure(figsize=(6,6))

# load info data
info = np.loadtxt('chain_info.txt')
n_link = info[0]
obstacles = info[1]
extra = info[2]
joint_radius = info[3]
length = info[4]
radius = info[5]
width = info[6]

# load path data
joints_history = np.loadtxt('simplepath.txt')
history_num = joints_history.shape[0]

# save animation
ani = animation.FuncAnimation(fig, update_anim, interval = 50, save_count=history_num)
ani.save('simplepath_animation.gif', writer='pillow')
plt.show()