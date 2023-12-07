# %%
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from turtle import update
from matplotlib import animation
import matplotlib.pyplot as plt
import numpy as np

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

def plotPlatform(axis, poly, color):
  axis.add_collection3d(Poly3DCollection([poly], color=color))

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

  platform_vertices = []
  chain_list_current = np.array_split(joints_history[i,:], n_chain)
  for clc in chain_list_current:
    plotChain(ax, clc, joint_color='r', link_color='b')
    platform_vertices.append(clc[-3:])

  chain_list_last = np.array_split(joints_history[-1,:], n_chain)
  for cll in chain_list_last:
    plotChain(ax, cll, joint_color='violet', link_color='steelblue')

  plotPlatform(ax, poly=platform_vertices, color=(0.1, 0.8, 0.1, 0.5))

# %% [markdown]
# ### 問題設定


# %% [markdown]
# ### 出力結果（後処理なし）

# %%
# figure settings
fig = plt.figure(figsize=(6,6))

# load info data
info = np.loadtxt('parallel_info.txt')
n_link = info[0].astype(np.int64)
n_chain = info[1].astype(np.int64)
joint_radius = info[2]
length = info[3]
radius = info[4]

# load path data
joints_history = np.loadtxt('simplepath.txt')
history_num = joints_history.shape[0]

# save animation
ani = animation.FuncAnimation(fig, update_anim, interval=50, save_count=history_num)
ani.save('simplepath_animation.gif', writer='pillow')
plt.show()
# %%
