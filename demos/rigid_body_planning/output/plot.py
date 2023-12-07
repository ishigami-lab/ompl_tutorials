# %% [markdown]
# ### 生成された軌道（単純化処理なし）
#%%
from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
data = numpy.loadtxt('path.txt')
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(data[:,0],data[:,1],data[:,2],'.-')
plt.show()

# %% [markdown]
# ### 生成された軌道（単純化処理あり）

# %%
from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
data = numpy.loadtxt('path_simplified.txt')
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(data[:,0],data[:,1],data[:,2],'.-')
plt.show()



# %%
