import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from VoxelMapBase import VoxelMap

def PlotVoxelMap(voxel_map: VoxelMap):
    x, y, z = voxel_map.GetMapData().nonzero()
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.azim = 90
    ax.elev = 0
    ax.scatter(x, y, z, marker='x', c="red")
    return fig, ax
    
def PlotVoxelState(axes, state: np.ndarray, color = "blue"):
    x, y, z = state
    axes.scatter(x, y, z, c=color, s=50)

def PlotVoxelProjection(voxel_map: VoxelMap):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot()
    x, y, z = voxel_map.GetMapSize()
    res = np.zeros((x, z), dtype=int)
    data = voxel_map.GetMapData()
    for i in range(y):
        res += data[:, i, :].astype(int)
    ax.imshow(res, cmap="gray")
    return fig, ax

def PlotVoxelStateProjection(axes, state: np.ndarray, color = "blue"):
    x, y, z = state
    circle = plt.Circle((z,x), 0.5, color=color)
    axes.add_patch(circle)

def PlotLocalMap(start, end):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.azim = 90
    ax.elev = 0
    x, y, z = start
    ax.scatter(x, y, z, marker='x', c="green")
    x, y, z = end
    ax.scatter(x, y, z, marker='x', c="blue")
    return fig, ax

def PlotLocalState(axes, state: np.ndarray, color = "blue"):
    x, y, z = state
    axes.scatter(x, y, z, c=color, s=50)