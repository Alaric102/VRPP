import numpy as np
from VoxelMapBase import VoxelMap

class PotentionalMap:
    def __init__(self) -> None:
        self.__voxelMap = VoxelMap()

    def MakePotentionalFromVoxel(self, voxelMap: VoxelMap):
        self.__voxelMap = voxelMap
