import numpy as np
from collections import deque
import time

from AlgorithmBase import AlgorithmBase
from PotentionalField import PotentionalMap
from Scripts.VoxelMapBase import VoxelMap
import utils

class RRTBase(AlgorithmBase):
    def __init__(self):
        super().__init__()
        self.__globalPlan = []
        self.__voxelMap = VoxelMap()

    def SetGlobalPlan(self, plan):
        self.__globalPlan = plan

    def SetVoxelMap(self, voxelMap :VoxelMap):
        self.__voxelMap = voxelMap
        

    def __GetSampleState(self, srcState, dstState) -> np.array((3,), dtype=float):
        pass

    def GetLocalPlan(self):
        currentState = self.__globalPlan.pop(0)
        while(len(self.__globalPlan)):
            nextState = self.__globalPlan.pop(0)
            print(currentState, " -> ", nextState)

            sample_state = self.__GetSampleState(currentState, nextState)

            currentState = nextState
        
        return False
