import numpy as np
from scipy import linalg
from collections import deque
import time

from AlgorithmBase import AlgorithmBase

class RRTBase(AlgorithmBase):
    def __init__(self):
        super().__init__()
        self.__globalPlan = []
        self.isActive = False

    def SetGlobalPlan(self, plan):
        print(plan)
        self.__globalPlan = plan
        # Take first current and goal states
        self.__currentState =self.__globalPlan.pop(0)
        self.__nextState = self.__globalPlan.pop(0)
    
    def GetNextState(self):
        return self.__nextState

    def GetCurrentState(self):
        return self.__currentState

    def GetSampleState(self) -> np.array((3,), dtype=float):
        meanState = (self.__currentState + self.__nextState)/2
        # assumption that our covariance is 3*sigma
        sigma = (np.linalg.norm(self.__currentState - self.__nextState, ord=2) / 3) ** 2
        L = linalg.cholesky(np.eye(3, 3) * sigma, lower = True)
        sample = np.random.randn(3)
        return np.matmul(L, sample) + meanState

    # def GetLocalPlan(self):
    #     while(len(self.__globalPlan)):
    #         nextState = 
    #         meanState = (currentState + nextState)/2
    #         sigma = (np.linalg.norm(nextState - currentState, ord=2) / 3) ** 2
    #         fig, axes = utils.PlotLocalMap(currentState, nextState)
    #         utils.PlotLocalState(axes, meanState, color="black")
    #         covariance = np.eye(3, 3) * sigma
    #         L = linalg.cholesky(covariance, lower = True)
    #         uAngles = np.linspace(0, 2*np.pi, 10)
    #         vAngles = np.linspace(0, 2*np.pi, 10)
    #         for u in uAngles:
    #             for v in vAngles:
    #                 x = np.cos(u)*np.sin(v)
    #                 y = np.sin(u)*np.sin(v)
    #                 z = np.cos(v)
    #                 sample = np.matmul(L, np.array([[x], [y], [z]])) + meanState
    #                 utils.PlotLocalState(axes, sample, color="r")
    #         samples = []
    #         samplesNum = 100
    #         sampleDimention = 3
    #         rawSamples = np.random.randn(samplesNum, sampleDimention)
    #         for sample in rawSamples:
    #             # print(type(sample), sample)
    #             sample = np.expand_dims(sample, axis=1)
    #             sample = np.matmul(L, sample) + meanState
    #             utils.PlotLocalState(axes, sample, color="gray")
    #             samples.append(sample)
    #         currentState = nextState
    #         return
    #     return False
