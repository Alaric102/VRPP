import numpy as np
import time

class AlgorithmBase:
    def __init__(self):
        self.__startState = np.ndarray((3,), dtype=float)         # Store float [x, y, z]
        self.__goalState = np.ndarray((3,), dtype=float)          # Store float [x, y, z]
        self.isActive = False

    def SetStartState(self, state :np.ndarray):
        assert self.__startState.shape == state.shape, \
            "setStartState(): " + str(self.__startState.shape) + " != " + str(state.shape)
        self.__startState = state
        print("New start state: ", self.__startState)
        
    def SetGoalState(self, state :np.ndarray):
        assert self.__goalState.shape == state.shape, \
            "SetGoalState(): " + str(self.__goalState.shape) + " != " + str(state.shape)
        self.__goalState = state
        print("New Goal state: ", self.__goalState)

    def GetStartState(self) -> np.ndarray:
        return self.__startState

    def GetGoalState(self) -> np.ndarray:
        return self.__goalState
