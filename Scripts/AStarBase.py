import numpy as np
from collections import deque
import time

from VoxelMapBase import VoxelMap
from GraphBase import Graph
from AlgorithmBase import AlgorithmBase

class AStar(AlgorithmBase):
    def __init__(self):
        super().__init__()
        self.__voxelMap = VoxelMap()
        self.startStateDescrete = np.ndarray((3,), dtype=np.uint)         # Store int [x, y, z]
        self.goalStateDescrete = np.ndarray((3,), dtype=np.uint)          # Store int [x, y, z]

        self.__queue = deque()

    def LoadVoxelMap(self, full_path: str = "map.txt") -> bool:
        return self.__voxelMap.LoadVoxelMap(full_path = full_path)
    
    def GetVoxelMap(self):
        return self.__voxelMap

    def GetGraph(self):
        return self.__graph
        
    def __getDescreteState(self, state: np.ndarray) -> np.ndarray:
        assert self.__voxelMap.GetGridSize().shape == state.shape, "__GetDescreteState(): Wrong shape!"
        state -= self.__voxelMap.GetMinCorner()
        return np.array(state/self.__voxelMap.GetGridSize() , dtype=np.uint)

    def __FindLowerBound(self, item, min, max):
        if (max - min == 1):
            key, _ = self.__queue[min]
            return min if item[0] < key else max
        mean = (max + min) // 2
        key, _ = self.__queue[mean]
        if key > item[0]:
            max = mean
        else:
            min = mean
        return self.__FindLowerBound(item, min, max)

    def __Insert(self,cost:int, state: np.ndarray):
        if (self.__queue.__len__() == 0):
            self.__queue.append((cost, state))
        else:
            min_id = 0
            max_id = self.__queue.__len__()
            id = self.__FindLowerBound((cost, state), min_id, max_id)
            self.__queue.insert(id, (cost, state))
        return

    def __SetVisited(self, cost: int, state: np.ndarray):
        x, y, z = state
        self.__visitedMap[x, y, z] = (True, cost)
    
    def __GetActionSpace(self):
        actionSpace = []
        for x in [1, 0, -1]:
            for y in [0]:
                for z in [1, 0, -1]:
                    if (x == 0) and (y == 0) and (z == 0):
                        continue
                    action = np.array([x, y, z], dtype=int)
                    actionSpace.append(action)
        return actionSpace

    def __GetActionCost(self, action: np.ndarray) -> int:
        x, y, z = action
        return 1 

    def __GetHeuristic(self, state):
        delta = self.goalStateDescrete - state
        delta[1] = 0
        return np.linalg.norm(delta, ord=2)

    def __IsNearGoal(self, state) -> bool:
        return ((self.goalStateDescrete[0] == state[0]) and (self.goalStateDescrete[2] == state[2]))

    def GetGlobalPlan(self) -> bool:
        # Descretize start/goal states
        self.startStateDescrete = self.__getDescreteState(self.GetStartState())
        self.goalStateDescrete = self.__getDescreteState(self.GetGoalState())
        while (not self.__voxelMap.IsObastacle(self.startStateDescrete + np.array([0, -1, 0], dtype=np.uint))):
            self.startStateDescrete += np.array([0, -1, 0], dtype=np.uint)
        while (not self.__voxelMap.IsObastacle(self.goalStateDescrete + np.array([0, -1, 0], dtype=np.uint))):
            self.goalStateDescrete += np.array([0, -1, 0], dtype=np.uint)
        print("startStateDescrete", self.startStateDescrete)
        print("goalStateDescrete", self.goalStateDescrete)
        if self.__voxelMap.IsObastacle(self.startStateDescrete):
            print("Start state is obstacle or out of box")
            return False
        if self.__voxelMap.IsObastacle(self.goalStateDescrete):
            print("Goal state is obstacle or out of box")
            return False

        # Init Map for visitted states and Graph
        self.__visitedMap = np.zeros_like(self.__voxelMap.GetMapData(), dtype=tuple)
        self.__visitedMap.fill((False,0))
        self.__graph = Graph(self.startStateDescrete)

        self.__Insert( 0, self.startStateDescrete )
        self.__SetVisited( 0, self.startStateDescrete )

        start = time.time()
        while len(self.__queue) > 0:
            _ , currentState = self.__queue.popleft()

            if self.__IsNearGoal(currentState):
                if (currentState == self.goalStateDescrete).all():
                    print("Finished in", time.time() - start, "sec.")
                    return True
                else:
                    self.__graph.Append(self.goalStateDescrete, currentState)
                    print("Finished in", time.time() - start, "sec.")
                    return True

            actionSpace = self.__GetActionSpace()
            for action in actionSpace:
                nextState = currentState + action

                if self.__voxelMap.IsObastacle(nextState):
                    # are we able to climb obstacle?
                    lifting = np.array([0, 1, 0], dtype=int)
                    liftingLevel = 1
                    while self.__voxelMap.IsObastacle(nextState + lifting*liftingLevel):
                        liftingLevel += 1
                        if liftingLevel > 8:
                            break

                    if liftingLevel > 8:
                        continue
                    else:
                        action += lifting*liftingLevel
                        nextState += lifting*liftingLevel
                
                # are we able to go down from obstacle?
                descent = np.array([0, -1, 0], dtype=int)
                descentLevel = 0
                while not self.__voxelMap.IsObastacle(nextState + descent*descentLevel):
                    descentLevel += 1
                descentLevel -= 1
                
                if descentLevel < 8:
                    action += descent * descentLevel
                    nextState += descent * descentLevel

                x, y, z = nextState
                isVisited, prevNextCost = self.__visitedMap[x, y, z]
                x, y, z = currentState
                _, currentCost = self.__visitedMap[x, y, z]
                if not isVisited:
                    nextCost = currentCost + self.__GetActionCost(action)
                    self.__SetVisited(nextCost, nextState)
                    dequeCost = nextCost + self.__GetHeuristic(nextState)
                    self.__Insert(dequeCost, nextState)
                    self.__graph.Append(nextState, currentState)
                    
                else:
                    newCost = currentCost + self.__GetActionCost(action)
                    if newCost < prevNextCost:
                        self.__SetVisited(newCost, nextState)
                        self.__graph.Append(nextState, currentState)

        print("No path exists. Last state:", currentState)
        return False

    def GetPlan(self):
        res = self.__graph.GetBranch(self.goalStateDescrete)
        print("Path length: ", len(res))
        return res

# ppAlg = Descrete()
