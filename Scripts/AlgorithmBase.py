from asyncio.windows_events import NULL
from xmlrpc.client import Boolean
import numpy as np
import time

# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

class Algorithm:
    # def __init__(self, env_map, conf_space, start_point, goal_point):
    def __init__(self):
        self.voxelMapSize = np.zeros((3,), dtype=np.uint)
        self.voxelMapGridSize = np.zeros((3,), dtype=float)
        self.voxelMapMinCorner = np.zeros((3,), dtype=float)
        self.voxelMap = np.empty((0), dtype=bool)               # Obstacles = True, free = False

        self.startState = np.ndarray((3,), dtype=float)         # Store float [x, y, z]
        self.goalState = np.ndarray((3,), dtype=float)          # Store float [x, y, z]
        self.dStartState = np.ndarray((3,), dtype=np.uint)      # Store uint [x, y, z]
        self.dGoalState = np.ndarray((3,), dtype=np.uint)       # Store uint [x, y, z]

        self.confSpace = np.empty((0), dtype=bool)
        self.path = []

        # map which hold visit status {0, 1} and cost of visiting
        self.visit_map = np.zeros_like(self.confSpace, dtype=tuple)
        self.visit_map.fill((False,0))
        self.visited_num = 0

        self.queue = []
        self.parent_table = []
        self.isActive = False
    
    def setStartState(self, state :np.ndarray):
        assert self.startState.shape == state.shape, \
            "setStartState(): " + str(self.startState.shape) + " != " + str(state.shape)
        self.startState = state
        print("New start state: ", self.startState)

    def setGoalState(self, state: np.ndarray):
        assert self.goalState.shape == state.shape, "setGoalState(): Wrong shape!"
        self.goalState = state
        print("New goal state: ", self.startState)

    def InitVoxelMap(self) -> Boolean:
        if self.__LoadVoxelMap():
            self.__GetConfSpace()
            self.dStartState = self.__GetDescreteState(self.startState)
            self.dGoalState = self.__GetDescreteState(self.goalState)
            print("dStartState: ", self.dStartState)
            print("dGoalState: ", self.dGoalState)
            return True
        return False

    def __GetConfSpace(self):
        self.confSpace = self.voxelMap
        self.visit_map = np.zeros_like(self.confSpace, dtype=tuple)
        self.visit_map.fill((False,0))
        
    def __LoadVoxelMap(self, full_path = "D:/catkin_ws/src/VRPP_ROS/launch/map.txt"):
        with open(full_path) as f:
            # Get voxelMap size
            line = f.readline()
            counter = 0
            for word in line.split(maxsplit=3):
                if word.isdecimal():
                    self.voxelMapSize[counter] = int(word)
                else:
                    print("Unexpected word: " + word)
                    return False
                counter += 1
            self.voxelMap = np.zeros(self.voxelMapSize, dtype=bool)
            print("VoxelMap size: \n", self.voxelMap.shape)
            
            # Get voxelMap grid size
            line = f.readline()
            counter = 0
            for word in line.split(maxsplit=3):
                word = word.replace(",", ".", 1)
                # if word.isdecimal():
                self.voxelMapGridSize[counter] = float(word)
                # else:
                #     print("Unexpected word: " + word)
                #     return False
                counter += 1
            print("VoxelMap grid size: \n", self.voxelMapGridSize)
            
            # Get voxelMap min corner
            line = f.readline()
            counter = 0
            for word in line.split(maxsplit=3):
                word = word.replace(",", ".", 1)
                self.voxelMapMinCorner[counter] = float(word)
                counter += 1
            print("VoxelMap min corner: \n", self.voxelMapMinCorner)

            # Get voxelMap data
            line = f.readline()
            while line:
                words = line.split(maxsplit=4)
                if not(words[0].isdecimal() and words[1].isdecimal() and words[2].isdecimal()):
                    print("Invalid data in words: " + words)
                    return False
                x = int(words[0])
                y = int(words[1])
                z = int(words[2])
                value = True if words[3] == "True" else "False"
                self.voxelMap[x, y, z] = value
                line = f.readline()

            f.close()
            return True

    def __GetDescreteState(self, state: np.ndarray):
        assert self.voxelMapGridSize.shape == state.shape, "__GetDescreteState(): Wrong shape!"
        state -= self.voxelMapMinCorner
        return np.array(state/self.voxelMapGridSize , dtype=np.uint)

    # def PlotVoxelMap(self, duration = 0.5):
    #     x, y, z = self.voxelMap.nonzero()
    #     fig = plt.figure(figsize=(10, 10))
    #     ax = fig.add_subplot(111, projection='3d')
    #     ax.azim = 90
    #     ax.elev = 0
    #     ax.scatter(x, y, z, marker='x', c="red")

    #     x, y, z = self.startState
    #     ax.scatter(x, y, z, c="green")

    #     x, y, z = self.goalState
    #     ax.scatter(x, y, z, c="blue")
    #     if (duration > 0):
    #         plt.pause(duration)
    #     return fig, ax

    # def CombinePlot(self, axes, point, color="blue"):
    #     x, y, z = point
    #     axes.scatter(x, y, z, c=color)
    #     return axes

    def isEqual(self, lft, rht): 
        return (lft == rht).all()

    def getActionSpace(self):
        actionSpace = []
        for x in [1, 0, -1]:
            for y in [0]:
                for z in [1, 0, -1]:
                    if (x == 0) and (y == 0) and (z == 0):
                        continue
                    action = np.zeros( (3,), dtype=int )
                    action[0] = x
                    action[1] = y
                    action[2] = z
                    actionSpace.append(action)
        return actionSpace

    def IsVisited(self, point): 
        item = self.visit_map[point[0], point[1], point[2]]
        return ( item[0])

    def setVisited(self, point, cost):
        item = (True, cost)
        x, y, z = point
        self.visit_map[x, y, z] = item
        self.visited_num += 1

    def getVisited(self, point):
        x, y, z = point
        item = self.visit_map[x, y, z]
        return item

    def queueInsert(self, point, cost):
        for i in range(len(self.queue)):
            item = self.queue[i]
            if item[1] <= cost:
                self.queue.insert(i, (point, cost) )
                return
        self.queue.append( (point, cost) )

    def isObstacle(self, point):
        x, y, z = point
        return ( self.confSpace[x, y, z] )

    def GetNextFromQueue(self):
        return self.queue.pop(-1)

    def getHeuristic(self, point):
        delta = self.goalState - point
        return np.linalg.norm(delta[:-1], ord=2)

    def getParentPoint(self, point):
        for i in self.parent_table:
            if (i[1] == point).all():
                return i[0]
        return np.empty

    def getBranch(self):
        child_point = self.dGoalState
        path = [child_point]
        while  not (child_point == self.dStartState).all():
            parent_point = self.getParentPoint(child_point)
            if (parent_point == np.empty).all():
                return path
            path.append(parent_point)
            child_point = parent_point
        return path

    def getPath(self):
        path = self.getBranch()
        path.reverse()
        return path
    
class Dijkstra(Algorithm):
    def Run(self):
        self.path = []
        self.queueInsert(self.dStartState, 0)
        self.setVisited(self.dStartState, 0)

        start = time.time()
        while self.queue != np.empty:
            currentState, cost = self.GetNextFromQueue()

            if self.isEqual(currentState, self.dGoalState):
                self.path = self.getPath()
                end = time.time()
                print("Finished in", end-start, "sec.")
                return True
            
            actionSpace = self.getActionSpace()
            for action in actionSpace:
                nextState = currentState + action
                if (nextState[0] >= self.voxelMapSize[0]) or (nextState[1] >= self.voxelMapSize[1]):
                    continue

                if self.isObstacle(nextState):
                    continue

                isVisited, currentCost = self.getVisited(nextState)
                if not isVisited:
                    nextCost = currentCost + 1
                    self.setVisited(nextState, nextCost)
                    self.queueInsert(nextState, nextCost + self.getHeuristic(nextState))
                    
                    # self.CombinePlot(axes, nextState, color="gray")
                    self.parent_table.append( (currentState, nextState) )
                else:
                    nextCost = currentCost + 1
                    _ , preVcost = self.getVisited(nextState)
                    if nextCost < preVcost:
                        self.setVisited(nextState, nextCost)
                        self.parent_table.append( (currentState, nextState) )
                
        print("No path exists.")
        return False