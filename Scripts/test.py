from collections import deque
from threading import local
from AStarBase import AStar
from RRTBase import RRTBase
import numpy as np
import utils
import matplotlib.pyplot as plt
from PotentialField import PotentialMap
import time as time

globalPlanner = AStar()
localPlanner = RRTBase()

globalPlanner.LoadVoxelMap(full_path = "D:/catkin_ws/src/VRPP_ROS/launch/mapObst.txt")

startState = np.array([3.710000, 0.000000, 7.040000], dtype=float)
goalState = np.array([-3.680000, 0.000000, -7.610000], dtype=float)

globalPlanner.SetStartState(startState)
globalPlanner.SetGoalState(goalState)
localPlanner.SetStartState(startState)
localPlanner.SetGoalState(goalState)

voxelMap = globalPlanner.GetVoxelMap()
fig, axes = utils.PlotVoxelProjection(voxelMap)

# localPlanner.SetVoxelMap(voxelMap)


timerList = []
for i in range(2):
    startTime = time.time()
    if globalPlanner.GetGlobalPlan():
        timerList.append(time.time() - startTime)
        descretePlan = globalPlanner.GetPlan()
        
        optimizedPath = [descretePlan[0]]
        prevDelta = descretePlan[1] - descretePlan[0]
        for i in range(1, len(descretePlan) - 1):
            delta = descretePlan[i + 1].astype(int) - descretePlan[i].astype(int)
            if np.any(prevDelta != delta):
                prevDelta = delta
                optimizedPath.append(descretePlan[i])
    
        print("path len optimized: ", len(optimizedPath))

        continuousPlan = []
        for descreteState in descretePlan:
            utils.PlotVoxelStateProjection(axes, descreteState, "yellow")
            
        continuousPlan = []
        for descreteState in optimizedPath:
            utils.PlotVoxelStateProjection(axes, descreteState, "red")
            print(globalPlanner.GetContinuousState(descreteState))
            continuousPlan.append(globalPlanner.GetContinuousState(descreteState))
        
        # localPlanner.SetGlobalPlan(continuousPlan)
        # currentState = localPlanner.GetCurrentState()
        # nextState = localPlanner.GetNextState()
        # figRRT, axesRRt = utils.PlotLocalMap(currentState, nextState)
        # for i in range(200):
        #     sampledState = localPlanner.GetSampleState()
        #     utils.PlotLocalState(axesRRt, sampledState, color="grey")
        
print(np.mean(np.array(timerList)))
utils.PlotVoxelStateProjection(axes, globalPlanner.startStateDescrete, "blue")
utils.PlotVoxelStateProjection(axes, globalPlanner.goalStateDescrete, "green")
plt.show()