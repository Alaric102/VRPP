from collections import deque
from AStarBase import AStar
from RRTBase import RRTBase
import numpy as np
import utils
import matplotlib.pyplot as plt
from PotentialField import PotentialMap

globalPlanner = AStar()
localPlanner = RRTBase()

globalPlanner.LoadVoxelMap(full_path = "D:/catkin_ws/src/VRPP_ROS/launch/map.txt")

startState = np.array([2.15109563e+00, -2.37947706e-19,  2.14324236e+00], dtype=float)
goalState = np.array([-1.03313118e-01, -5.38950659e-20,  4.85443622e-01], dtype=float)

globalPlanner.SetStartState(startState)
globalPlanner.SetGoalState(goalState)
localPlanner.SetStartState(startState)
localPlanner.SetGoalState(goalState)

voxelMap = globalPlanner.GetVoxelMap()
potentialMap = PotentialMap()

localPlanner.SetVoxelMap(voxelMap)

fig, axes = utils.PlotVoxelProjection(voxelMap)

# if globalPlanner.GetGlobalPlan():
#     descretePlan = globalPlanner.GetPlan()
#     continuousPlan = []

#     for descreteState in descretePlan:
#         utils.PlotVoxelStateProjection(axes, descreteState, "yellow")
#         continuousPlan.append(globalPlanner.GetContinuousState(descreteState))

#     localPlanner.SetGlobalPlan(continuousPlan)
#     localPlanner.GetLocalPlan()
    
# utils.PlotVoxelStateProjection(axes, globalPlanner.startStateDescrete, "blue")
# utils.PlotVoxelStateProjection(axes, globalPlanner.goalStateDescrete, "green")
plt.show()