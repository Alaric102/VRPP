from AStarBase import AStar
import numpy as np
import utils
import matplotlib.pyplot as plt

globalPlanner = AStar()
globalPlanner.LoadVoxelMap(full_path = "D:/catkin_ws/src/VRPP_ROS/launch/map.txt")

startState = np.array([0.16760781, 2.0,         2.2329998], dtype=float)
goalState = np.array([0.0, 2.0, 0.0], dtype=float)
globalPlanner.SetStartState(startState)
globalPlanner.SetGoalState(goalState)

if globalPlanner.GetGlobalPlan():
    plan = globalPlanner.GetPlan()

    voxelMap = globalPlanner.GetVoxelMap()
    fig, axes = utils.PlotVoxelProjection(voxelMap)
    for state in plan:
        utils.PlotVoxelStateProjection(axes, state, "yellow")
    
    utils.PlotVoxelStateProjection(axes, globalPlanner.startStateDescrete, "blue")
    utils.PlotVoxelStateProjection(axes, globalPlanner.goalStateDescrete, "green")
    plt.show()