import rospy
from geometry_msgs.msg import Transform, PoseStamped, Pose
from collections import deque
from nav_msgs.msg import Path
from std_msgs.msg import Bool

from AStarBase import AStar
import numpy as np

globalPlanner = AStar()

def startState_cb(msg):
    rospy.loginfo(rospy.get_caller_id() + ": Start state: position x: %f, y: %f, z: %f", \
        msg.translation.x, msg.translation.y, msg.translation.z)
    
    # startState = np.array([msg.translation.x, msg.translation.y, msg.translation.z], dtype=float)
    startState = np.array([msg.translation.x, msg.translation.y, msg.translation.z], dtype=float)
    globalPlanner.SetStartState(startState)
    
def goalState_cb(msg):
    rospy.loginfo(rospy.get_caller_id() + ": Goal state: position x: %f, y: %f, z: %f", \
        msg.translation.x, msg.translation.y, msg.translation.z)
    
    # goalState = np.array([msg.translation.x, msg.translation.y, msg.translation.z], dtype=float)
    goalState = np.array([msg.translation.x, msg.translation.y, msg.translation.z], dtype=float)
    globalPlanner.SetGoalState(goalState)

def startPlan_cb(msg):
    if msg.data:
        rospy.loginfo(rospy.get_caller_id() + " : Start plan: " + str(msg.data) )
        if globalPlanner.LoadVoxelMap(full_path = "D:/catkin_ws/src/VRPP_ROS/launch/mapObst.txt"):
            globalPlanner.isActive = True

def main():
    rospy.init_node('globalPP_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    rospy.Subscriber("startState", Transform, startState_cb)
    rospy.Subscriber("goalState", Transform, goalState_cb)
    rospy.Subscriber("startPlan", Bool, startPlan_cb)
    
    globalPathPub = rospy.Publisher('globalPath', Path, queue_size=10)

    while not rospy.is_shutdown():
        if globalPlanner.isActive:
            globalPlanner.isActive = False
            if globalPlanner.GetGlobalPlan():
                globalPathMsg = Path()
                globalPathMsg.header.stamp = rospy.Time.now()
                globalPathMsg.header.frame_id = "globalPath"

                path = globalPlanner.GetPlan()
                print(len(path))
                for pose in path:
                    poseMsg = PoseStamped()
                    x, y, z = globalPlanner.GetContinuousState(pose)
                    poseMsg.pose.position.x = x
                    poseMsg.pose.position.y = y
                    poseMsg.pose.position.z = z

                    globalPathMsg.poses.append(poseMsg)
                globalPathPub.publish(globalPathMsg)
        rate.sleep()
        # rospy.

if __name__ == '__main__':
    main()