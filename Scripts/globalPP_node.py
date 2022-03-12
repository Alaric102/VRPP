import rospy
from geometry_msgs.msg import Transform, PoseStamped, Pose
from nav_msgs.msg import Path
from std_msgs.msg import Bool

import numpy as np

from AlgorithmBase import Dijkstra

ppAlgorithm = Dijkstra()
# if ppAlgorithm.Run():
#     fig, axes = ppAlgorithm.PlotVoxelMap()
#     for point in ppAlgorithm.path:
#         ppAlgorithm.CombinePlot(axes, point, "black")
#     plt.show()

def startState_cb(msg):
    rospy.loginfo(rospy.get_caller_id() + ": Start state: position x: %f, y: %f, z: %f", \
        msg.translation.x, msg.translation.y, msg.translation.z)
    
    # startState = np.array([msg.translation.x, msg.translation.y, msg.translation.z], dtype=float)
    startState = np.array([msg.translation.x, 2, msg.translation.z], dtype=float)
    ppAlgorithm.SetStartState(startState)
    
def goalState_cb(msg):
    rospy.loginfo(rospy.get_caller_id() + ": Goal state: position x: %f, y: %f, z: %f", \
        msg.translation.x, msg.translation.y, msg.translation.z)
    
    # goalState = np.array([msg.translation.x, msg.translation.y, msg.translation.z], dtype=float)
    goalState = np.array([msg.translation.x, 2, msg.translation.z], dtype=float)
    ppAlgorithm.SetGoalState(goalState)

def startPlan_cb(msg):
    if msg.data:
        rospy.loginfo(rospy.get_caller_id() + " : Start plan: " + str(msg.data) )
        if ppAlgorithm.InitVoxelMap():
            ppAlgorithm.isActive = True
        

def main():
    rospy.init_node('globalPP_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    rospy.Subscriber("startState", Transform, startState_cb)
    rospy.Subscriber("goalState", Transform, goalState_cb)
    rospy.Subscriber("startPlan", Bool, startPlan_cb)
    
    globalPathPub = rospy.Publisher('globalPath', Path, queue_size=10)

    while not rospy.is_shutdown():
        if ppAlgorithm.isActive:
            ppAlgorithm.isActive = False
            if ppAlgorithm.Run():
                globalPathMsg = Path()
                globalPathMsg.header.stamp = rospy.Time.now()
                globalPathMsg.header.frame_id = "globalPath"

                path = ppAlgorithm.path
                for pose in path:
                    poseMsg = PoseStamped()
                    poseMsg.pose.position.x = pose[0]
                    poseMsg.pose.position.y = pose[1]
                    poseMsg.pose.position.z = pose[2]

                    globalPathMsg.poses.append(poseMsg)
                globalPathPub.publish(globalPathMsg)

        rate.sleep()

if __name__ == '__main__':
    main()