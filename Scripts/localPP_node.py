import rospy
from geometry_msgs.msg import Transform, PoseStamped, Pose
from collections import deque
from nav_msgs.msg import Path
from std_msgs.msg import Bool

from RRTBase import RRTBase
import numpy as np

localPlanner = RRTBase()

def startState_cb(msg):
    rospy.loginfo(rospy.get_caller_id() + ": Start state: position x: %f, y: %f, z: %f", \
        msg.translation.x, msg.translation.y, msg.translation.z)
    
    # startState = np.array([msg.translation.x, msg.translation.y, msg.translation.z], dtype=float)
    startState = np.array([msg.translation.x, msg.translation.y, msg.translation.z], dtype=float)
    localPlanner.SetStartState(startState)
    
def goalState_cb(msg):
    rospy.loginfo(rospy.get_caller_id() + ": Goal state: position x: %f, y: %f, z: %f", \
        msg.translation.x, msg.translation.y, msg.translation.z)
    
    # goalState = np.array([msg.translation.x, msg.translation.y, msg.translation.z], dtype=float)
    goalState = np.array([msg.translation.x, msg.translation.y, msg.translation.z], dtype=float)
    localPlanner.SetGoalState(goalState)

def globalPath_cb(msg):
    globalPath = []
    for pose in msg.poses:
        state = np.array([[pose.pose.position.x], [pose.pose.position.y], [pose.pose.position.z]], dtype=float)
        globalPath.append(state)
    localPlanner.SetGlobalPlan(globalPath)
    localPlanner.isActive = True

def main():
    rospy.init_node('localPP_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    rospy.Subscriber("startState", Transform, startState_cb)
    rospy.Subscriber("goalState", Transform, goalState_cb)
    rospy.Subscriber("globalPath", Path, globalPath_cb)
    
    localPathPub = rospy.Publisher('localPath', Path, queue_size=10)
    requestedPosePub = rospy.Publisher('requestedPose', Pose, queue_size=10)

    while not rospy.is_shutdown():
        if localPlanner.isActive:
            nextState = localPlanner.GetNextState()
            print("nextState: ", np.transpose(nextState))
            poseMsg = Pose()
            poseMsg.position.x = nextState[0, 0]
            poseMsg.position.y = nextState[1, 0]
            poseMsg.position.z = nextState[2, 0]

            requestedPosePub.publish(poseMsg)
            localPlanner.isActive = False
        rate.sleep()

if __name__ == '__main__':
    main()