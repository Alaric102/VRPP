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
        state = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z], dtype=float)
        globalPath.append(state)
    localPlanner.SetGlobalPlan(globalPath)
    localPlanner.isActive = True

def requestedPose_cb(msg):
    rospy.loginfo(rospy.get_caller_id() + ": Requested state: position x: %f, y: %f, z: %f", \
        msg.translation.x, msg.translation.y, msg.translation.z)
    # print("counterReceived")
    # startState = np.array([msg.translation.x, msg.translation.y, msg.translation.z], dtype=float)
    # startState = np.array([msg.translation.x, msg.translation.y, msg.translation.z], dtype=float)
    # localPlanner.SetStartState(startState)

def main():
    rospy.init_node('localPP_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    rospy.Subscriber("startState", Transform, startState_cb)
    rospy.Subscriber("goalState", Transform, goalState_cb)
    rospy.Subscriber("requestedPose", Transform, requestedPose_cb)
    rospy.Subscriber("globalPath", Path, globalPath_cb)
    
    localPathPub = rospy.Publisher('localPath', Path, queue_size=10)
    requestedPosePub = rospy.Publisher('requestPose', Pose, queue_size=10)

    counterSended = 10
    while not rospy.is_shutdown():
        if localPlanner.isActive and counterSended > 0:
            nextState = localPlanner.GetSampleState()
            print("nextState: ", nextState)
            poseMsg = Pose()
            poseMsg.position.x = float(nextState[0])
            poseMsg.position.y = float(nextState[1])
            poseMsg.position.z = float(nextState[2])
            requestedPosePub.publish(poseMsg)
            counterSended -= 1
        rate.sleep()

if __name__ == '__main__':
    main()