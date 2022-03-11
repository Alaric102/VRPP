# import rospy
# from geometry_msgs.msg import Transform
# from tf.transformations import euler_from_quaternion

import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from AlgorithmBase import Algorithm, Dijkstra

startState = np.zeros((3, 2), dtype=float)
goalState = np.zeros((3, 2), dtype=float)

ppAlgorithm = Dijkstra()
fig, axes = ppAlgorithm.Run()
plt.show()

def startState_cb(msg):
    pass
    # quaternion = (
    #     msg.rotation.x,
    #     msg.rotation.z,
    #     msg.rotation.y,
    #     msg.rotation.w)
    # euler = euler_from_quaternion(quaternion)

    # startState[0, 0] = msg.translation.x
    # startState[1, 0] = msg.translation.y
    # startState[2, 0] = msg.translation.z

    # startState[0, 1] = euler[0]
    # startState[1, 1] = euler[1]
    # startState[2, 1] = euler[2]

    # rospy.loginfo(rospy.get_caller_id() + "Start point:\n \
    #     position x: %f, y: %f, z: %f \n \
    #     orientation x: %f, y: %f, z: %f.", \
    #     startState[0, 0], startState[1, 0], startState[2, 0], \
    #     startState[0, 1], startState[1, 1], startState[2, 1])
        
    # rospy.loginfo(rospy.get_caller_id() + "Start point:\n \
    #     position x: %f, y: %f, z: %f \n \
    #     orientation x: %f, y: %f, z: %f.", \
    #     msg.translation.x, msg.translation.y, msg.translation.z, \
    #     euler[0], euler[1], euler[2])
    
def goalState_cb(msg):
    pass
    # rospy.loginfo(rospy.get_caller_id() + "I heard %f", msg.translation.x)

def main():
    pass
    # rospy.init_node('path_planning_node', anonymous=True)

    # rospy.Subscriber("startState", Transform, startState_cb)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    main()