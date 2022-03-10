from cv2 import transform
from lark import Transformer
import rospy
from geometry_msgs.msg import Transform

def startState_cb(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", msg.translation.x)

def main():
    rospy.init_node('path_planning_node', anonymous=True)

    rospy.Subscriber("startState", Transform, startState_cb)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()