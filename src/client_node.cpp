#include <iostream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

#include "TcpClient.h"
#include "CircleBuffer.h"

#define DEFAULT_PORT_SEND "12345"
#define DEFAULT_PORT_RECV "12344"
#define DEFAULT_ADDR "localhost"

TcpClient tcpSender(DEFAULT_ADDR, DEFAULT_PORT_SEND);
TcpClient tcpReceiver(DEFAULT_ADDR, DEFAULT_PORT_RECV);
CircleBuffer circleBuffer(512);

enum UnityCommands {
    setStartPoint = 1,
    setGoalPoint,
    startPlanning
};

enum ROSCommands {
    globalPath = 1,
    requestedPose
};

// Client
// 1. Initialize Winsock.
// 2. Create a socket.
// 3. Connect to the server.
// 4. Send and receive data.
// 5. Disconnect.

void requestedPose_cb(const geometry_msgs::Pose &msg){
    // std::cout << tcpClient.requestPose(msg) << std::endl;
}


void globalPath_cb(const nav_msgs::Path &msg){
    // tcpClient.sendPath(msg.poses);
}


int main(int argc, char **argv){
    // Init sender (make one function in future)
    if (!tcpSender.init()){
        std::cout << "Failed to init TCP Sender." << std::endl;
        return 1;
    }

    if (!tcpReceiver.init()){
        std::cout << "Failed to init TCP Receiver." << std::endl;
        return 1;
    }
    
    if (!tcpSender.connectScoket()){}

    if (!tcpReceiver.connectScoket()){}

    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;
    ros::Rate loopRate(10);

    ros::Publisher startStatePub = nh.advertise<geometry_msgs::Transform>("startState", 1);
    ros::Publisher goalStatePub = nh.advertise<geometry_msgs::Transform>("goalState", 1);
    static geometry_msgs::Transform stateMsg;

    ros::Publisher startPlanPub = nh.advertise<std_msgs::Bool>("startPlan", 1);
    static std_msgs::Bool startPlanMsg;
    startPlanMsg.data = false;

    // ros::Subscriber globalPathSub = nh.subscribe("globalPath", 10, globalPath_cb);
    // ros::Subscriber requestedPoseSub = nh.subscribe("requestedPose", 10, requestedPose_cb);


    int counter = 0;
    while(nh.ok()){
        // if (tcpSender.IsConnected()){
        //     // std::cout << "Sender is connected." << std::endl;
        //     // counter = ++counter % 100;
        //     // geometry_msgs::Pose msg;
        //     // msg.position.x = 1.5 + counter;
        //     // msg.position.y = -0.1 - counter;
        //     // msg.position.z = 0.01 * counter;
        //     // tcpSender.requestPose(msg);
        // } else {
        //     tcpSender.Reconnect();
        // }


        // receive from Unity
        if (tcpReceiver.IsConnected()){
            size_t recvlen = (size_t)tcpReceiver.recvBuffer();
            // If something was received add to circle buffer
            if (recvlen > 0){
                uint8_t* receivedBytes = tcpReceiver.GetReceived(recvlen);
                circleBuffer.AddData(receivedBytes, recvlen);
                delete[] receivedBytes;
            }
        } else {
            tcpReceiver.Reconnect();
        }

        while(!circleBuffer.IsEmpty()){
            int cmd = circleBuffer.GetCommand();
            switch (cmd){
                case (setStartPoint):{
                    std::cout << "Set Start State" << std::endl;
                    circleBuffer.GetVector3(stateMsg.translation);
                    circleBuffer.GetQuaternion(stateMsg.rotation);
                    startStatePub.publish(stateMsg);
                    break;
                }
                case (setGoalPoint):{
                    std::cout << "Set Goal State" << std::endl;
                    circleBuffer.GetVector3(stateMsg.translation);
                    circleBuffer.GetQuaternion(stateMsg.rotation);
                    goalStatePub.publish(stateMsg);
                    break;
                }
                case (startPlanning):{
                    std::cout << "Start planning" << std::endl;
                    startPlanMsg.data = true;
                    startPlanPub.publish(startPlanMsg);
                    break;
                }
                
                default:{
                    std::cout << "Unrecognized command: " << cmd << std::endl;
                    break;
                }
            }
        }
        

        ros::spinOnce();
    }
    return 0;
}