#include <iostream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

#include "TcpClient.h"

#define DEFAULT_PORT_SEND "12345"
#define DEFAULT_PORT_RECV "12344"
#define DEFAULT_ADDR "localhost"

TcpClient tcpSender(DEFAULT_ADDR, DEFAULT_PORT_SEND);
TcpClient tcpReceiver(DEFAULT_ADDR, DEFAULT_PORT_RECV);

enum UnityCommands {
    setStartPoint = 1,
    setGoalPoint,
    startPlanning
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

    // ros::Publisher startStatePub = nh.advertise<geometry_msgs::Transform>("startState", 1);
    // ros::Publisher goalStatePub = nh.advertise<geometry_msgs::Transform>("goalState", 1);
    // static geometry_msgs::Transform stateMsg;

    // ros::Publisher startPlanPub = nh.advertise<std_msgs::Bool>("startPlan", 1);
    // static std_msgs::Bool startPlanMsg;
    // startPlanMsg.data = false;

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


        if (tcpReceiver.IsConnected()){
            int recvNum = tcpReceiver.recvBuffer();
            if (recvNum > 0)
                std::cout << "Received: " << recvNum << std::endl;

            // tcpReceiver.recvBuffer();
            // std::cout << "Recv len: " << tcpReceiver.recvBuffer() << std::endl;
            // int cmd = tcpReceiver.recvCommand();
            // switch (cmd) {
            //     case setStartPoint: {
            //         std::cout << "setStartPoint" << std::endl;
            //         // stateMsg.translation = tcpReceiver.getVector3();
            //         // stateMsg.rotation = tcpReceiver.getQuaternion();
            //         // startStatePub.publish(stateMsg);
            //         break;                
            //     } case setGoalPoint: {
            //         std::cout << "setGoalPoint" << std::endl;
            //         // stateMsg.translation = tcpReceiver.getVector3();
            //         // stateMsg.rotation = tcpReceiver.getQuaternion();
            //         // goalStatePub.publish(stateMsg);
            //         break;
            //     } case startPlanning: {
            //         std::cout << "startPlanning" << std::endl;
            //         // startPlanMsg.data = true;
            //         // startPlanPub.publish(startPlanMsg);
            //         break;
            //     }       
            //     default:{
            //         // std::cout << "Unrecognized command: " << cmd << std::endl;
            //         break;
            //     }
            // }
        } else {
            tcpReceiver.Reconnect();
        }
        

        ros::spinOnce();
    }
    return 0;
}