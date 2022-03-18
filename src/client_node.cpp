#include <iostream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

#include "TcpClient.h"

#define DEFAULT_PORT "12345"
#define DEFAULT_ADDR "localhost"

TcpClient tcpClient(DEFAULT_ADDR, DEFAULT_PORT);



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
    std::cout << tcpClient.requestPose(msg) << std::endl;
}


void globalPath_cb(const nav_msgs::Path &msg){
    tcpClient.sendPath(msg.poses);
}


int main(int argc, char **argv){

    if (!tcpClient.init()){
        std::cout << "Failed to init TCP Client." << std::endl;
        return 1;
    }

    if (!tcpClient.createSocket()){
        std::cout << "Failed to Create socket." << std::endl;
        return 1;
    }

    if (!tcpClient.connectScoket()){
        std::cout << "Failed to connect socket." << std::endl;
    }

    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;
    ros::Rate loopRate(10);

    ros::Publisher startStatePub = nh.advertise<geometry_msgs::Transform>("startState", 1);
    ros::Publisher goalStatePub = nh.advertise<geometry_msgs::Transform>("goalState", 1);
    static geometry_msgs::Transform stateMsg;

    ros::Publisher startPlanPub = nh.advertise<std_msgs::Bool>("startPlan", 1);
    static std_msgs::Bool startPlanMsg;
    startPlanMsg.data = false;

    ros::Subscriber globalPathSub = nh.subscribe("globalPath", 10, globalPath_cb);
    ros::Subscriber requestedPoseSub = nh.subscribe("requestedPose", 10, requestedPose_cb);

    while(nh.ok()){
        if (tcpClient.isConnected()){
            
        } else {
            tcpClient.Reconnect();
            std::cout << "Reconnecting" << std::endl;
        }
        
        int cmd = tcpClient.recvCommand();
        switch (cmd) {
            case setStartPoint: {
                stateMsg.translation = tcpClient.getVector3();
                stateMsg.rotation = tcpClient.getQuaternion();
                startStatePub.publish(stateMsg);
                break;                
            } case setGoalPoint: {
                stateMsg.translation = tcpClient.getVector3();
                stateMsg.rotation = tcpClient.getQuaternion();
                goalStatePub.publish(stateMsg);
                break;
            } case startPlanning: {
                startPlanMsg.data = true;
                startPlanPub.publish(startPlanMsg);
                break;
            }       
            default:{
                // std::cout << "Unrecognized command: " << cmd << std::endl;
                break;
            }
        }

        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}