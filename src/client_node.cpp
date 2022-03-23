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
    startPlanning,
    setRequestedState
};

enum ROSCommands {
    globalPath = 1,
    requestedPose
};

void requestedPose_cb(const geometry_msgs::Pose &msg){
    // BeginBytes + cmdByte + corrdinatesNumber*floatSize
    unsigned int buffSize = 2 + 1 + 3*4;
    uint8_t *buff = new uint8_t[buffSize];
    buff[0] = 0xFF;
    buff[1] = 0xFF;
    buff[2] = (uint8_t)requestedPose;
    unsigned int offset = 3;
    
    float f = (float)msg.position.x;
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&f);
    std::memcpy(buff + offset, bytes, 4);
    offset += 4;

    f = (float)msg.position.y;
    bytes = reinterpret_cast<uint8_t*>(&f);
    std::memcpy(buff + offset, bytes, 4);
    offset += 4;

    f = (float)msg.position.z;
    bytes = reinterpret_cast<uint8_t*>(&f);
    std::memcpy(buff + offset, bytes, 4);
    offset += 4;
        
    std::cout << "Request: "<< (float)msg.position.x << ", " << 
        (float)msg.position.y << ", " << 
        (float)msg.position.z << std::endl;

    tcpSender.SendBuffer((char*) buff, buffSize);
}

void globalPath_cb(const nav_msgs::Path &msg){
    const std::vector<geometry_msgs::PoseStamped> poses = msg.poses;
    // BeginBytes + cmdByte + pathLength + posesSize*corrdinatesNumber*floatSize
    unsigned int buffSize = 2 + 1 + 1 + poses.size()*3*4;
    uint8_t *buff = new uint8_t[buffSize];
    buff[0] = 0xFF;
    buff[1] = 0xFF;
    buff[2] = (uint8_t)globalPath;
    buff[3] = poses.size() & 0xFF;
    unsigned int offset = 4;

    for (int i = 0; i < poses.size(); ++i){
        float f = (float)poses[i].pose.position.x;
        uint8_t* bytes = reinterpret_cast<uint8_t*>(&f);
        std::memcpy(buff + offset, bytes, 4);
        offset += 4;

        f = (float)poses[i].pose.position.y;
        bytes = reinterpret_cast<uint8_t*>(&f);
        std::memcpy(buff + offset, bytes, 4);
        offset += 4;
        
        f = (float)poses[i].pose.position.z;
        bytes = reinterpret_cast<uint8_t*>(&f);
        std::memcpy(buff + offset, bytes, 4);
        offset += 4;

        // std::cout << (float)poses[i].pose.position.x << ", " << 
        //     (float)poses[i].pose.position.y << ", " << 
        //     (float)poses[i].pose.position.z << std::endl;
        // std::cout << "f: " << f << ", bytes: ";
        // for (unsigned int j = 0; j < 4; ++j)
        //     std::cout << +bytes[j] << " ";
        // std::cout << std::endl;
    }
    tcpSender.SendBuffer((char*) buff, buffSize);
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
    ros::Publisher requestedPosePub = nh.advertise<geometry_msgs::Transform>("requestedPose", 1);
    static geometry_msgs::Transform stateMsg;

    ros::Publisher startPlanPub = nh.advertise<std_msgs::Bool>("startPlan", 1);
    static std_msgs::Bool startPlanMsg;
    startPlanMsg.data = false;

    ros::Subscriber globalPathSub = nh.subscribe("globalPath", 10, globalPath_cb);
    ros::Subscriber requestedPoseSub = nh.subscribe("requestPose", 10, requestedPose_cb);


    while(nh.ok()){
        // Check Sender connection
        if (!tcpSender.IsConnected()){
            tcpSender.Reconnect();
        }

        // Receive from Unity
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

        // Proccess circle buffer while it is not empty
        while(!circleBuffer.IsEmpty()){
            int cmd = circleBuffer.GetCommand();
            switch (cmd){
                case (setStartPoint):{
                    circleBuffer.GetVector3(stateMsg.translation);
                    circleBuffer.GetQuaternion(stateMsg.rotation);
                    startStatePub.publish(stateMsg);
                    break;
                }
                case (setGoalPoint):{
                    circleBuffer.GetVector3(stateMsg.translation);
                    circleBuffer.GetQuaternion(stateMsg.rotation);
                    goalStatePub.publish(stateMsg);
                    break;
                }
                case (startPlanning):{
                    startPlanMsg.data = true;
                    startPlanPub.publish(startPlanMsg);
                    break;
                } case (setRequestedState):{
                    circleBuffer.GetVector3(stateMsg.translation);
                    circleBuffer.GetQuaternion(stateMsg.rotation);
                    uint16_t isFree;
                    circleBuffer.GetData((uint8_t*)&isFree, 2);
                    if (isFree > 0){
                        requestedPosePub.publish(stateMsg);
                    }
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