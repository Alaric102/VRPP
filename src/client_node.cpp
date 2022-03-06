#include <iostream>
#include <string>

#include <ros/ros.h>

#include "TcpClient.h"
#include "MsgBridge.h"

#define DEFAULT_PORT "12345"
#define DEFAULT_ADDR "localhost"

// Client
// 1. Initialize Winsock.
// 2. Create a socket.
// 3. Connect to the server.
// 4. Send and receive data.
// 5. Disconnect.

int main(int argc, char **argv){
    TcpClient tcpClient(DEFAULT_ADDR, DEFAULT_PORT);
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

    while(nh.ok()){
        if (tcpClient.isConnected()){
            std::cout << ".";
        } else {
            tcpClient.Reconnect();
            std::cout << "Reconnecting" << std::endl;
        }

        std::string recvLine = tcpClient.recvLine();
        processString(recvLine);
        loopRate.sleep();
    }
    return 0;
}