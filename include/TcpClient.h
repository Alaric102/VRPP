#pragma once
// #ifndef _TCP_CLIENT_H_
// #define _TCP_CLIENT_H_

// #ifdef _WIN32
// #else
// #define SD_BOTH 0
// #endif

#include <iostream>
#include <string>

#include <winsock2.h>
#include <ws2tcpip.h>

#include <geometry_msgs/Transform.h>

#pragma comment(lib, "Ws2_32.lib")

class TcpClient {
public:
    TcpClient(const PCSTR address, const PCSTR port):
    port_(port),
    address_(address) {};

    TcpClient::~TcpClient() {};
    
    // init WSA
    bool init();

    // Create a SOCKET for connecting to server
    bool createSocket();

    bool connectScoket();
    
    bool Reconnect();
    
    bool isConnected();

    int recvBuffer();

    int recvCommand();

    geometry_msgs::Vector3 getVector3();
    geometry_msgs::Quaternion getQuaternion();

    struct addrinfo* getAddrInfo() const;
private:
    WSADATA wsaData;
    const PCSTR port_, address_;
    struct addrinfo *pAddrInfo = NULL, hints;
    SOCKET socket_;

    const static int recvbuflen = 512;
    char recvbuf[recvbuflen];

    int startId = 0;
    bool sendBuffer(char *buff, int len);
    int getRecvMsgStart();
    float getFloat();
};

bool TcpClient::init(){
    if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0) {
        return false;
    }

    ZeroMemory( &hints, sizeof(hints) );
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    if (getaddrinfo(address_, port_, &hints, &pAddrInfo) != 0) {
        WSACleanup();
        return false;
    }
    return true;
}

bool TcpClient::createSocket(){
    socket_ = socket( pAddrInfo->ai_family, pAddrInfo->ai_socktype, pAddrInfo->ai_protocol);
    if (socket_ == INVALID_SOCKET) {
        return false;
    }
    return true;
}

// Connect SOCKET to server
bool TcpClient::connectScoket(){
    if (connect(socket_, pAddrInfo->ai_addr, (int)pAddrInfo->ai_addrlen)) {
        closesocket(socket_);
        socket_ = INVALID_SOCKET;
        return false;
    }
    return true;
}

bool TcpClient::Reconnect(){
    closesocket(socket_);
    if (!createSocket()){
        return false;
    }

    if (!connectScoket()) {
        return false;
    }
    return true;
}

bool TcpClient::isConnected(){
    char *sendbuf = "0";
    return sendBuffer(sendbuf, 1);
}

struct addrinfo* TcpClient::getAddrInfo() const {
        return pAddrInfo;
};

bool TcpClient::sendBuffer(char *buff, int len){
    if (send(socket_, buff, len, 0) == SOCKET_ERROR) {
        return false;
    }
    return true;
}

int TcpClient::recvBuffer(){
    bool l = true;
    if (SOCKET_ERROR == ioctlsocket (socket_, FIONBIO, (unsigned long*) &l) ) {
        return WSAGetLastError();
    }
    int len;
    if ( SOCKET_ERROR == (len=recv(socket_, (char *)&recvbuf, recvbuflen, 0) )){
        return WSAGetLastError();
    }
    std::cout << "reveived " << len << " bytes" << std::endl;
    return 0;
}

int TcpClient::recvCommand(){
    if (recvBuffer()){
        std::cout << "error" << std::endl;
        return -1;
    }
    startId = getRecvMsgStart();
    int msgLength = recvbuf[startId];
    startId += 2;

    if (startId + msgLength >= recvbuflen){
        return 0;
    }
    int msgCmdCode = recvbuf[startId];
    startId += 2;

    // std::cout << "Received Len: " << msgLength << " cmd: "  << msgCmdCode << std::endl;
    return msgCmdCode;
}

int TcpClient::getRecvMsgStart(){
    for (int i = 0; i < recvbuflen - 2; ++i){
        if ( ((uint8_t)recvbuf[i] == 255) && ((uint8_t)recvbuf[i + 1] == 255) ){
            return i + 2;
        }
    }
    return -1;
}

geometry_msgs::Vector3 TcpClient::getVector3(){
    geometry_msgs::Vector3 res;
    res.x = getFloat();
    res.y = getFloat();
    res.z = getFloat();
    return res;
}

geometry_msgs::Quaternion TcpClient::getQuaternion(){
    geometry_msgs::Quaternion res;
    float yaw = getFloat();
    float pitch = getFloat();
    float roll = getFloat();
    
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    res.w = cr * cp * cy + sr * sp * sy;
    res.x = sr * cp * cy - cr * sp * sy;
    res.y = cr * sp * cy + sr * cp * sy;
    res.z = cr * cp * sy - sr * sp * cy;

    return res;
}

float TcpClient::getFloat(){
    float res;
    *((uint8_t*)(&res) + 0) = recvbuf[startId];
    *((uint8_t*)(&res) + 1) = recvbuf[startId + 1];
    *((uint8_t*)(&res) + 2) = recvbuf[startId + 2];
    *((uint8_t*)(&res) + 3) = recvbuf[startId + 3];
    startId += 4;
    return res;
}