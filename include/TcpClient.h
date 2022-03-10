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

#include "CircleBuffer.h"

#pragma comment(lib, "Ws2_32.lib")

class TcpClient {
public:
    TcpClient(const PCSTR address, const PCSTR port):
    port_(port),
    address_(address) {
        circleBuf = new CircleBuffer(512);
    };

    TcpClient::~TcpClient() {
        delete circleBuf;
    };
    
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
    CircleBuffer *circleBuf;

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

//Return len of received data
int TcpClient::recvBuffer(){
    bool l = true;
    if (SOCKET_ERROR == ioctlsocket (socket_, FIONBIO, (unsigned long*) &l) ) {
        std::cout << "recvBuffer() error: " << WSAGetLastError() << std::endl;
        return -1;
    }
    int len = 0;
    if ( SOCKET_ERROR == (len=recv(socket_, (char *)&recvbuf, recvbuflen, 0) )){
        if (WSAGetLastError() != 10035)
            std::cout << "recvBuffer() error: " << WSAGetLastError() << std::endl;
        return -1;
    }
    // std::cout << "Received " << len << " bytes" << std::endl;
    return len;
}

int TcpClient::recvCommand(){
    if (circleBuf->IsEmpty()){
        // std::cout << "Empty circle buffer." << std::endl;
        int recvLen = 0;
        if ((recvLen = recvBuffer()) > 0){
            circleBuf->AddData((uint8_t*)recvbuf, recvLen);
            // std::cout << "Added ";
            // circleBuf->PrintBuffer();
        } else {
            // std::cout << "Nothing to Add" << std::endl;
            return 0;
        }
    }
    
    // Get first head byte
    uint8_t fhead = circleBuf->GetItem();
    uint8_t shead = circleBuf->PredictNext();
    if ((fhead == 255) && (shead == 255)){
        // std::cout << "Message head found" << std::endl;
        circleBuf->GetItem();
    } else {
        return 0;
    }
    // Get message length
    uint16_t msgLen = 0;
    circleBuf->GetData((uint8_t*)(&msgLen), 2);
    // Get message command code
    uint16_t cmdCode = 0;
    circleBuf->GetData((uint8_t*)(&cmdCode), 2);

    return cmdCode;
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
    circleBuf->GetData((uint8_t*)(&res), 4);
    return res;
}