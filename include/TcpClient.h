#pragma once
// #ifndef _TCP_CLIENT_H_
// #define _TCP_CLIENT_H_

// #ifdef _WIN32
// #else
// #define SD_BOTH 0
// #endif

#include <iostream>
#include <vector>
#include <string>

#include <winsock2.h>
#include <ws2tcpip.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "CircleBuffer.h"

#pragma comment(lib, "Ws2_32.lib")

# define PI_NUMBER           3.1415926f

enum ROSCommands {
    globalPath = 1,
    requestedPose
};

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

    bool connectScoket();
    
    bool Reconnect();
    
    bool IsConnected();

    int recvBuffer();

    int recvCommand();

    int sendPath(const std::vector<geometry_msgs::PoseStamped> &poses);

    int requestPose(const geometry_msgs::Pose &pose);

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
    bool isConnected = false;

    int startId = 0;
    int getRecvMsgStart();
    float getFloat();
    void getFloat(float *fDst);
    void GetBytes(float v, uint8_t* dst, unsigned int offset = 0);

    
    int SendBuffer(char *buff, int len);
    void PrintBuffer(char *buff, int len) const;
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

/*
    Connect socket to server. Will recreate socket, then try to connect.
    Close socket if connection failed.
    Return: true if connected, false otherwise.
*/
bool TcpClient::connectScoket(){
    isConnected = false;

    // recreate socket
    socket_ = socket( pAddrInfo->ai_family, pAddrInfo->ai_socktype, pAddrInfo->ai_protocol);
    if (socket_ == INVALID_SOCKET) {
        std::cerr << "connectScoket(" << port_ << ") to create socket: " << WSAGetLastError() << std::endl;
        return false;
    }

    // try to connect socket
    if (SOCKET_ERROR == connect(socket_, pAddrInfo->ai_addr, (int)pAddrInfo->ai_addrlen)) {
        std::cerr << "connectScoket(" << port_ << ") failed to connect: " << WSAGetLastError() << std::endl;
        if (SOCKET_ERROR == closesocket(socket_)){
            std::cerr << "connectScoket(" << port_ << ") failed to close socket: " << WSAGetLastError() << std::endl;
        }
        socket_ = INVALID_SOCKET;
        return false;
    }

    isConnected = true;
    std::cout << "connectScoket(" << port_ << ") connected." << std::endl;
    return true;
}

/*
    Reconnect socket to server. Will close socket, then call bool TcpClient::connectScoket();
    Return: true if reconnected, false otherwise.
*/
bool TcpClient::Reconnect(){
    std::cout << "Reconnecting port: " << port_ << std::endl;

    // Close socket before reconnecting
    if (SOCKET_ERROR == closesocket(socket_)){
        std::cerr << "Reconnect(" << port_ << ") failed to close socket: " << WSAGetLastError() << std::endl;
    }

    // Try to connect socket
    if (!connectScoket())
        return false;
    
    isConnected = true;
    return true;
}


struct addrinfo* TcpClient::getAddrInfo() const {
        return pAddrInfo;
};

geometry_msgs::Vector3 TcpClient::getVector3(){
    geometry_msgs::Vector3 res;
    res.x = getFloat();
    res.y = getFloat();
    res.z = getFloat();
    return res;
}

geometry_msgs::Quaternion TcpClient::getQuaternion(){
    geometry_msgs::Quaternion res;
    float roll;
    getFloat(&roll);
    float pitch;
    getFloat(&pitch);
    float yaw;
    getFloat(&yaw);
    
    double cy = cos(yaw*PI_NUMBER/180.0f * 0.5);
    double sy = sin(yaw*PI_NUMBER/180.0f * 0.5);
    double cp = cos(pitch*PI_NUMBER/180.0f * 0.5);
    double sp = sin(pitch*PI_NUMBER/180.0f * 0.5);
    double cr = cos(roll*PI_NUMBER/180.0f * 0.5);
    double sr = sin(roll*PI_NUMBER/180.0f * 0.5);

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

void TcpClient::getFloat(float *fDst){
    circleBuf->GetData((uint8_t*)(fDst), 4);
}

int TcpClient::sendPath(const std::vector<geometry_msgs::PoseStamped> &poses){
    unsigned int buffSize = 2 + 1 + poses.size()*3*4;   // BeginBytes + cmdByte + posesSize*corrdinatesNumber*floatSize
    uint8_t *buff = new uint8_t[buffSize];
    buff[0] = 0xFF;
    buff[1] = 0xFF;
    buff[2] = (uint8_t)globalPath;
    unsigned int offset = 3;

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

    // std::cout << offset << ", " << buffSize << std::endl;
    // std::cout << "Buffer: ";
    // for (unsigned int j = 0; j < buffSize; ++j)
    //     std::cout << +buff[j] << " ";
    // std::cout << std::endl;
    // return (int)sendBuffer((char*) buff, buffSize);
    return 0;
}

int TcpClient::requestPose(const geometry_msgs::Pose &pose){
    unsigned int buffSize = 2 + 1 + 3*4;   // BeginBytes + cmdByte + corrdinatesNumber*floatSize
    uint8_t *buff = new uint8_t[buffSize];
    buff[0] = 0xFF;
    buff[1] = 0xFF;
    buff[2] = (uint8_t)requestedPose;
    unsigned int offset = 3;

    float f = (float)pose.position.x;
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&f);
    std::memcpy(buff + offset, bytes, 4);
    offset += 4;

    f = (float)pose.position.y;
    bytes = reinterpret_cast<uint8_t*>(&f);
    std::memcpy(buff + offset, bytes, 4);
    offset += 4;
    
    f = (float)pose.position.z;
    bytes = reinterpret_cast<uint8_t*>(&f);
    std::memcpy(buff + offset, bytes, 4);
    offset += 4;

    return SendBuffer((char*) buff, buffSize);
}

int TcpClient::SendBuffer(char *buff, int len){
    int sendResult = 0;
    if (send(socket_, buff, len, 0) == SOCKET_ERROR) {
        sendResult = WSAGetLastError();
        isConnected = false;
        std::cout << "sendBuffer error: " << sendResult << std::endl;
    }
    return sendResult;
}

int TcpClient::recvBuffer(){
    bool l = true;
    if (SOCKET_ERROR == ioctlsocket (socket_, FIONBIO, (unsigned long*) &l) ) {
        std::cerr << "ioctlsocket() error: " << WSAGetLastError() << std::endl;
        isConnected = false;
        return -1;
    }

    int recvLen = 0;
    if ( SOCKET_ERROR == (recvLen=recv(socket_, (char *)&recvbuf, recvbuflen, 0) )){
        int err = WSAGetLastError();
        switch (err) {
            case (WSAEWOULDBLOCK):{ // Resource temporarily unavailable.
                return 0;
            }
            default: // other errors
                std::cerr << "recv() error: " << err << std::endl;
                isConnected = false;
                return -1;
        }
    }

    return recvLen;
}

void TcpClient::PrintBuffer(char *buff, int len) const{
    std::cout << port_ << " buffer " << len << " bytes: ";
    for (int i = 0; i < len; ++i)
        std::cout << +(uint8_t)buff[i] << " ";
    std::cout << std::endl;
}

bool TcpClient::IsConnected(){
    // uint8_t *buff = new uint8_t[3];
    // buff[0] = 0xFF; buff[1] = 0xFF; buff[2] = 0x00;
    // SendBuffer((char*)buff, 3);
    return isConnected;
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

    uint8_t fhead = circleBuf->GetItem();
    uint8_t shead = circleBuf->PredictNext();
    if ((fhead == 255) && (shead == 255)){
        // std::cout << "Message head found" << std::endl;
        circleBuf->GetItem();
    } else {
        return -1;
    }
    
    // Get message length
    uint16_t msgLen = 0;
    circleBuf->GetData((uint8_t*)(&msgLen), 2);

    // Get message command code
    uint16_t cmdCode = 0;
    circleBuf->GetData((uint8_t*)(&cmdCode), 2);

    return cmdCode;
}