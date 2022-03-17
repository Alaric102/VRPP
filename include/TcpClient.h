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
    globalPath = 1
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

    // Create a SOCKET for connecting to server
    bool createSocket();

    bool connectScoket();
    
    bool Reconnect();
    
    bool isConnected();

    int recvBuffer();

    int recvCommand();

    int sendPath(int cmd, const std::vector<geometry_msgs::PoseStamped> &poses);

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
    void getFloat(float *fDst);
    void GetBytes(float v, uint8_t* dst, unsigned int offset = 0);
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

void TcpClient::GetBytes(float v, uint8_t* dst, unsigned int offset){

}

int TcpClient::sendPath(int cmd, const std::vector<geometry_msgs::PoseStamped> &poses){
    unsigned int buffSize = 2 + 1 + poses.size()*3*4;   // BeginBytes + cmdByte + posesSize*corrdinatesNumber*floatSize
    uint8_t *buff = new uint8_t[buffSize];
    buff[0] = 0xFF;
    buff[1] = 0xFF;
    buff[2] = (uint8_t)cmd;
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

        std::cout << (float)poses[i].pose.position.x << ", " << 
            (float)poses[i].pose.position.y << ", " << 
            (float)poses[i].pose.position.z << std::endl;
        // std::cout << "f: " << f << ", bytes: ";
        // for (unsigned int j = 0; j < 4; ++j)
        //     std::cout << +bytes[j] << " ";
        // std::cout << std::endl;
    }

    std::cout << offset << ", " << buffSize << std::endl;
    std::cout << "Buffer: ";
    for (unsigned int j = 0; j < buffSize; ++j)
        std::cout << +buff[j] << " ";
    std::cout << std::endl;
    

    return (int)sendBuffer((char*) buff, buffSize);
}