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

#pragma comment(lib, "Ws2_32.lib")

class TcpClient {
public:
    TcpClient(const PCSTR address, const PCSTR port):
    port_(port),
    address_(address) {
        // circleBuf = new CircleBuffer(512);
    };

    TcpClient::~TcpClient() {
        // delete circleBuf;
    };
    
    // init WSA
    bool init();

    bool connectScoket();
    
    bool Reconnect();
    
    bool IsConnected();

    int recvBuffer();

    uint8_t* GetReceived(const int &size){
        uint8_t* res = new uint8_t[size];
        memcpy(res, recvbuf, (size_t)size);
        return res;
    }
    struct addrinfo* getAddrInfo() const;
    int SendBuffer(char *buff, int len);
private:
    WSADATA wsaData;
    const PCSTR port_, address_;
    struct addrinfo *pAddrInfo = NULL, hints;
    SOCKET socket_;

    const static int recvbuflen = 512;
    char recvbuf[recvbuflen];
    // CircleBuffer *circleBuf;
    bool isConnected = false;

    int startId = 0;
    int getRecvMsgStart();
    float getFloat();
    void getFloat(float *fDst);
    void GetBytes(float v, uint8_t* dst, unsigned int offset = 0);

    
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
 *Connect socket to server. Will recreate socket, then try to connect.
 *Close socket if connection failed.
 *Return: true if connected, false otherwise.
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
 *Reconnect socket to server. 
 *Will close socket, then call bool TcpClient::connectScoket();
 *Return: true if reconnected, false otherwise.
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

// int TcpClient::requestPose(const geometry_msgs::Pose &pose){
//     unsigned int buffSize = 2 + 1 + 3*4;   // BeginBytes + cmdByte + corrdinatesNumber*floatSize
//     uint8_t *buff = new uint8_t[buffSize];
//     buff[0] = 0xFF;
//     buff[1] = 0xFF;
//     buff[2] = (uint8_t)requestedPose;
//     unsigned int offset = 3;

//     float f = (float)pose.position.x;
//     uint8_t* bytes = reinterpret_cast<uint8_t*>(&f);
//     std::memcpy(buff + offset, bytes, 4);
//     offset += 4;

//     f = (float)pose.position.y;
//     bytes = reinterpret_cast<uint8_t*>(&f);
//     std::memcpy(buff + offset, bytes, 4);
//     offset += 4;
    
//     f = (float)pose.position.z;
//     bytes = reinterpret_cast<uint8_t*>(&f);
//     std::memcpy(buff + offset, bytes, 4);
//     offset += 4;

//     return SendBuffer((char*) buff, buffSize);
// }

int TcpClient::SendBuffer(char *buff, int len){
    int sendResult = 0;
    if (send(socket_, buff, len, 0) == SOCKET_ERROR) {
        sendResult = WSAGetLastError();
        isConnected = false;
        std::cout << "sendBuffer error: " << sendResult << std::endl;
        return -1;
    }
    
    std::cout << "Sended " << len << " bytes: ";
    for (int i = 0; i < len; ++i){
        std::cout << +(uint8_t)buff[i] << " ";
    }
    std::cout << std::endl;

    return sendResult;
}

int TcpClient::recvBuffer(){
    bool l = true;
    if (SOCKET_ERROR == ioctlsocket (socket_, FIONBIO, (unsigned long*) &l) ) {
        std::cerr << "ioctlsocket() error: " << WSAGetLastError() << std::endl;
        isConnected = false;
        return -1;
    }

    // Proccess socket.recv()
    int recvLen = 0;
    if ( SOCKET_ERROR == (recvLen = recv(socket_, (char *)&recvbuf, recvbuflen, 0)) ){
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

    // Add to circle buffer if something was received
    // if (recvLen > 0)
    //     circleBuf->AddData((uint8_t *)&recvbuf, recvLen);

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
