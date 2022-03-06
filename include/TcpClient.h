#ifndef _TCP_CLIENT_H_
#define _TCP_CLIENT_H_

#ifdef _WIN32
#else
#define SD_BOTH 0
#endif

#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "Ws2_32.lib")

class TcpClient {
public:
    TcpClient(const PCSTR address, const PCSTR port);

    ~TcpClient();
    
    // init WSA
    bool init();

    // Create a SOCKET for connecting to server
    bool createSocket();

    bool connectScoket();
    
    bool Reconnect();
    
    bool isConnected();

    int recvBuffer();

    std::string recvLine();

    struct addrinfo* getAddrInfo() const;
private:
    WSADATA wsaData;
    const PCSTR port_, address_;
    struct addrinfo *pAddrInfo = NULL, hints;
    SOCKET socket_;

    const static int recvbuflen = 512;
    char recvbuf[recvbuflen];

    bool sendBuffer(char *buff, int len);
};

TcpClient::TcpClient(const PCSTR address, const PCSTR port):
    port_(port),
    address_(address) {

}

TcpClient::~TcpClient()
{
}

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

    std::cout << "received: " << len << " bytes." << std::endl;
    return 0;
}

std::string TcpClient::recvLine(){
    if (recvBuffer()){
        return "ERR";
    }
    std::string res = "";
    for (int i = 0; (i < recvbuflen) && (recvbuf[i] != '\n'); ++i){
        res += recvbuf[i];
    }
    return res;
}

#endif // _TCP_CLIENT_H_