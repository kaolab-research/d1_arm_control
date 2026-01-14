#ifndef ARM_SERVER_H
#define ARM_SERVER_H

#include <sys/socket.h> 
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <cstring> 
#include <vector> 
#include <iostream>

class D1ArmController;
class ArmServer 
{
public: 
    ArmServer(const std::string& host="192.168.123.10", int port=5555)
    : host_(host), port_(port), sock_(-1), connected_(false) {}
    ~ArmServer() { disconnect(); }

    bool connect();
    void disconnect();
    bool handle_request(D1ArmController& controller);

    bool is_connected() { return connected_; }


private: 
    std::string host_;
    int port_; 
    int sock_; 
    bool connected_; 
};

#endif // ARM_SERVER_H