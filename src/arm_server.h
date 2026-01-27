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
    : host_(host), port_(port), client_sock_(-1), server_sock_(-1), listening_(false) {}

    bool start_listening();
    bool accept_connection();
    bool handle_request(D1ArmController& controller);

private: 
    std::string host_;
    int port_; 
    int server_sock_; 
    int client_sock_;
    bool listening_; 
};

#endif // ARM_SERVER_H