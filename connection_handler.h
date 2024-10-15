#ifndef ROBOTARM_CONNECTION_HANDLER_H
#define ROBOTARM_CONNECTION_HANDLER_H

#include <iostream>
#include <netinet/in.h>

#include "onltrack_handler.h"


//debug macros
#define DEBUG_JSON_ELEMENT
#define DEBUG_UDP_MESSAGES
//#define DEBUG_QUATERNIONS
//#define DEBUG_H_MATRICES
//#define DEBUG_LERP              //print res and incr XYZ values (1) or not (any)

// robot parameters
#define LERP_INTERP_FACTOR 0.01     // changes interpolation resolution (can try also 0.005)
#define SLERP_INTERP_FACTOR 0.001   // changes interpolation resolution (can try also 0.005)
#define POSITIONING_ACCURACY 0.02   // distance from ee to target that is considered negligible
#define SCISSORS_LENGTH 0.3         // in meters
#define CAMERA_POS_X 0.0            // -0.04 m
#define CAMERA_POS_Y 0.0            // 0.033 m

// poll() macros
#define POLL_LIST_FD_COUNT 3        //number of file descriptors(sockets) for poll_list structure

#define POLL_LIST_ONLTRACK 0        //ToDo: convert these 4 to enum
#define POLL_LIST_ENET1 1
#define POLL_LIST_TCP_LISTENER 2
#define POLL_LIST_TCP_CLIENT 3      //if more sockets are added, then update POLL_LIST_FD_COUNT

#define TIMEOUT_MS 5                //in msec
#define MAX_CLIENT_COUNT 1

// socket macros
#define PORT_TCP 62222
#define PORT_ONLTRACK 9569
#define PORT_ENET1 7777
#define MAX_BUFFER_SIZE 8192
#define TYPE_UDP 0
#define TYPE_TCP 1

// send recv macros
#define RECV_OK 1
#define RECV_END 0
#define RECV_ERR (-1)
#define SEND_ERR (-1)
#define SEND_END 0
#define LISTEN_SUCCESS 0
#define LISTEN_ERR (-1)


#define PRINT_SEND 0
#define PRINT_RECV 1
#define TARGET_2_WORLD 0
#define CAM_2_WORLD 1

typedef enum {
    TCP_NOT_INITIALIZED,
    TCP_CLIENT_CONNECTED,
    TCP_CLIENT_DISCONNECTED
} Tcp_Connection_Status_t;


typedef enum {
    UDP_ACTIVE,
    UDP_CLOSED
} Udp_State_t;


typedef enum {
    TCP_DATA_NOT_AVAILABLE, //todo: change to NOT_INITIALISED?
    TCP_DATA_NEW_AVAILABLE,
    TCP_DATA_OLD
} Tcp_Data_t;


typedef enum {
    ONLTRACK_ON,
    ONLTRACK_OFF,
} Onltrack_State_t;


typedef enum {
    SOCKTYPE_ONLTRACK,
    SOCKTYPE_ENET1,
    SOCKTYPE_TCP_LISTENER,
    SOCKTYPE_COMPV
} SockType_t;


typedef struct{
    int errnum{RECV_OK};
    long received_bytes_count{};
    sockaddr_in server_address{};
    std::string data{};
} Received_Message_t;


Hyundai_Data_t* Connection_GetEePosWorldFrame();
void Connection_SetOnltrackState(Onltrack_State_t state);
Onltrack_State_t Connection_GetOnltrackState();
Tcp_Connection_Status_t Connection_GetCompvSocketState();
int Connection_GetSockfd(SockType_t socktype);
sockaddr_in Connection_GetSockAddr(SockType_t socktype);

void Connection_Init();
void Connection_ProcessAction();

long Connection_SendTcp(const int sockfd, const std::string *msg);
long Connection_SendUdp(const int sockfd, const sockaddr_in remote_addr, const void *buffer, size_t buffer_size);

#endif //ROBOTARM_CONNECTION_HANDLER_H
