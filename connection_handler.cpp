#include <fcntl.h>
#include <poll.h>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <iostream>

#include "connection_handler.h"
#include "compv_handler.h"
#include "enet_handler.h"
#include "onltrack_handler.h"

//ToDo: cant handle multiple tcp client connections
// if client1 connects, and then client2 connects, client2 is in queue, but buffer is being filled with client2 data
// when client2 disconects, and then client1 disconects, the buffer of client2 is printed and compv_socket_state
// is not set to CLIENT_DISCONNECTED
// not critical as we have only 1 client

using std::cout;
using std::cerr;
using std::endl;

Hyundai_Data_t eePos_worldFrame{};

static int sockfd_onltrack{-1};
static int sockfd_enet1{-1};
static int sockfd_tcp_listener{-1};
static int sockfd_tcp_client{-1};
static sockaddr_in sockaddr_onltrack{};
static sockaddr_in sockaddr_enet1{};
static sockaddr_in sockaddr_compv{};

// Set up pollfd structure for polling the socket
static struct pollfd poll_list[4];      //3 already assigned (+ 1 client to be added later)
static int active_client_count = 0;     //at the start no tcp clients are connected

static Udp_State_t enet1_socket_state = UDP_CLOSED;         //Todo: do we need it?
static Udp_State_t onltrack_socket_state = UDP_CLOSED;      //Todo: do we need it?
static Tcp_Connection_Status_t compv_socket_state = TCP_NOT_INITIALIZED;   //program begins with tcp disconnected

static Onltrack_State_t onltrack_state = ONLTRACK_OFF;


static void pollingSockets();
static void acceptNewClient();
static void handleConnectionEnet1();
static void handleConnectionOnltrack();
static void handleConnectionCompV();

static Received_Message_t receiveTcp(const int sockfd);
static Received_Message_t recvUdp(const int sockfd, void* buffer, size_t buffer_size);
static int initializeSocket(int port, bool socktype);
static int listenForClients(const int sockfd);


// Interface to get pointer to eePos_worldFrame //todo move to RobotAPI
Hyundai_Data_t* Connection_GetEePosWorldFrame() {
    return &eePos_worldFrame;
}


// Interface to set Onltrack state
void Connection_SetOnltrackState(Onltrack_State_t state) {
    onltrack_state = state;
}

// Interface to get Onltrack state
Onltrack_State_t Connection_GetOnltrackState() {
    return onltrack_state;
}


// Interface to get Compv socket state
Tcp_Connection_Status_t Connection_GetCompvSocketState() {
    return compv_socket_state;
}


// Interface to get sockfd value
int Connection_GetSockfd(SockType_t socktype) {
    switch (socktype) {
        case SOCKTYPE_ONLTRACK:
            return sockfd_onltrack;
        case SOCKTYPE_ENET1:
            return sockfd_enet1;
        case SOCKTYPE_TCP_LISTENER:
            return sockfd_tcp_listener;
        case SOCKTYPE_COMPV:
            return sockfd_tcp_client;
        default:
            cerr << "Connection_GetSockfd(): Unexpected socktype" << endl;
            return -1;
    }
}


// Interface to get socket address struct
sockaddr_in Connection_GetSockAddr(SockType_t socktype) {
    switch (socktype) {
        case SOCKTYPE_ONLTRACK:
            return sockaddr_onltrack;
        case SOCKTYPE_ENET1:
            return sockaddr_enet1;
        case SOCKTYPE_TCP_LISTENER:
            cerr << "Connection_GetSockAddr(): unexpected socktype, why do you need addr of tcp listener?" << endl;
            return {};
        case SOCKTYPE_COMPV:
            return sockaddr_compv;
        default:
            cerr << "Connection_GetSockfd(): Unexpected socktype" << endl;
            return {};
    }
}


// Inits async udp and tcp sockets, inits pollfd structure
void Connection_Init(){
    std::cout << "Starting server...\n";

    // Initialize sockets
    sockfd_onltrack = initializeSocket(PORT_ONLTRACK, TYPE_UDP);
    if(sockfd_onltrack >= 0) {
        onltrack_socket_state = UDP_ACTIVE;        //if init is successful then udp is active
    }
    sockfd_enet1 = initializeSocket(PORT_ENET1, TYPE_UDP);
    if(sockfd_enet1 >= 0) {
        enet1_socket_state = UDP_ACTIVE;        //if init is successful then udp is active
    }
    sockfd_tcp_listener = initializeSocket(PORT_TCP, TYPE_TCP);
    if(sockfd_tcp_listener >= 0) {
        compv_socket_state = TCP_CLIENT_DISCONNECTED;  //after init server listen()->LISTENING and accept()->CONNECTED
    }

    // Listen for clients. If fails - exit program.
    listenForClients(sockfd_tcp_listener);

    // Prepare pollfd struct for polling sockets
    poll_list[POLL_LIST_ONLTRACK].fd = sockfd_onltrack;         // OnlTrack poll
    poll_list[POLL_LIST_ONLTRACK].events = POLLIN;              // Wait for incoming data (POLLIN)
    poll_list[POLL_LIST_ENET1].fd = sockfd_enet1;               // ENET1 poll
    poll_list[POLL_LIST_ENET1].events = POLLIN;                 // Wait for incoming data (POLLIN)
    poll_list[POLL_LIST_TCP_LISTENER].fd = sockfd_tcp_listener; // TCP poll
    poll_list[POLL_LIST_TCP_LISTENER].events = POLLIN;          // Wait for incoming data (POLLIN)
}


//Gets called in while loop
void Connection_ProcessAction(){
    pollingSockets();
    handleConnectionOnltrack();
    handleConnectionEnet1();
    handleConnectionCompV();
}


//Polls all sockets, catches poll() errors
static void pollingSockets() {
    // Poll for events
    int poll_count = poll(poll_list, (POLL_LIST_FD_COUNT + active_client_count), TIMEOUT_MS);

    // Check for polling errors
    if (poll_count == -1) {
        perror("Error during poll");
        printf("The last error message is: %s\n", strerror(errno));
        close(sockfd_enet1);
        close(sockfd_onltrack);
        close(sockfd_tcp_listener);
        enet1_socket_state = UDP_CLOSED;
        onltrack_socket_state = UDP_CLOSED;
        compv_socket_state = TCP_NOT_INITIALIZED;
    }
#if TIMEOUT_MS != 0
    if (poll_count == 0) {
        if (onltrack_state == ONLTRACK_ON) {
            //cout << "Poll timed out. No data received. Sending one more OnlTrack P message" << std::endl;
            Onltrack_AnswerHandle(&eePos_worldFrame, sizeof(eePos_worldFrame));
        }
    }
#endif
}


// Manages communication with Controller via OnLTrack
static void handleConnectionOnltrack(){
    Received_Message_t received_message;

    //Check for new Onltrack messages available
    if (poll_list[POLL_LIST_ONLTRACK].revents == POLLIN) {
        received_message = recvUdp(sockfd_onltrack, &eePos_worldFrame, sizeof(eePos_worldFrame));
        sockaddr_onltrack = received_message.server_address;

        if (received_message.received_bytes_count > 0) {
            Onltrack_AnswerHandle(&eePos_worldFrame, received_message.received_bytes_count);
        }
    }
}


// Manages communication with Hyundai Controller via ENET1
static void handleConnectionEnet1(){
    char buffer[MAX_BUFFER_SIZE]{};
    Received_Message_t received_message;

    //Check for new ENET1 messages available
    if (poll_list[POLL_LIST_ENET1].revents == POLLIN) {
        received_message = recvUdp(sockfd_enet1, &buffer, sizeof(buffer));
        sockaddr_enet1 = received_message.server_address;

        if (received_message.received_bytes_count > 0) {
            Enet1_HandleCmd(buffer, received_message.received_bytes_count);
        }
    }
}


// Manages communication with Computer Vision via TCP
static void handleConnectionCompV() {
    bool newDataAvailable;
    Received_Message_t received_message;

    // Check if any client wants to connect
    if ((poll_list[POLL_LIST_TCP_LISTENER].revents == POLLIN) &&
        (compv_socket_state == TCP_CLIENT_DISCONNECTED)) {

        acceptNewClient();  //accept new client
        }

    // Check if received any data from client
    if (compv_socket_state == TCP_CLIENT_CONNECTED) {
        if (poll_list[POLL_LIST_TCP_CLIENT].revents == POLLIN){
            // if data is available - recv
            received_message = receiveTcp(sockfd_tcp_client);
            sockaddr_compv = received_message.server_address;

            if (received_message.errnum > 0) {
                //cout << received_message.data;
                Compv_HandleCmd(&received_message.data);
            }
        }
    }
}


// Accepts new client, changes active_client_count
static void acceptNewClient() {
    // Accept new client
    // If accept fails with EWOULDBLOCK, then program have accepted all clients.

    sockfd_tcp_client = accept(sockfd_tcp_listener, nullptr, nullptr);

    if (sockfd_tcp_client < 0) {
        if (errno != EWOULDBLOCK) {
            perror("  accept() failed");
            printf("The last error message is: %s\n", strerror(errno));
            compv_socket_state = TCP_CLIENT_DISCONNECTED;
            // ToDo: Any other failure on accept will cause program to end the server.
        }
        if (errno == EWOULDBLOCK) {
            perror("  accept() all clients accepted");
            printf("The last error message is: %s\n", strerror(errno));
        }
    }
    else {
        // Set socket to non-blocking mode
        int flags = fcntl(sockfd_tcp_client, F_GETFL, 0);
        fcntl(sockfd_tcp_client, F_SETFL, flags | O_NONBLOCK);

        // Add the new incoming connection to the poll_list
        printf("  New incoming connection, sockfd: %d\n", sockfd_tcp_client);
        poll_list[POLL_LIST_TCP_CLIENT].fd = sockfd_tcp_client;
        poll_list[POLL_LIST_TCP_CLIENT].events = POLLIN;
        active_client_count = 1;        //add +1 client, so poll() could process it
        compv_socket_state = TCP_CLIENT_CONNECTED;
    }
}


// Sends buffer via TCP
// Returns: number of bytes sent or -1 for error
long Connection_SendTcp(const int sockfd, const std::string *msg){
    const char *buffer = msg->c_str();
    long send_bytes = send(sockfd, (void*)buffer, strlen(buffer), MSG_NOSIGNAL);
    if (send_bytes == SEND_ERR){
        perror("sendTCP(): send error\n");
        printf("The last error message is: %s\n", strerror(errno));
        // If send error occurs, client is disconnected
        active_client_count = 0;
        compv_socket_state = TCP_CLIENT_DISCONNECTED;
        poll_list[POLL_LIST_TCP_CLIENT].fd = -1;
        close(sockfd);
    }
    return send_bytes;
}


// Receives buffer via TCP
// Returns: number of bytes sent
static Received_Message_t receiveTcp(const int sockfd){
    Received_Message_t received_message;
    char buffer[MAX_BUFFER_SIZE];
    bzero(buffer, MAX_BUFFER_SIZE);

    cout << "recvTCP(): sockfd = " << sockfd << endl;

    long recv_bytes = recv(sockfd, buffer, MAX_BUFFER_SIZE-1, 0);

    if (recv_bytes == RECV_ERR) {
        if (errno == EWOULDBLOCK) {
            perror("recvTCP(): recv error EWOULDBLOCK\n");
        }
        else {
            perror("recvTCP(): recv error\n");
            printf("The last error received_message is: %s\n", strerror(errno));
            // client is disconnected. Redundant if exit() is performed right away
            //active_client_count = 0;
            //compv_socket_state = TCP_CLIENT_DISCONNECTED;
            close(sockfd);
            exit(EXIT_FAILURE);
        }
    }
    if (recv_bytes == RECV_END) {
        perror("TCP server: recv returned 0, closing socket");
        received_message.errnum = RECV_END;
        // client has disconnected
        active_client_count = 0;
        compv_socket_state = TCP_CLIENT_DISCONNECTED;
        close(sockfd);
        poll_list[POLL_LIST_TCP_CLIENT].fd = -1;
    }

    received_message.data = buffer;
    return received_message;
}


// Sends buffer via UDP, displays errors if occur
// Returns: err number, received bytes count, client address
long Connection_SendUdp(const int sockfd, const sockaddr_in remote_addr, const void *buffer, size_t buffer_size) {
    socklen_t addr_len = sizeof(remote_addr);
    long send_bytes = sendto(sockfd, buffer, buffer_size, 0,
                            (sockaddr*)&remote_addr, addr_len);

    if (send_bytes == SEND_ERR) {
        perror("Connection_SendUdp(): send error\n");
        printf("The last error message is: %s\n", strerror(errno));
    }
    if (send_bytes == SEND_END) {
        perror("Connection_SendUdp(): send end\n");
        printf("The last error message is: %s\n", strerror(errno));
    }

    return send_bytes;  // Return number on bytes sent, or error value
}


// Receives buffer via UDP, does NOT return std::string in message.data, displays errors if occur
// Returns: err number, received bytes count, client address, buffer data as std::string
static Received_Message_t recvUdp(const int sockfd, void* buffer, size_t buffer_size) {
    sockaddr_in remote_addr{};
    socklen_t addr_len = sizeof(remote_addr);
    Received_Message_t received_message;

    long recv_bytes = recvfrom(sockfd, (char *)buffer, buffer_size, 0,
                              (struct sockaddr*)&remote_addr, &addr_len);

    if (recv_bytes == RECV_ERR) {
        perror("UDP server recv error\n");
        printf("The last error received_message is: %s\n", strerror(errno));
        received_message.errnum = errno; // Store the error number
    } else if (recv_bytes == RECV_END) {
        std::cout << "UDP server recv end\n";
        received_message.errnum = RECV_END;
    } else if (recv_bytes >0) {
        received_message.errnum = RECV_OK;
    } else {
        std::cout << "UDP server recv unexpected behaviour. Exiting execution...\n";
        exit(EXIT_FAILURE); //ToDo: handle error?
    }

    received_message.server_address = remote_addr;
    received_message.received_bytes_count = recv_bytes;

    return received_message;
}


// Initializes UDP and TCP sockets, binds.
// Returns: sockfd
static int initializeSocket(int port, bool socktype){
    int sockfd{-1};
    sockaddr_in serv_addr{};

    //Create socket
    if (socktype == TYPE_TCP) {
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) {
            perror("Error: Failed to initialize TCP socket.\n");
            exit(1);
        }
    }
    if (socktype == TYPE_UDP) {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            perror("Error: Failed to initialize UDP socket.\n");
            exit(1);
        }
    }

    // Set socket to non-blocking mode
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    // Enable socket reuse
    int opt = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt");
        exit(1);
    }

    // Bind socket
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family        = AF_INET;
    serv_addr.sin_port      = htons(port);
    serv_addr.sin_addr.s_addr   = INADDR_ANY;
    if(bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0){
        perror("Error: Failed to bind socket to address.\n");
        exit(1);
    }

    return sockfd;
}

// Listens for new clients
// Closes socket if error occured
// Returns: 0 on success, -1 at errors
static int listenForClients(const int sockfd) {
    int rc;

    rc = listen(sockfd, MAX_CLIENT_COUNT);

    if (rc == LISTEN_SUCCESS) {
        std::cout << "TCP server: start listening on port " << PORT_TCP << "\n";
    }
    if (rc == LISTEN_ERR)
    {
        perror("listen() failed with err: ");
        printf("The last error message is: %s\n", strerror(errno));
        close(sockfd);
        exit(EXIT_FAILURE); //if listen fails in program init, then program will not work anyway
    }
    return rc;
}