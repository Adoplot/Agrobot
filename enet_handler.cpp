#include <iostream>
#include <cstring>
#include <netinet/in.h>

#include "nlohmann/json.hpp"

#include "enet_handler.h"
#include "connection_handler.h"

using std::cout;
using std::cerr;
using std::endl;

static Enet_RecvStr_t enet_recv_str;

static Enet_Cmd_t getEnet1CmdType(char* buffer, long buf_len);


// Compares received buffer with cmd c-string identifying received ENET1 cmd
// Returns: ENET1 cmd enum
static Enet_Cmd_t getEnet1CmdType(char* buffer, long buf_len){

    if (strncmp(buffer, enet_recv_str.init, buf_len) == 0) {
        return ENET_INIT;       //program receives init from hyundai to detect pc address
    }
    if (strncmp(buffer, enet_recv_str.return_to_base, buf_len) == 0) {
        return ENET_RETURN_TO_BASE_COMPLETE;
    }
    if (strncmp(buffer, enet_recv_str.cut, buf_len) == 0) {
        return ENET_CUT_COMPLETE;
    }
    if (strncmp(buffer, enet_recv_str.store, buf_len) == 0) {
        return ENET_STORE_COMPLETE;
    }

    return ENET_UNDEFINED;
}


// Handles cmd received from Hyundai controller
void Enet1_HandleCmd(char* buffer, long buf_len){
    // Get socket data for Compv
    int sockfd_compv = Connection_GetSockfd(SOCKTYPE_COMPV);
    sockaddr_in sockaddr_compv = Connection_GetSockAddr(SOCKTYPE_COMPV);

    // Declare jsons and strings to send
    nlohmann::json json_send{};
    std::string string_send{};

    Enet_Cmd_t enet1Cmd = getEnet1CmdType(buffer, buf_len);

    switch(enet1Cmd){
        case ENET_UNDEFINED: {
            cerr << "ENET_UNDEFINED: can't recognise received ENET1 cmd" << endl;
            // ToDo: handle when cmd in not recognised?
            break;
        }
        case ENET_INIT: {
            cout << "ENET1 received <init>" << endl;
            //program receives init from hyundai at the start to detect pc address automatically
            break;
        }
        case ENET_RETURN_TO_BASE_COMPLETE: {
            cout << "ENET1 received <return_to_base_complete>" << endl;
            json_send["request"] = "RETURN_TO_BASE";
            json_send["status"] = "COMPLETE";
            string_send = json_send.dump();
            Connection_SendTcp(sockfd_compv, &string_send);
            break;
        }

        case ENET_CUT_COMPLETE:
            cout << "ENET1 received <cut_complete>" << endl;
            json_send["request"] = "CUT";
            json_send["status"] = "COMPLETE";
            string_send = json_send.dump();
            Connection_SendTcp(sockfd_compv, &string_send);
            break;

        case ENET_STORE_COMPLETE:
            cout << "ENET1 received <store_complete>" << endl;
            json_send["request"] = "STORE";
            json_send["status"] = "COMPLETE";
            string_send = json_send.dump();
            Connection_SendTcp(sockfd_compv, &string_send);
        break;

        default:
            cerr << "enet1Cmd: unexpected enet1Cmd value" << endl;
            break;
    }
}