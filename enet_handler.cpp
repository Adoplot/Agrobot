#include <iostream>
#include <cstring>
#include <netinet/in.h>

#include "nlohmann/json.hpp"

#include "enet_handler.h"
#include "connection_handler.h"
#include "robot_api.h"

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
    if (strncmp(buffer, enet_recv_str.switch_base_home_success, buf_len) == 0) {
        return ENET_SWITCH_BASE_HOME_COMPLETE;
    }
    if (strncmp(buffer, enet_recv_str.switch_base_next_success, buf_len) == 0) {
        return ENET_SWITCH_BASE_NEXT_COMPLETE;
    }
    if (strncmp(buffer, enet_recv_str.switch_base_no_base_left, buf_len) == 0) {
        return ENET_SWITCH_BASE_NO_BASE_LEFT;
    }

    if (strncmp(buffer, enet_recv_str.safe_position_success, buf_len) == 0) {
        return ENET_SAFE_POSITION_COMPLETE;
    }

    if(strncmp(buffer, enet_recv_str.robot_configuration, strlen(enet_recv_str.robot_configuration)) == 0){
        return ENET_ROBOT_CONFIGURATION;
    }

    return ENET_UNDEFINED;
}


// Handles cmd received from Hyundai controller
void Enet1_HandleCmd(char* buffer, long buf_len){
    std::string string_send{};

    Enet_Cmd_t enet1Cmd = getEnet1CmdType(buffer, buf_len);

    RobotAPI_HandleEnetResponse(enet1Cmd, buffer, buf_len);
}