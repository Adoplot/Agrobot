//
// Created by Pavel Maksimkin on 18/09/2024.
//

#ifndef ROBOTARM_ENET_HANDLER_H
#define ROBOTARM_ENET_HANDLER_H
#include "compv_handler.h"

typedef enum {
    ENET_UNDEFINED,
    ///< Requests
    ENET_INIT,      //program receives init from hyundai to detect pc address
    ENET_RETURN_TO_BASE,    //!!! if change smth, then correct it in Hyundai controller !!!
    ENET_CUT,
    ENET_STORE,
    ENET_SWITCH_BASE_NEXT, ///< Switch to next available base/configuration
    ENET_SWITCH_BASE_HOME, ///< Go to Home position
    ENET_SAFE_POSITION,

    ///< Responses
    ENET_RETURN_TO_BASE_COMPLETE,
    ENET_CUT_COMPLETE,
    ENET_STORE_COMPLETE,
    ENET_SWITCH_BASE_NEXT_COMPLETE,
    ENET_SWITCH_BASE_HOME_COMPLETE, ///< Robot is in Home position
    ENET_SWITCH_BASE_NO_BASE_LEFT, ///< No next base available
    ENET_ROBOT_CONFIGURATION, ///< Received robot axis configuration
    ENET_SAFE_POSITION_COMPLETE ///< Robot is in safe position for transportation
} Enet_Cmd_t;


typedef struct Enet_RecvStr_t {
    char init[6] = "init\012";      //program receives init from hyundai to detect pc address
    char return_to_base[25] = "return_to_base_complete\012";
    char cut[14] = "cut_complete\012";
    char store[16] = "store_complete\012";

    char switch_base_next_success[26] = "switch_base_next_success\012";
    char switch_base_home_success[26] = "switch_base_home_success\012";
    char switch_base_no_base_left[26] = "switch_base_no_base_left\012";
    char switch_base_fail[18] = "switch_base_fail\012";
    char safe_position_success[23] = "safe_position_success\012"; ///< robot moved to safe position successfully

    char robot_configuration[20] = "robot_configuration"; //do not use \012 here
    char started_execution[19] = "started_execution\012";
} Enet_RecvStr_t;

void Enet1_HandleCmd(char* buffer, long buf_len);

#endif //ROBOTARM_ENET_HANDLER_H
