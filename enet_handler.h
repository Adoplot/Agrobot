//
// Created by Pavel Maksimkin on 18/09/2024.
//

#ifndef ROBOTARM_ENET_HANDLER_H
#define ROBOTARM_ENET_HANDLER_H
#include "compv_handler.h"

typedef enum {
    ENET_UNDEFINED,
    ENET_INIT,      //program receives init from hyundai to detect pc address
    ENET_RETURN_TO_BASE,    //!!! if change smth, then correct it in Hyundai controller !!!
    ENET_CUT,
    ENET_STORE,
    ENET_RETURN_TO_BASE_COMPLETE,
    ENET_CUT_COMPLETE,
    ENET_STORE_COMPLETE,
    ENET_SWITCH_BASE,
    ENET_SWITCH_BASE_COMPLETE,
    ENET_ROBOT_CONFIGURATION ///< Received robot axis configuration
} Enet_Cmd_t;


typedef struct Enet_RecvStr_t {
    char init[6] = "init\012";      //program receives init from hyundai to detect pc address
    char return_to_base[25] = "return_to_base_complete\012";
    char cut[14] = "cut_complete\012";
    char store[16] = "store_complete\012";
    char switch_base[22] = "switch_base_complete\012";
    char robot_configuration[20] = "robot_configuration"; //do not use \012 here
} Enet_RecvStr_t;

void Enet1_HandleCmd(char* buffer, long buf_len);

#endif //ROBOTARM_ENET_HANDLER_H
