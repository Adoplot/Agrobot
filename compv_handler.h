#ifndef ROBOTARM_COMPV_HANDLER_H
#define ROBOTARM_COMPV_HANDLER_H

#include <iostream>

#include "onltrack_handler.h"

#define COMPV_ANSW_UNREACHABLE "UNREACHABLE"

#define COMPV_REQUEST_SET_POSITION "SET_POS"
#define COMPV_ANSW_COMPLETE "COMPLETE"
#define COMPV_ANSW_IN_PROGRESS "IN_PROGRESS"
typedef enum {
    REQ_INVALID_JSON, //TODO: not used?
    REQ_INVALID,
    REQ_SET_POS,
    REQ_RETURN_TO_BASE,
    REQ_CUT,
    REQ_STORE
} CompV_Request_t;
/*
typedef struct {
    char return_to_base[15] = "return_to_base";
    char cut[4] = "cut";
    char store[6] = "store";
} Enet1_Cmd_t;
*/

void CompV_SetTargetPosWorldFrame(Cartesian_Pos_t new_value);
void CompV_SetTargetPosCamFrame(Cartesian_Pos_t new_value);
Cartesian_Pos_t* Compv_GetTargetPosCamFrame();
Cartesian_Pos_t* Compv_GetTargetPosWorldFrame();

void Compv_HandleCmd(const std::string* data);

#endif //ROBOTARM_COMPV_HANDLER_H
