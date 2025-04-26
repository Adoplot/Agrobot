#ifndef ROBOTARM_COMPV_HANDLER_H
#define ROBOTARM_COMPV_HANDLER_H

#include <iostream>

#include "onltrack_handler.h"


typedef enum {
    COMPV_REQ_INVALID_JSON, //TODO: not used?
    COMPV_REQ_INVALID,
    COMPV_REQ_SYNC_TARGETS,
    COMPV_REQ_SET_POS,
    COMPV_REQ_FINAL_APPROACH,
    COMPV_REQ_CUT,
    COMPV_REQ_STORE,
    COMPV_REQ_RETURN_TO_BASE,
    COMPV_REQ_SWITCH_BASE,
    COMPV_REQ_GO_HOME
} CompV_Request_t;
/*
typedef struct {
    char return_to_base[15] = "return_to_base";
    char cut[4] = "cut";
    char store[6] = "store";
} Enet1_Cmd_t;
*/
void TEST_handleSetPositionRequest(); //ToDo: delete
void TEST_handleFinalApproachRequest(); //toDo: delete

void CompV_SetTargetPosWorldFrame(Cartesian_Pos_t new_value);
void CompV_SetTargetPosCamFrame(Cartesian_Pos_t new_value);
Cartesian_Pos_t* Compv_GetTargetPosCamFrame();
Cartesian_Pos_t* Compv_GetTargetPosWorldFrame();

void Compv_HandleCmd(const std::string* data);
void CompV_Init();

#endif //ROBOTARM_COMPV_HANDLER_H
