#ifndef ROBOTARM_ROBOT_API_H
#define ROBOTARM_ROBOT_API_H
#include "transform_calc.h"
#include "enet_handler.h"
#include "Matlab_ik_types.h"
#include <functional>




enum class Robot_Sequence_t {
    IDLE,
    APPROACH,
    FINAL_APPROACH,
    CUT,
    STORE,
    RETURN_TO_BASE,
    SWITCH_BASE
};

enum class Robot_Sequence_State_t {
    INIT,
    REQUESTED,
    COMPLETE,
    FAIL
};

enum class Robot_Sequence_Result_t {
    UNREACHABLE,
    FAILED,
    SUCCESS
};

using RobotSequenceCallback = std::function<void(Robot_Sequence_t, Robot_Sequence_State_t)>;

void RobotAPI_SetSequenceCallback(RobotSequenceCallback callback);

bool RobotAPI_TargetIsReachable(Cartesian_Pos_t *targetWorldFrame);

void RobotAPI_StartApproachSequence();
void RobotAPI_StartFinalApproachSequence();
void RobotAPI_StartCutSequence();
void RobotAPI_StartStoreSequence();
void RobotAPI_StartReturnToBaseSequence();
void RobotAPI_StartSwitchBaseSequence();

bool RobotAPI_IsApproachSequenceActive();
bool RobotAPI_IsFinalApproachSequenceActive();

void RobotAPI_EndSequence(Robot_Sequence_Result_t reason);

void RobotAPI_HandleEnetResponse(Enet_Cmd_t cmd);

void RobotAPI_ProcessAction();

#endif //ROBOTARM_ROBOT_API_H
