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
    SWITCH_BASE,
    SAFE_POSITION,
    GO_HOME
};

enum class Robot_Sequence_State_t {
    INIT,
    REQUESTED,
    COMPLETE,
    FAIL
};

enum class Robot_Sequence_Result_t {
    INIT,
    BUSY, ///< Robot is busy
    UNREACHABLE, ///< Target is unreachable
    BASE_END, ///< No more bases available
    SUCCESS ///< Finished successfully
};

enum class Robot_Comm_State_t {
    INIT,
    IDLE,
    WAITING_FOR_RESPONSE,
    RESPONSE_RECEIVED,
    TIMED_OUT,
    SUCCESS,
    FAIL,
    END
};
bool RobotAPI_StartApproachSequence();
bool RobotAPI_StartFinalApproachSequence();
bool RobotAPI_StartCutSequence();
bool RobotAPI_StartStoreSequence();
bool RobotAPI_StartReturnToBaseSequence();
bool RobotAPI_StartSwitchBaseSequence();
bool RobotAPI_StartGoHomeSequence();
bool RobotAPI_StartSafePositionSequence();

bool RobotAPI_IsApproachSequenceActive();
bool RobotAPI_IsFinalApproachSequenceActive();
bool RobotAPI_IsTargetReachable(Target_Parameters_t *targetParameters_worldFrame, Hyundai_Data_t *eePos_worldFrame);

void RobotAPI_EndSequence(Robot_Sequence_Result_t reason);

double* RobotAPI_GetCurrentConfig();

void RobotAPI_HandleEnetResponse(Enet_Cmd_t cmd, char* buffer, long buf_len);

void RobotAPI_SetPath(const std::vector<std::array<double, 6>> &pathCartesian);
std::vector<std::array<double, 6>> RobotAPI_GetPathCopy();
void RobotAPI_ClearPath();

Robot_Sequence_t RobotAPI_GetSequence();
Robot_Sequence_State_t RobotAPI_GetSequenceState();
Robot_Sequence_Result_t RobotAPI_GetSequenceResult();
void RobotAPI_ResetSequenceData();
void RobotAPI_ProcessAction();

#endif //ROBOTARM_ROBOT_API_H
