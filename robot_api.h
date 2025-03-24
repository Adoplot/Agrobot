#ifndef ROBOTARM_ROBOT_API_H
#define ROBOTARM_ROBOT_API_H
#include "transform_calc.h"
#include "enet_handler.h"
#include "Matlab_ik_types.h"
#include <functional>


//Solver parameters
#define SOLVER_MAX_ITERATIONS           1500
#define SOLVER_MAXTIME                  10
#define SOLVER_ENFORCE_JOINT_LIMITS     true
#define SOLVER_ALLOW_RANDOM_RESTARTS    true
#define SOLVER_STEP_TOLERANCE           1.0E-13
#define SOLVER_POSITION_TOLERANCE       0.2
#define SOLVER_ORIENTATION_TOLERANCE    0.1

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

void RobotAPI_InitSolverParameters(struct0_T *solverParameters);

#endif //ROBOTARM_ROBOT_API_H
