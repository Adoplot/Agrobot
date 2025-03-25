#include "Matlab_ik_types.h"
#include "Matlab_ik.h"
#include "connection_handler.h"
#include "robot_api.h"
#include <netinet/in.h>
#include "ik_wrapper.h"


void IK_InitSolverParameters(struct0_T *solverParameters){
    solverParameters->maxIterations          = SOLVER_MAX_ITERATIONS;
    solverParameters->maxTime                = SOLVER_MAXTIME;
    solverParameters->enforceJointLimits     = SOLVER_ENFORCE_JOINT_LIMITS;
    solverParameters->allowRandomRestarts    = SOLVER_ALLOW_RANDOM_RESTARTS;
    solverParameters->stepTolerance          = SOLVER_STEP_TOLERANCE;
    solverParameters->positionTolerance      = SOLVER_POSITION_TOLERANCE;
    solverParameters->orientationTolerance   = SOLVER_ORIENTATION_TOLERANCE;
}