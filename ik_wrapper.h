#ifndef ROBOTARM_IK_WRAPPER_H
#define ROBOTARM_IK_WRAPPER_H

#include <functional>
#include "Matlab_ik_types.h"
#include "enet_handler.h"
#include "transform_calc.h"

//Solver parameters
#define SOLVER_MAX_ITERATIONS           1500
#define SOLVER_MAXTIME                  10
#define SOLVER_ENFORCE_JOINT_LIMITS     true
#define SOLVER_ALLOW_RANDOM_RESTARTS    true
#define SOLVER_STEP_TOLERANCE           1.0E-13
#define SOLVER_POSITION_TOLERANCE       0.02
#define SOLVER_ORIENTATION_TOLERANCE    0.1

void IK_getWaypointsForApproach(const double branchStart[3], const double branchDir[3], const Hyundai_Data_t *eeCoords_worldFrame,
                                const double currentConfig[6], int *code, double qWaypoints[18]);

void IK_InitSolverParameters(struct0_T *solverParameters);

void IK_PrintWaypoints(const double qWaypoints[18]);

void IK_PrintSortedPointList(coder::array<double, 2U> *sortedList);

#endif //ROBOTARM_IK_WRAPPER_H
