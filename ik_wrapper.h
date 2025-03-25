#ifndef ROBOTARM_IK_WRAPPER_H
#define ROBOTARM_IK_WRAPPER_H

#include <functional>
#include "Matlab_ik_types.h"
#include "enet_handler.h"
#include "transform_calc.h"

void IK_InitSolverParameters(struct0_T *solverParameters);

#endif //ROBOTARM_IK_WRAPPER_H
