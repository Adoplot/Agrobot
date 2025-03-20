//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Matlab_ik.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 20-Mar-2025 13:24:41
//

#ifndef MATLAB_IK_H
#define MATLAB_IK_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct struct0_T;

struct struct1_T;

// Function Declarations
extern void Matlab_checkCollision(const double currentConfig[6],
                                  bool *isSelfColliding, double collPairs[2]);

extern void Matlab_getForwardKinematics(const double currentConfig[6],
                                        int whichEE, double robotSE3[16],
                                        double pos[3], double ori[3]);

extern void Matlab_getGikCut(const double currentConfig[6],
                             const double target_Apr[8],
                             const double targetPos_Fin[3],
                             const struct0_T *solverParameters,
                             double *exitCode, struct1_T *solutionInfoFin,
                             double qWaypoint[6]);

extern void Matlab_getGikFull(const double currentConfig[6],
                              const double target_Apr[8],
                              const double targetPos_Fin[3],
                              const struct0_T *solverParameters,
                              double *exitCode, struct1_T *solutionInfoApr,
                              double qWaypoints[18]);

extern void Matlab_getSortedCirclePointList(
    double R, const double branchStart[3], const double branchEnd[3],
    double numCirclePoints, const double pos_toolEE[3],
    coder::array<double, 2U> &sortedList, double *listLength);

extern void Matlab_ik_initialize();

extern void Matlab_ik_terminate();

#endif
//
// File trailer for Matlab_ik.h
//
// [EOF]
//
