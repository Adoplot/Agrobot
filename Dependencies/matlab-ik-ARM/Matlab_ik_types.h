//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Matlab_ik_types.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 20-Mar-2025 13:24:41
//

#ifndef MATLAB_IK_TYPES_H
#define MATLAB_IK_TYPES_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include "coder_bounded_array.h"

// Type Definitions
struct struct0_T {
  double maxIterations;
  double maxTime;
  bool enforceJointLimits;
  bool allowRandomRestarts;
  double stepTolerance;
  double positionTolerance;
  double orientationTolerance;
};

struct struct2_T {
  coder::bounded_array<char, 11U, 2U> Type;
  coder::array<double, 2U> Violation;
};

struct struct1_T {
  double Iterations;
  double NumRandomRestarts;
  struct2_T ConstraintViolations[2];
  double ExitFlag;
  coder::bounded_array<char, 14U, 2U> Status;
};

#endif
//
// File trailer for Matlab_ik_types.h
//
// [EOF]
//
