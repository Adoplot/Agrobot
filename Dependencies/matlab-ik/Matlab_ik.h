//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Matlab_ik.h
//
// Code generation for function 'Matlab_ik'
//

#ifndef MATLAB_IK_H
#define MATLAB_IK_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct struct0_T;

struct struct1_T;

// Function Declarations
extern void Matlab_checkCollision(const double currentConfig[6],
                                  boolean_T *isSelfColliding,
                                  double collPairs[2]);

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

/*
 * Draws a circle around target branch, calculates points on that circle
 * and sorts them based on distance from the robot's bodyToolEE
 * Input:
 * R                 - double, radius of the circle (in meters)
 * branchStart       - 1x3 array representing [x y z] position of the
 *                     cutting point
 * branchEnd         - 1x3 array representing [x y z] position of the
 *                     target branch end or any point on the branch in
 *                     the direction of the branch growth
 * numCirclePoints   - double, defines how many evenly spaced points on
 *                     the circle to output
 * pos_toolEE        - 1x3 array representing [x y z] position of the
 *                     robot's bodyToolEE (scissor's end effector)
 * Output:
 * sortedList        - Nx8 array of points on the circle with orientation
 *                     (Z toward center, Y inverted normal). Each point
 *                     represented by xyz position (col1:3), quaternion
 *                     (col4:7-wxyz) and distance to bodyToolEE (col8).
 *                     N is defined with numCirclePoints.
 * listLength        - double, represents the number of rows in
 *                     sortedList. Should be the same as numCirclePoints
 */
extern void Matlab_getSortedCirclePointList(
    double R, const double branchStart[3], const double branchEnd[3],
    double numCirclePoints, const double pos_toolEE[3],
    coder::array<double, 2U> &sortedList, double *listLength);

extern void Matlab_ik_initialize();

extern void Matlab_ik_terminate();

#endif
// End of code generation (Matlab_ik.h)
