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
#define SOLVER_POSITION_TOLERANCE       0.01
#define SOLVER_ORIENTATION_TOLERANCE    0.1

//Positioning parameters
#define CAM_ANGLE_OFFSET    (20)    //(in DEG) offset angle for camera so tool do not obstruct view on the target branch
#define DIST_WEIGHT_POS     1       //weight for position in total distance calculation between bodyEE and Approach seq target
#define DIST_WEIGHT_ORI     0.5     //weight for orientation in total distance calculation between bodyEE and Approach seq target

//Trajectory parameters
#define PATH_VELOCITY       0.05       //vel=1 => 0.4 rad/s (23 deg/s)
#define PATH_STEP_TIME      0.005   //seconds

//Axis limits (in DEGREES)              Robot soft limits:
#define AXIS_MIN_A1         (-167)  //-170
#define AXIS_MIN_A2         (-52)   //-55
#define AXIS_MIN_A3         (-62)   //-65
#define AXIS_MIN_A4         (-187)  //-190
#define AXIS_MIN_A5         (-122)  //-125
#define AXIS_MIN_A6         (-357)  //-360

#define AXIS_MAX_A1         (167)   // 170
#define AXIS_MAX_A2         (177)   // 180
#define AXIS_MAX_A3         (210)   // 213
#define AXIS_MAX_A4         (187)   // 190
#define AXIS_MAX_A5         (92)    // 95
#define AXIS_MAX_A6         (357)   // 360

enum whichEE {
    ROBOTEE,
    TOOLEE
};

bool IK_getTrajectory(const double currentConfig[6], const double waypoint[6], const double velocity,
                      const double step_time, std::vector<std::array<double, 6>> &pathCartesian);

bool IK_InterpolatePath(const double q_start[6], const double q_end[6], const double velocity,
                        const double step_time, std::vector<std::array<double,6>> &pathConfig);

bool IK_AxisInLimits(const double axisValue, const int axisNum);

void IK_getWaypointsForApproach(const double branchStart[3], const double branchDir[3], const Hyundai_Data_t *eeCoords_worldFrame,
                                const double currentConfig[6], int *code, double qWaypoints[18]);

void IK_PrintWaypoints(const double qWaypoints[18]);

void IK_PrintSortedPointList(coder::array<double, 2U> *sortedList);

void IK_InitSolverParameters(struct0_T *solverParameters);

#endif //ROBOTARM_IK_WRAPPER_H
