#ifndef ROBOTARM_MATHS_H
#define ROBOTARM_MATHS_H

#include <Dense>

#include "onltrack_handler.h"
#include "ik_wrapper.h"

// robot parameters
#define LERP_INTERP_FACTOR 0.01     // changes interpolation resolution (can try also 0.005)
#define SLERP_INTERP_FACTOR 0.001   // changes interpolation resolution (can try also 0.005)
#define POSITIONING_ACCURACY 0.02   // distance from ee to target that is considered negligible
#define ORIENTATION_ACCURACY 0.02   // difference between ee and target ori that is considered negligible
#define SCISSORS_LENGTH 0.3         // in meters
#define LOWPASS_ALPHA 0.2           // Low-pass filter parameter for increment smoothing

// Camera and toolEE position in robotEE frame
#define CAMERA_POS_X (-0.1025)      // -0.1025 m
#define CAMERA_POS_Y 0.0325         // 0.0325 m
#define CAMERA_POS_Z 0.037          // 0.037 m
#define TOOLEE_POS_X 0.03           // 0.03 m
#define TOOLEE_POS_Y 0.0            // 0 m
#define TOOLEE_POS_Z 0.26           // 0.26 m

// Circle point list parameters
#define CIRCLE_RADIUS       0.2
#define CIRCLE_POINT_NUM    10

double Transform_getDistanceBetweenPositions(const Hyundai_Data_t *eePos_worldFrame, const std::array<double, 6> target);

int Transform_getClosestPathPoint(const std::vector<std::array<double, 6>> &robotPath,
                                  const Hyundai_Data_t *eePos_worldFrame, const int pathLength);

bool Transform_getIncrements(const std::vector<std::array<double, 6>> &pathCartesian, const int step,
                             const Hyundai_Data_t *eePos_worldFrame, const double prev_increment[6],
                             double increment[6]);

Hyundai_Data_t Transform_ConvertFrameRobotEE2ToolEE(const Hyundai_Data_t* eePos_worldFrame);

Cartesian_Pos_t Transform_ConvertFrameTarget2World(const Cartesian_Pos_t* targetPos_camFrame,
                                                   const Hyundai_Data_t* eePos_worldFrame,
                                                   double scissors_length);

Cartesian_Pos_t Transform_ConvertFrameTarget2World(const Cartesian_Pos_t* targetPos_camFrame,
                                                   const Hyundai_Data_t* eePos_worldFrame);

Cartesian_Pos_t Transform_CalculatePositionIncrements(const Hyundai_Data_t *eePos_worldFrame,
                                                      const Cartesian_Pos_t *targetPos_worldFrame);

Cartesian_Pos_t Transform_CalculateOrientationIncrements(const Hyundai_Data_t *eePos_worldFrame,
                                                         const Cartesian_Pos_t *targetPos_worldFrame);

double Transform_CalcDistanceBetweenPoints(const Hyundai_Data_t *eePos_worldFrame,
                                            const Cartesian_Pos_t *targetPos_worldFrame);


bool Transform_CompareOrientations(double precision,
                                    const Hyundai_Data_t *eePos_worldFrame,
                                    const Cartesian_Pos_t *targetPos_worldFrame);

double Transform_Deg2Rad(const double value);
double Transform_Rad2Deg(const double value);

Eigen::Quaterniond Transform_ConvertEuler2Quat(const double rotZ, const double rotY, const double rotX);

Cartesian_Pos_t Transform_ConvertQuat2Euler(Eigen::Quaterniond q);

#endif //ROBOTARM_MATHS_H
