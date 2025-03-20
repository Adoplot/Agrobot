#ifndef ROBOTARM_MATHS_H
#define ROBOTARM_MATHS_H

#include <Dense>

#include "onltrack_handler.h"

// robot parameters
#define LERP_INTERP_FACTOR 0.01     // changes interpolation resolution (can try also 0.005)
#define SLERP_INTERP_FACTOR 0.001   // changes interpolation resolution (can try also 0.005)
#define POSITIONING_ACCURACY 0.02   // distance from ee to target that is considered negligible
#define ORIENTATION_ACCURACY 0.02   // difference between ee and target ori that is considered negligible
#define SCISSORS_LENGTH 0.3         // in meters
#define CAMERA_POS_X 0.03           // 0.03 m
#define CAMERA_POS_Y 0.0            // 0 m
#define CAMERA_POS_Z 0.26           // 0.26 m

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



#endif //ROBOTARM_MATHS_H
