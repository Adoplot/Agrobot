#ifndef ROBOTARM_MATHS_H
#define ROBOTARM_MATHS_H

#include <Dense>

#include "onltrack_handler.h"


Cartesian_Pos_t Transform_ConvertFrameTarget2World(const Cartesian_Pos_t* targetPos_camFrame,
                                                   const Hyundai_Data_t* eePos_worldFrame,
                                                   double scissors_length);

Cartesian_Pos_t Transform_CalculatePositionIncrements(const Hyundai_Data_t *eePos_worldFrame,
                                                      const Cartesian_Pos_t *targetPos_worldFrame);

Cartesian_Pos_t Transform_CalculateOrientationIncrements(const Hyundai_Data_t *eePos_worldFrame,
                                                         const Cartesian_Pos_t *targetPos_worldFrame);

double Transform_CalcDistanceBetweenPoints(const Hyundai_Data_t *eePos_worldFrame,
                                            const Cartesian_Pos_t *targetPos_worldFrame);

bool Transform_CompareOrientations(double precision,
                                    const Eigen::Quaterniond *q1,
                                    const Eigen::Quaterniond *q2);



#endif //ROBOTARM_MATHS_H
