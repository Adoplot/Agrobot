#ifndef ROBOTARM_ROBOT_API_H
#define ROBOTARM_ROBOT_API_H
#include "transform_calc.h"

bool RobotAPI_TargetIsReachable(Cartesian_Pos_t *targetWorldFrame);

void RobotAPI_SendCmd(const int sockfd_enet, const sockaddr_in remoteAddr_enet, const void *buffer, size_t buffer_size);

#endif //ROBOTARM_ROBOT_API_H
