#include <netinet/in.h>
#include "robot_api.h"
#include "connection_handler.h"

using std::cout;
using std::cerr;
using std::endl;

static bool withinRobotsKinematics(Cartesian_Pos_t *targetWorldFrame);
static bool withinRobotsWorkspace(Cartesian_Pos_t *targetWorldFrame);

bool RobotAPI_TargetIsReachable(Cartesian_Pos_t *targetWorldFrame){
    if(withinRobotsKinematics(targetWorldFrame)){
        if(withinRobotsWorkspace(targetWorldFrame)){
            return true;
        } else {
            cout << "Robot API: Target unreachable" << endl;
            return false;
        }
    } else {
        cout << "Inverse kinematics: Target unreachable" << endl;
        return false;
    }

    return true;
}

void RobotAPI_SendCmd(const int sockfd_enet, const sockaddr_in remoteAddr_enet, const void *buffer, size_t buffer_size){
    Connection_SendUdp(sockfd_enet, remoteAddr_enet, buffer, sizeof(buffer_size));
}

static bool withinRobotsKinematics(Cartesian_Pos_t *targetWorldFrame){
    //todo: integrate check of inverse kinematics
}

static bool withinRobotsWorkspace(Cartesian_Pos_t *targetWorldFrame){
    //todo: integrate check of the workspace violations
}
