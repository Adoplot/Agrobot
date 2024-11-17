#include "robot_api.h"

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

void RobotAPI_SendCmd(){

}

static bool withinRobotsKinematics(Cartesian_Pos_t *targetWorldFrame){
    //todo: integrate check of inverse kinematics
}

static bool withinRobotsWorkspace(Cartesian_Pos_t *targetWorldFrame){
    //todo: integrate check of the workspace violations
}
