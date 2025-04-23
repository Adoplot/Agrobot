#include <netinet/in.h>
#include "robot_api.h"
#include "connection_handler.h"
#include "Matlab_ik.h"
#include "Matlab_ik_types.h"
#include "ik_wrapper.h"
#include <sstream>

using std::cout;
using std::cerr;
using std::endl;

#define LOCAL_LOG_PREFIX "[RobotAPI]: "

#define LOCAL_LOG_INFO(msg) std::cout << LOCAL_LOG_PREFIX << msg << std::endl
#define LOCAL_LOG_ERR(msg) std::cerr << LOCAL_LOG_PREFIX << msg << std::endl

static int sockfd_enet1;
static sockaddr_in sockaddr_enet1;

static double currentRobotConfig[6];
static double robotPathCartesian[PATH_STEP_NUM][6] {0};

static RobotSequenceCallback stateCallback = nullptr;

static Robot_Sequence_t currentSequenceType = Robot_Sequence_t::IDLE;
static Robot_Sequence_State_t currentSequenceState = Robot_Sequence_State_t::INIT;

static void setSequenceState(Robot_Sequence_t newType, Robot_Sequence_State_t newState);
static void sendRobotCommand(int command, const std::string& action_name);
static void resetSequenceState();
static void setRobotConfig(double a1, double a2, double a3, double a4, double a5, double a6);

static void setSequenceState(Robot_Sequence_t newType, Robot_Sequence_State_t newState) {
    currentSequenceType = newType;
    currentSequenceState = newState;

    if (stateCallback == nullptr) {
        LOCAL_LOG_ERR("State callback is not initialized\n\tWill not send state change notifications");
    } else {
        stateCallback(newType, newState);
    }
}

static void resetSequenceState(){
    currentSequenceType = Robot_Sequence_t::IDLE;
    currentSequenceState = Robot_Sequence_State_t::INIT;

    LOCAL_LOG_INFO("Reset sequence to idle state");
}

// Todo: PASHA - targetParameters_worldFrame has to include cutplace [x1 y1 z1] and branch direction [x2 y2 z2]
//             - create pseudo function for calculating targetParameters_worldFrame from targetParameters_camFrame.
//               Input - *eePos_worldFrame
// Todo: PASHA - get currentConfig from Hyundai
// Todo: ADOPLOT get branchStart and branchDir from targetParameters_worldFrame
bool RobotAPI_IsTargetReachable(Target_Parameters_t *targetParameters_worldFrame, Hyundai_Data_t *eePos_worldFrame){
    double branchStart[3]       {targetParameters_worldFrame->x1,targetParameters_worldFrame->y1,targetParameters_worldFrame->z1};
    double branchDir[3]         {targetParameters_worldFrame->x2,targetParameters_worldFrame->y2,targetParameters_worldFrame->z2};

    // Declare outputs
    int code;
    double qWaypoints[18];

    // Get waypoints and success/fail code
    IK_getWaypointsForApproach(branchStart, branchDir, eePos_worldFrame, RobotAPI_GetCurrentConfig(), &code, qWaypoints);

    if (code == 1){
        return true;
    }
    else{
        return false;
    }

}

static void setRobotConfig(double a1, double a2, double a3, double a4, double a5, double a6){
    currentRobotConfig[0] = a1;
    currentRobotConfig[1] = a2;
    currentRobotConfig[2] = a3;
    currentRobotConfig[3] = a4;
    currentRobotConfig[4] = a5;
    currentRobotConfig[5] = a6;
/*
    std::stringstream ss;
    ss << "Set Robot config: ";
    for (int i = 0; i < 6; ++i) {
        ss << currentRobotConfig[i];
        if (i < 5) ss << ", ";
    }
    LOCAL_LOG_INFO(ss.str());
    */
}

double* RobotAPI_GetCurrentConfig(){
    return currentRobotConfig;
}

static void sendRobotCommand(int command, const std::string& action_name) {
    sockfd_enet1 = Connection_GetSockfd(SOCKTYPE_ENET1);
    sockaddr_enet1 = Connection_GetSockAddr(SOCKTYPE_ENET1);

    LOCAL_LOG_INFO("Initiating <" << action_name << "> sequence");

    char buf_cmd[2]{};
    snprintf(buf_cmd, sizeof(buf_cmd), "%d", command);
    Connection_SendUdp(sockfd_enet1, sockaddr_enet1, buf_cmd, sizeof(buf_cmd));
}

void RobotAPI_StartApproachSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE){
        LOCAL_LOG_INFO("Robot API is busy, cant start approach sequence");
        setSequenceState(Robot_Sequence_t::APPROACH, Robot_Sequence_State_t::FAIL);

        return;
    }

    // coordinates are handled in handleOnltrackPlayCmd()
    setSequenceState(Robot_Sequence_t::APPROACH, Robot_Sequence_State_t::REQUESTED);
}

void RobotAPI_StartFinalApproachSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE){
        cout << "Robot API is busy, cant start approach sequence" << endl; //todo: enhance

        setSequenceState(Robot_Sequence_t::FINAL_APPROACH, Robot_Sequence_State_t::FAIL);

        return;
    }

    // coordinates are handled in handleOnltrackPlayCmd()
    setSequenceState(Robot_Sequence_t::FINAL_APPROACH, Robot_Sequence_State_t::REQUESTED);
}

void RobotAPI_StartCutSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE) {
        cout << "Robot API is busy, cant start cut sequence" << endl; //todo: enhance
        setSequenceState(Robot_Sequence_t::CUT, Robot_Sequence_State_t::FAIL);

        return;
    }

    sendRobotCommand(ENET_CUT, "Cut");
    setSequenceState(Robot_Sequence_t::CUT, Robot_Sequence_State_t::REQUESTED);

}

void RobotAPI_StartStoreSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE){
        cout << "Robot API is busy, cant start store sequence" << endl; //todo: enhance
        setSequenceState(Robot_Sequence_t::STORE, Robot_Sequence_State_t::FAIL);

        return;
    }

    sendRobotCommand(ENET_STORE, "Store");
    setSequenceState(Robot_Sequence_t::STORE, Robot_Sequence_State_t::REQUESTED);
}

void RobotAPI_StartReturnToBaseSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE){
        cout << "Robot API is busy, cant start return to base sequence" << endl; //todo: enhance
        setSequenceState(Robot_Sequence_t::RETURN_TO_BASE, Robot_Sequence_State_t::FAIL);

        return;
    }

    sendRobotCommand(ENET_RETURN_TO_BASE, "Return To Base");
    setSequenceState(Robot_Sequence_t::RETURN_TO_BASE, Robot_Sequence_State_t::REQUESTED);
}

void RobotAPI_StartSwitchBaseSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE){
        cout << "Robot API is busy, cant start switch base sequence" << endl; //todo: enhance
        setSequenceState(Robot_Sequence_t::SWITCH_BASE, Robot_Sequence_State_t::FAIL);

        return;
    }

    sendRobotCommand(ENET_SWITCH_BASE, "Switch Base");
    setSequenceState(Robot_Sequence_t::SWITCH_BASE, Robot_Sequence_State_t::REQUESTED);
}

void RobotAPI_HandleEnetResponse(Enet_Cmd_t cmd, char* buffer, long buf_len){
    Enet_RecvStr_t enet_str;
    size_t prefix_len;
    const char* data_start;
    double values[6];
    int i = 0;
    char temp[128];
    char* token;

    switch(cmd){
        case ENET_UNDEFINED: {
            cerr << "ENET_UNDEFINED: can't recognise received ENET1 cmd" << endl;
            // ToDo: handle when cmd in not recognised?
            break;
        }
        case ENET_INIT: {
            cout << "ENET1 received <init>" << endl;
            //program receives init from hyundai at the start to detect pc address automatically
            break;
        }
        case ENET_RETURN_TO_BASE_COMPLETE: {
            cout << "ENET1 received <return_to_base_complete>" << endl;
            if(currentSequenceType == Robot_Sequence_t::RETURN_TO_BASE){
                setSequenceState(Robot_Sequence_t::RETURN_TO_BASE, Robot_Sequence_State_t::COMPLETE);
            } else {
                LOCAL_LOG_ERR("Sequence type does not match");
            }

            break;
        }

        case ENET_CUT_COMPLETE:
            cout << "ENET1 received <cut_complete>" << endl;

            if(currentSequenceType == Robot_Sequence_t::CUT){
                setSequenceState(Robot_Sequence_t::CUT, Robot_Sequence_State_t::COMPLETE);
            } else {
                LOCAL_LOG_ERR("Sequence type does not match");
            }

            break;

        case ENET_STORE_COMPLETE:
            cout << "ENET1 received <store_complete>" << endl;

            if(currentSequenceType == Robot_Sequence_t::STORE){
                setSequenceState(Robot_Sequence_t::STORE, Robot_Sequence_State_t::COMPLETE);
            } else {
                LOCAL_LOG_ERR("Sequence type does not match");
            }
            break;

        case ENET_SWITCH_BASE_COMPLETE:
            cout << "ENET1 received <switch_base_complete>" << endl;

            if(currentSequenceType == Robot_Sequence_t::SWITCH_BASE){
                setSequenceState(Robot_Sequence_t::SWITCH_BASE, Robot_Sequence_State_t::COMPLETE);
            } else {
                LOCAL_LOG_ERR("Sequence type does not match");
            }

            break;

        case ENET_ROBOT_CONFIGURATION:
            //cout << "ENET1 received <robot_configuration>" << endl;

            if (buf_len == 0 || buffer[buf_len - 2] != '\n') {
                LOCAL_LOG_ERR("Message incomplete (missing \\012)");
                return;
            }

            // Strip newline for clean processing
            buffer[buf_len - 1] = '\0';

            // Step 2: Check prefix
            prefix_len = strlen(enet_str.robot_configuration);

            if (strncmp(buffer, enet_str.robot_configuration, prefix_len) != 0) {
                LOCAL_LOG_ERR("Message does not start with 'robot_configuration'");
                return;
            }

            // Step 3: Move past the prefix and whitespace
            data_start = buffer + prefix_len;
            while (*data_start == ' ') data_start++;

            // Make a copy since strtok modifies the string
            strncpy(temp, data_start, sizeof(temp));
            temp[sizeof(temp) - 1] = '\0';

            token = strtok(temp, " ");
            while (token && i < 6) {
                values[i++] = Transform_Deg2Rad(strtod(token, NULL));
                token = strtok(NULL, " ");
            }

            if (i != 6) {
                LOCAL_LOG_ERR("Invalid number of values (expected 6)");
                return ;
            }

            setRobotConfig(values[0], values[1], values[2], values[3], values[4], values[5]);
            break;

            default:
            cerr << "enet1Cmd: unexpected enet1Cmd value" << endl;
            break;
    }

}

bool RobotAPI_IsApproachSequenceActive(){
    return currentSequenceType == Robot_Sequence_t::APPROACH;
}
bool RobotAPI_IsFinalApproachSequenceActive(){
    return currentSequenceType == Robot_Sequence_t::FINAL_APPROACH;
}

void RobotAPI_SetSequenceCallback(RobotSequenceCallback callback){
    stateCallback = callback;
}

void RobotAPI_EndSequence(Robot_Sequence_Result_t reason){
    if(reason != Robot_Sequence_Result_t::SUCCESS){
        setSequenceState(currentSequenceType, Robot_Sequence_State_t::FAIL);
    } else {
        setSequenceState(currentSequenceType, Robot_Sequence_State_t::COMPLETE);
    }
}

void RobotAPI_ProcessAction(){
    switch(currentSequenceState){
        case Robot_Sequence_State_t::INIT:
        case Robot_Sequence_State_t::REQUESTED:

            break;

        case Robot_Sequence_State_t::COMPLETE:
        case Robot_Sequence_State_t::FAIL:
            resetSequenceState();
            break;
    }
}

std::vector<std::array<double, 6>> RobotAPI_GetPathCopy() {
    std::vector<std::array<double, 6>> copy(PATH_STEP_NUM);
    for (int i = 0; i < PATH_STEP_NUM; ++i) {
        for (int j = 0; j < 6; ++j) {
            copy[i][j] = robotPathCartesian[i][j];
        }
    }
    return copy; // copy elision or move constructor used — efficient
}

void RobotAPI_SetPath(const double pathCartesian[PATH_STEP_NUM][6]) {

    for (int i = 0; i < PATH_STEP_NUM; ++i) {
        for (int j = 0; j < 6; ++j) {
            robotPathCartesian[i][j] = pathCartesian[i][j];
        }
    }
}

void RobotAPI_ClearPath(){
    memset(robotPathCartesian, 0, sizeof(robotPathCartesian));
}


