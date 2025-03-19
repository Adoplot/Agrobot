#include <netinet/in.h>
#include "robot_api.h"
#include "connection_handler.h"

using std::cout;
using std::cerr;
using std::endl;

#define ROBOTAPI_LOG(msg) std::cout << "[RobotAPI]: " << msg << std::endl

static int sockfd_enet1;
static sockaddr_in sockaddr_enet1;

static RobotSequenceCallback stateCallback = nullptr;

static RobotSequenceCallback robotSequenceCallback = nullptr;
static Robot_Sequence_t currentSequenceType = Robot_Sequence_t::IDLE;
static Robot_Sequence_State_t currentSequenceState = Robot_Sequence_State_t::INIT;

static bool withinRobotsKinematics(Cartesian_Pos_t *targetWorldFrame);
static bool withinRobotsWorkspace(Cartesian_Pos_t *targetWorldFrame);
static void setSequenceState(Robot_Sequence_t newType, Robot_Sequence_State_t newState);
static void sendRobotCommand(int command, const std::string& action_name);
static void resetSequenceState();

static void setSequenceState(Robot_Sequence_t newType, Robot_Sequence_State_t newState) {
    currentSequenceType = newType;
    currentSequenceState = newState;
    stateCallback(newType, newState);
}

static void resetSequenceState(){
    currentSequenceType = Robot_Sequence_t::IDLE;
    currentSequenceState = Robot_Sequence_State_t::INIT;

    ROBOTAPI_LOG("Reset sequence to idle state");
}

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

static bool withinRobotsKinematics(Cartesian_Pos_t *targetWorldFrame){
    //todo: integrate check of inverse kinematics

    return true;
}

static bool withinRobotsWorkspace(Cartesian_Pos_t *targetWorldFrame){
    //todo: integrate check of the workspace violations
    return true;
}

static void sendRobotCommand(int command, const std::string& action_name) {
    sockfd_enet1 = Connection_GetSockfd(SOCKTYPE_ENET1);
    sockaddr_enet1 = Connection_GetSockAddr(SOCKTYPE_ENET1);

    ROBOTAPI_LOG("Initiating <" << action_name << "> sequence");

    char buf_cmd[2]{};
    snprintf(buf_cmd, sizeof(buf_cmd), "%d", command);
    Connection_SendUdp(sockfd_enet1, sockaddr_enet1, buf_cmd, sizeof(buf_cmd));
}

void RobotAPI_StartApproachSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE){
        ROBOTAPI_LOG("Robot API is busy, cant start approach sequence");
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

void RobotAPI_HandleEnetResponse(Enet_Cmd_t cmd){
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
            setSequenceState(Robot_Sequence_t::RETURN_TO_BASE, Robot_Sequence_State_t::COMPLETE);
            break;
        }

        case ENET_CUT_COMPLETE:
            cout << "ENET1 received <cut_complete>" << endl;
            setSequenceState(Robot_Sequence_t::CUT, Robot_Sequence_State_t::COMPLETE);
            break;

        case ENET_STORE_COMPLETE:
            cout << "ENET1 received <store_complete>" << endl;
            setSequenceState(Robot_Sequence_t::STORE, Robot_Sequence_State_t::COMPLETE);
            break;

        case ENET_SWITCH_BASE_COMPLETE:
            cout << "ENET1 received <switch_base_complete>" << endl;
            setSequenceState(Robot_Sequence_t::SWITCH_BASE, Robot_Sequence_State_t::COMPLETE);
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