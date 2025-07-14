#include <netinet/in.h>
#include "robot_api.h"
#include "connection_handler.h"
#include "Matlab_ik.h"
#include "Matlab_ik_types.h"
#include "ik_wrapper.h"
#include <sstream>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

using std::cout;
using std::cerr;
using std::endl;

#define LOCAL_LOG_PREFIX "[RobotAPI]: "
#define COMM_TIMEOUT_MS 250

#define LOCAL_LOG_INFO(msg) std::cout << LOCAL_LOG_PREFIX << msg << std::endl
#define LOCAL_LOG_ERR(msg) std::cerr << LOCAL_LOG_PREFIX << msg << std::endl

class OneShotTimer {
public:
    OneShotTimer() : cancelled(false) {}

    void start(std::chrono::milliseconds duration, const std::function<void()>& callback) {
        cancelled = false;
        worker = std::thread([this, duration, callback]() {
            std::unique_lock<std::mutex> lock(mtx);
            if (cv.wait_for(lock, duration, [this]() { return cancelled.load(); })) {
                return;
            }
            callback();
        });
    }

    void cancel() {
        {
            std::lock_guard<std::mutex> lock(mtx);
            cancelled = true;
        }
        cv.notify_all();
        if (worker.joinable())
            worker.join();
    }

    ~OneShotTimer() {
        cancel();
    }

private:
    std::thread worker;
    std::condition_variable cv;
    std::mutex mtx;
    std::atomic<bool> cancelled;
};

static int sockfd_enet1;
static sockaddr_in sockaddr_enet1;

static int lastSentCmd = 0;
static std::string lastSentCmdString;
static OneShotTimer commTimer;

static double currentRobotConfig[6];
static std::vector<std::array<double, 6>> robotPathCartesian;

static Robot_Sequence_t currentSequenceType = Robot_Sequence_t::IDLE;
static Robot_Sequence_State_t currentSequenceState = Robot_Sequence_State_t::INIT;
static Robot_Sequence_Result_t currentSequenceResult = Robot_Sequence_Result_t::INIT;
static Robot_Comm_State_t currentCommState = Robot_Comm_State_t::INIT;

static void setSequenceState(Robot_Sequence_t newType, Robot_Sequence_State_t newState, Robot_Sequence_Result_t result);
static void sendRobotCommand(int command, const std::string& action_name);
static void setRobotConfig(double a1, double a2, double a3, double a4, double a5, double a6);

static void setCommState(Robot_Comm_State_t newState);
static Robot_Comm_State_t getCommState();
static void setCommTimer(int millis);
static void disableCommTimer();
static void commTimerCb();

static void setSequenceState(Robot_Sequence_t newType, Robot_Sequence_State_t newState, Robot_Sequence_Result_t result) {
    currentSequenceType = newType;
    currentSequenceState = newState;
    currentSequenceResult = result;
}

static void setCommState(Robot_Comm_State_t newState) {
    currentCommState = newState;
}

static Robot_Comm_State_t getCommState(){
    return currentCommState;
}

static void setCommTimer(int millis){
    commTimer.start(std::chrono::milliseconds(millis), commTimerCb);
}

static void disableCommTimer(){
    commTimer.cancel();
}

static void commTimerCb(){
    if(getCommState() == Robot_Comm_State_t::WAITING_FOR_RESPONSE){
        setCommState(Robot_Comm_State_t::TIMED_OUT);
    } else {
        LOCAL_LOG_ERR("Comm timer fired in wrong state");
    }
}

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

    lastSentCmd = command;
    lastSentCmdString = action_name;

    char buf_cmd[2]{};
    snprintf(buf_cmd, sizeof(buf_cmd), "%d", command);

    if(Connection_SendUdp(sockfd_enet1, sockaddr_enet1, buf_cmd, sizeof(buf_cmd)) != -1){
        setCommTimer(COMM_TIMEOUT_MS);
        setCommState(Robot_Comm_State_t::WAITING_FOR_RESPONSE);
    } else {
        LOCAL_LOG_ERR("Command transmission did not occur");
    }

}

bool RobotAPI_StartApproachSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE){
        LOCAL_LOG_INFO("Robot API is busy, cant start approach sequence");
        return false;
    }

    // coordinates are handled in handleOnltrackPlayCmd()
    setSequenceState(Robot_Sequence_t::APPROACH, Robot_Sequence_State_t::REQUESTED, Robot_Sequence_Result_t::SUCCESS);

    return true;

}

bool RobotAPI_StartFinalApproachSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE) {
        cout << "Robot API is busy, cant start approach sequence" << endl; //todo: enhance
        return false;

    }

    // coordinates are handled in handleOnltrackPlayCmd()
    setSequenceState(Robot_Sequence_t::FINAL_APPROACH, Robot_Sequence_State_t::REQUESTED, Robot_Sequence_Result_t::SUCCESS);
    return true;
}

bool RobotAPI_StartCutSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE) {
        cout << "Robot API is busy, cant start cut sequence" << endl; //todo: enhance

        return false;
    }

    sendRobotCommand(ENET_CUT, "Cut");
    setSequenceState(Robot_Sequence_t::CUT, Robot_Sequence_State_t::REQUESTED, Robot_Sequence_Result_t::SUCCESS);

    return true;

}

bool RobotAPI_StartStoreSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE){
        cout << "Robot API is busy, cant start store sequence" << endl; //todo: enhance
        return false;
    }

    sendRobotCommand(ENET_STORE, "Store");
    setSequenceState(Robot_Sequence_t::STORE, Robot_Sequence_State_t::REQUESTED, Robot_Sequence_Result_t::SUCCESS);

    return true;
}

bool RobotAPI_StartReturnToBaseSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE){
        cout << "Robot API is busy, cant start return to base sequence" << endl; //todo: enhance

        return false;
    }

    sendRobotCommand(ENET_RETURN_TO_BASE, "Return To Base");
    setSequenceState(Robot_Sequence_t::RETURN_TO_BASE, Robot_Sequence_State_t::REQUESTED, Robot_Sequence_Result_t::SUCCESS);

    return true;
}

bool RobotAPI_StartSwitchBaseSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE){
        cout << "Robot API is busy, cant start switch base sequence" << endl; //todo: enhance

        return false;
    }

    sendRobotCommand(ENET_SWITCH_BASE_NEXT, "Switch Base");
    setSequenceState(Robot_Sequence_t::SWITCH_BASE, Robot_Sequence_State_t::REQUESTED, Robot_Sequence_Result_t::SUCCESS);

    return true;
}

bool RobotAPI_StartGoHomeSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE){
        cout << "Robot API is busy, cant start go home sequence" << endl; //todo: enhance

        return false;
    }

    sendRobotCommand(ENET_SWITCH_BASE_HOME, "Go Home");
    setSequenceState(Robot_Sequence_t::GO_HOME, Robot_Sequence_State_t::REQUESTED, Robot_Sequence_Result_t::SUCCESS);

    return true;
}

bool RobotAPI_StartSafePositionSequence(){
    if(currentSequenceType != Robot_Sequence_t::IDLE){
        cout << "Robot API is busy, cant start safe position sequence" << endl; //todo: enhance

        return false;
    }

    sendRobotCommand(ENET_SAFE_POSITION, "Safe Position");
    setSequenceState(Robot_Sequence_t::SAFE_POSITION, Robot_Sequence_State_t::REQUESTED, Robot_Sequence_Result_t::SUCCESS);

    return true;
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
            cerr << "ENET_UNDEFINED: can't recognise received ENET1 cmd: " << std::string(buffer, buf_len) << endl;
            // ToDo: handle when cmd in not recognised?
            break;
        }
        case ENET_INIT: {
            cout << "[RobotAPI]: ENET1 received <init> from Hyundai at start" << endl;
            //program receives init from hyundai at the start to detect pc address automatically
            setCommState(Robot_Comm_State_t::IDLE);
            break;
        }
        case ENET_RETURN_TO_BASE_COMPLETE: {
            cout << "ENET1 received <return_to_base_complete>" << endl;
            if(currentSequenceType == Robot_Sequence_t::RETURN_TO_BASE){
                setSequenceState(Robot_Sequence_t::RETURN_TO_BASE, Robot_Sequence_State_t::COMPLETE, Robot_Sequence_Result_t::SUCCESS);
            } else {
                LOCAL_LOG_ERR("Sequence type does not match");
            }

            break;
        }

        case ENET_CUT_COMPLETE:
            cout << "ENET1 received <cut_complete>" << endl;

            if(currentSequenceType == Robot_Sequence_t::CUT){
                setSequenceState(Robot_Sequence_t::CUT, Robot_Sequence_State_t::COMPLETE, Robot_Sequence_Result_t::SUCCESS);
            } else {
                LOCAL_LOG_ERR("Sequence type does not match");
            }

            break;

        case ENET_STORE_COMPLETE:
            cout << "ENET1 received <store_complete>" << endl;

            if(currentSequenceType == Robot_Sequence_t::STORE){
                setSequenceState(Robot_Sequence_t::STORE, Robot_Sequence_State_t::COMPLETE, Robot_Sequence_Result_t::SUCCESS);
            } else {
                LOCAL_LOG_ERR("Sequence type does not match");
            }
            break;

        case ENET_SWITCH_BASE_NO_BASE_LEFT:
            cout << "ENET1 received <ENET_SWITCH_BASE_NO_BASE_LEFT>" << endl;

            if(currentSequenceType == Robot_Sequence_t::SWITCH_BASE){
                setSequenceState(Robot_Sequence_t::SWITCH_BASE, Robot_Sequence_State_t::COMPLETE, Robot_Sequence_Result_t::BASE_END);
            } else {
                LOCAL_LOG_ERR("Sequence type does not match");
            }
            break;

        case ENET_SWITCH_BASE_NEXT_COMPLETE:
            cout << "ENET1 received <switch_base_next_success>" << endl;

            if(currentSequenceType == Robot_Sequence_t::SWITCH_BASE){
                setSequenceState(Robot_Sequence_t::SWITCH_BASE, Robot_Sequence_State_t::COMPLETE, Robot_Sequence_Result_t::SUCCESS);
            } else {
                LOCAL_LOG_ERR("Sequence type does not match");
            }

            break;

        case ENET_SWITCH_BASE_HOME_COMPLETE:
            cout << "ENET1 received <switch_base_home_success>" << endl;
            setSequenceState(Robot_Sequence_t::GO_HOME, Robot_Sequence_State_t::COMPLETE, Robot_Sequence_Result_t::SUCCESS);
            break;

        case ENET_SAFE_POSITION_COMPLETE:
            cout << "ENET1 received <safe_position_complete>" << endl;
            setSequenceState(Robot_Sequence_t::SAFE_POSITION, Robot_Sequence_State_t::COMPLETE, Robot_Sequence_Result_t::SUCCESS);
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

        case ENET_STARTED_EXECUTION:

            if(getCommState() == Robot_Comm_State_t::WAITING_FOR_RESPONSE){
                LOCAL_LOG_INFO("Received STARTED_EXECUTION cmd, switching state machine");
                setCommState(Robot_Comm_State_t::RESPONSE_RECEIVED);
            } else {
                LOCAL_LOG_ERR("Received STARTED_EXECUTION cmd outside of the case");
            }
            break;

            default:
            cerr << "enet1Cmd: unexpected enet1Cmd value" << endl;
            break;
    }

}

bool RobotAPI_IsApproachSequenceActive(){
    return (currentSequenceType == Robot_Sequence_t::APPROACH
    && currentSequenceState == Robot_Sequence_State_t::REQUESTED);
}
bool RobotAPI_IsFinalApproachSequenceActive(){
    return (currentSequenceType == Robot_Sequence_t::FINAL_APPROACH
           && currentSequenceState == Robot_Sequence_State_t::REQUESTED);
}

void RobotAPI_EndSequence(Robot_Sequence_Result_t reason){
    setSequenceState(currentSequenceType, Robot_Sequence_State_t::COMPLETE, reason);
}

std::vector<std::array<double, 6>> RobotAPI_GetPathCopy() {
    return robotPathCartesian; // pathCopy elision or move constructor used — efficient
}

void RobotAPI_SetPath(const std::vector<std::array<double, 6>> &pathCartesian) {
    robotPathCartesian = pathCartesian; // vector copy assignment
}

Robot_Sequence_t RobotAPI_GetSequence(){
    return currentSequenceType;
}

Robot_Sequence_State_t RobotAPI_GetSequenceState(){
    return currentSequenceState;
}

Robot_Sequence_Result_t RobotAPI_GetSequenceResult(){
    return currentSequenceResult;
}

void RobotAPI_ResetSequenceData(){
    currentSequenceType = Robot_Sequence_t::IDLE;
    currentSequenceState = Robot_Sequence_State_t::INIT;
    currentSequenceResult = Robot_Sequence_Result_t::INIT;

    LOCAL_LOG_INFO("Reset sequence parameters to idle state");
}

static void commStateMachine(){
    static uint8_t commRetries;
    Robot_Comm_State_t commSt = getCommState();

    switch(commSt){
        case Robot_Comm_State_t::INIT:
        case Robot_Comm_State_t::IDLE:
        case Robot_Comm_State_t::WAITING_FOR_RESPONSE:
            break;

        case Robot_Comm_State_t::RESPONSE_RECEIVED:
            LOCAL_LOG_INFO("Response received");

            disableCommTimer();
            setCommState(Robot_Comm_State_t::SUCCESS);
            break;

        case Robot_Comm_State_t::TIMED_OUT:
            if(commRetries < 3){
                LOCAL_LOG_ERR("Command transmission timed out, retrying...");
                commRetries++;

                sendRobotCommand(lastSentCmd, lastSentCmdString);
            } else {
                LOCAL_LOG_ERR("Command transmission timed out, will not retry");
                setCommState(Robot_Comm_State_t::FAIL);
            }
            break;

        case Robot_Comm_State_t::FAIL:
            LOCAL_LOG_ERR("Command transmission failed");
            setCommState(Robot_Comm_State_t::END);
            break;

        case Robot_Comm_State_t::SUCCESS:
            LOCAL_LOG_INFO("Command transmitted successfully");
            setCommState(Robot_Comm_State_t::END);
            break;

        case Robot_Comm_State_t::END:
            commRetries = 0;
            lastSentCmd = 0;
            lastSentCmdString.clear();
            setCommState(Robot_Comm_State_t::IDLE);
            break;
    }
}

void RobotAPI_ProcessAction(){
    commStateMachine();
}


void RobotAPI_ClearPath(){
    robotPathCartesian.clear();
}


