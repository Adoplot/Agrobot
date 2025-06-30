#include "compv_handler.h"

#include "connection_handler.h"
#include "onltrack_handler.h"
#include "transform_calc.h"
#include "nlohmann/json.hpp"
#include "robot_api.h"
#include "Matlab_ik.h"
#include "Matlab_ik_types.h"
#include "ik_wrapper.h"


using std::cout;
using std::cerr;
using std::endl;
using nlohmann::json;

constexpr const char* NONE_STR = "IDLE";
constexpr const char* SYNC_TARGETS_STR = "SYNC_TARGETS";
constexpr const char* APPROACH_STR = "APPROACH";
constexpr const char* FINAL_APPROACH_STR = "FINAL_APPROACH";
constexpr const char* CUT_STR = "CUT";
constexpr const char* STORE_STR = "STORE";
constexpr const char* RETURN_TO_BASE_STR = "RETURN_TO_BASE";
constexpr const char* SWITCH_BASE_STR = "SWITCH_BASE_NEXT";
constexpr const char* GO_HOME_STR = "GO_HOME";
constexpr const char* SAFE_POSITION = "SAFE_POSITION";
constexpr const char* GET_ROBOT_STATE_STR = "GET_ROBOT_STATE";

constexpr const char* COMPV_RESULT_COMPLETE = "COMPLETE";
constexpr const char* COMPV_RESULT_IN_PROGRESS = "IN_PROGRESS";
constexpr const char* COMPV_RESULT_FAIL = "FAIL";
constexpr const char* COMPV_RESULT_REQUESTED = "REQUESTED";

constexpr const char* COMPV_REASON_UNREACHABLE = "UNREACHABLE";
constexpr const char* COMPV_REASON_BASE_END = "BASE_END";
constexpr const char* COMPV_REASON_BUSY = "BUSY";
constexpr const char* COMPV_REASON_SUCCESS = "SUCCESS";
constexpr const char* COMPV_REASON_JSON_ERR = "JSON_ERROR";
constexpr const char* COMPV_REASON_GENERIC_ERROR = "ERROR";
constexpr const char* COMPV_REASON_INIT = "NO_REASON";

static int sockfd_compv;

static Cartesian_Pos_t targetPos_camFrame{};
static Cartesian_Pos_t targetPos_worldFrame{};

static Tcp_Data_t tcp_data_state = TCP_DATA_NOT_AVAILABLE;  // tracks if new cmd is received from compv

static json jsonParse(const std::string* data);
static CompV_Request_t getJsonRequest(const json* json);
static Target_Parameters_t getTargetParametersFromJsonEntry(const json* json);
static std::vector<Target_Parameters_t> getTargetParametersFromJson(const nlohmann::json* json);

static void convertTargetParameterToJson(json& j, const Target_Parameters_t& pos);

static void sendSyncTargetsResponse(std::vector<Target_Parameters_t> positions);
static void sendApproachResponse(double x1, double y1, double z1, double x2, double y2, double z2);
static void sendFinalApproachResponse(double x1, double y1, double z1, double x2, double y2, double z2);
static void sendStatusResponse(const char* request, const char* result, const char* reason);

static void handleSyncTargetsRequest(const json& json);
static void handleApproachRequest(const json& json);
static void handleFinalApproachRequest(const json& json);
static void handleCutRequest();
static void handleStoreRequest();
static void handleReturnToBaseRequest();
static void handleSwitchBaseRequest();
static void handleGoHomeRequest();
static void handleSafePositionRequest();
static void handleGetRobotStateRequest();

static const char* sequenceToString(Robot_Sequence_t sequence);
static const char* sequenceResultToString(Robot_Sequence_State_t state);
static const char* sequenceReasonToString(Robot_Sequence_Result_t result);

static const char* sequenceToString(Robot_Sequence_t sequence) {
    switch (sequence) {
        case Robot_Sequence_t::IDLE: return NONE_STR;
        case Robot_Sequence_t::APPROACH: return APPROACH_STR;
        case Robot_Sequence_t::FINAL_APPROACH: return FINAL_APPROACH_STR;
        case Robot_Sequence_t::CUT: return CUT_STR;
        case Robot_Sequence_t::STORE: return STORE_STR;
        case Robot_Sequence_t::RETURN_TO_BASE: return RETURN_TO_BASE_STR;
        case Robot_Sequence_t::SWITCH_BASE: return SWITCH_BASE_STR;
        case Robot_Sequence_t::GO_HOME: return GO_HOME_STR;
        default: return "UNKNOWN";
    }
}

static const char* sequenceResultToString(Robot_Sequence_State_t state) {
    switch (state) {
        case Robot_Sequence_State_t::INIT: return COMPV_RESULT_FAIL;
        case Robot_Sequence_State_t::REQUESTED: return COMPV_RESULT_REQUESTED;
        case Robot_Sequence_State_t::COMPLETE: return COMPV_RESULT_COMPLETE;
        case Robot_Sequence_State_t::FAIL: return COMPV_RESULT_FAIL;
        default: return "UNKNOWN";
    }
}

static const char* sequenceReasonToString(Robot_Sequence_Result_t result){
    switch(result){
        case Robot_Sequence_Result_t::INIT: return COMPV_REASON_INIT;
        case Robot_Sequence_Result_t::BUSY: return COMPV_REASON_BUSY;
        case Robot_Sequence_Result_t::BASE_END: return COMPV_REASON_BASE_END;
        case Robot_Sequence_Result_t::SUCCESS: return COMPV_REASON_SUCCESS;
        case Robot_Sequence_Result_t::UNREACHABLE: return COMPV_REASON_UNREACHABLE;

        default: return "UNKNOWN";
    }
}
// Interface to set value of targetPos_worldFrame
void CompV_SetTargetPosWorldFrame(Cartesian_Pos_t new_value) {
    targetPos_worldFrame = new_value;
}


// Interface to set value of targetPos_camFrame
void CompV_SetTargetPosCamFrame(Cartesian_Pos_t new_value) {
    targetPos_camFrame = new_value;
}


// Interface to get pointer to targetPos_camFrame
// Returns: pointer to targetPos_camFrame
Cartesian_Pos_t* Compv_GetTargetPosCamFrame() {
    return &targetPos_camFrame;
}


// Interface to get pointer to targetPos_worldFrame
// Returns: pointer to targetPos_worldFrame
Cartesian_Pos_t* Compv_GetTargetPosWorldFrame() {
    return &targetPos_worldFrame;
}


// Reads received message, parses json to request, handles the request
void Compv_HandleCmd(const std::string* data) {
    printf("[CompV]: Reqeusted to handle cmd\n");
    // Get socket data
    sockfd_compv = Connection_GetSockfd(SOCKTYPE_COMPV);

    // Parse JSON and get request type
    json json = jsonParse(data);
    CompV_Request_t request = getJsonRequest(&json);

    switch (request) {
        case COMPV_REQ_INVALID:
            std::cerr << "[CompV]: Cmd is not valid" << endl;
            break;

        case COMPV_REQ_SYNC_TARGETS:
            handleSyncTargetsRequest(json);
            break;

        case COMPV_REQ_APPROACH:
            handleApproachRequest(json);
            break;

        case COMPV_REQ_FINAL_APPROACH:
            handleFinalApproachRequest(json);
            break;

        case COMPV_REQ_CUT:
            handleCutRequest();
            break;

        case COMPV_REQ_STORE:
            handleStoreRequest();
            break;

        case COMPV_REQ_RETURN_TO_BASE:
            handleReturnToBaseRequest();
            break;

        case COMPV_REQ_SWITCH_BASE:
            handleSwitchBaseRequest();
            break;

        case COMPV_REQ_SAFE_POSITION:
            handleSafePositionRequest();

        case COMPV_REQ_GO_HOME:
            handleGoHomeRequest();
            break;

        case COMPV_REQ_GET_ROBOT_STATE:
            handleGetRobotStateRequest();
            break;

        default:
            cerr << "CompV_HandleCmd(): Unexpected cmd" << endl;
            break;
    }
}

static void handleReturnToBaseRequest(){
    cout << "[CompV]: Received Return to base request" << endl;

    if(!RobotAPI_StartReturnToBaseSequence()){
        sendStatusResponse(RETURN_TO_BASE_STR, COMPV_RESULT_FAIL, COMPV_REASON_BUSY);
    } else {
        sendStatusResponse(RETURN_TO_BASE_STR, COMPV_RESULT_REQUESTED, COMPV_REASON_SUCCESS);
    }
}

static void handleSwitchBaseRequest(){
    cout << "[CompV]: Received Switch base request" << endl;

    if(!RobotAPI_StartSwitchBaseSequence()){
        sendStatusResponse(SWITCH_BASE_STR, COMPV_RESULT_FAIL, COMPV_REASON_BUSY);
    } else {
        sendStatusResponse(SWITCH_BASE_STR, COMPV_RESULT_REQUESTED, COMPV_REASON_SUCCESS);
    }
}

static void handleSafePositionRequest(){
    cout << "[CompV]: Received Safe position request" << endl;

    if(!RobotAPI_StartSafePositionSequence()){
        sendStatusResponse(SAFE_POSITION, COMPV_RESULT_FAIL, COMPV_REASON_BUSY);
    } else {
        sendStatusResponse(SAFE_POSITION, COMPV_RESULT_REQUESTED, COMPV_REASON_SUCCESS);
    }
}

static void handleGoHomeRequest(){
    cout << "[CompV]: Received Go Home request" << endl;

    if(!RobotAPI_StartGoHomeSequence()){
        sendStatusResponse(GO_HOME_STR, COMPV_RESULT_FAIL, COMPV_REASON_BUSY);
    } else {
        sendStatusResponse(GO_HOME_STR, COMPV_RESULT_REQUESTED, COMPV_REASON_SUCCESS);
    }
}

static void handleGetRobotStateRequest(){
    cout << "[CompV]: Sending Robot State Response" << endl;

    json json_send;

    Robot_Sequence_State_t sequenceState = RobotAPI_GetSequenceState();

    const char* sequenceName = sequenceToString(RobotAPI_GetSequence());
    const char* currentSequenceResult = sequenceResultToString(sequenceState);
    const char* currentSequenceReason = sequenceReasonToString(RobotAPI_GetSequenceResult());


    if( sequenceState == Robot_Sequence_State_t::COMPLETE ||
        sequenceState == Robot_Sequence_State_t::FAIL){

        RobotAPI_ResetSequenceData();
    }


    json_send["request"] = GET_ROBOT_STATE_STR;
    json_send["sequence"] = sequenceName;
    json_send["result"] = currentSequenceResult;
    json_send["reason"] = currentSequenceReason;

    std::string string_send = json_send.dump();
    Connection_SendTcp(sockfd_compv, &string_send);
}

static void handleStoreRequest(){
    cout << "[CompV]: Received Store request" << endl;

    if(!RobotAPI_StartStoreSequence()){
        sendStatusResponse(STORE_STR, COMPV_RESULT_FAIL, COMPV_REASON_BUSY);
    } else {
        sendStatusResponse(STORE_STR, COMPV_RESULT_REQUESTED, COMPV_REASON_SUCCESS);
    }
}

// Parses string to JSON, checks if JSON is valid
// Returns: parsed JSON
static json jsonParse(const std::string* data) {
    json received_json{};
    // Check if TCP input is a valid JSON
    if (!json::accept(*data)) {     //ToDo: <lib bug> accept() returns true if std::string is numerical "1", "1234" etc.
        cerr << "Json_Parse(): Invalid JSON received - " << *data;
    }
    else {
        received_json = json::parse(*data);
    }
    return received_json;
}

static void handleCutRequest(){
    if(!RobotAPI_StartCutSequence()){
        sendStatusResponse(CUT_STR, COMPV_RESULT_FAIL, COMPV_REASON_BUSY);
    } else {
        sendStatusResponse(CUT_STR, COMPV_RESULT_REQUESTED, COMPV_REASON_SUCCESS);
    }
}

static void sendStatusResponse(const char* request, const char* result, const char* reason){
    cout << "[CompV]: Sending reason response:\n\tRequest = " << request << "\n\tStatus = " << result << "\n\tResult = " << result << endl; //todo elaborate
    json json_send;
    json_send["request"] = request;
    json_send["result"] = result;
    json_send["reason"] = reason;
    std::string string_send = json_send.dump();
    Connection_SendTcp(sockfd_compv, &string_send);
}

static void handleSyncTargetsRequest(const json& json){
    cout    << "[CompV]: Received Sync Targets request\n\t"
            << "Checking if targets are reachable" << endl;

    std::vector<Target_Parameters_t> targetParameters = getTargetParametersFromJson(&json); //cam frame
    Hyundai_Data_t *eePos_worldFrame = Connection_GetEePosWorldFrame();

    for (auto& target : targetParameters) {
        if(RobotAPI_IsTargetReachable(&target, eePos_worldFrame)){
            cout << "Target ID: " << target.id << "is reachable\n"
                << "Coordinates:\n"
                << "x1: " << target.x1 << "\n"
                << "y1: " << target.y1 << "\n"
                << "z1: " << target.z1 << "\n" << endl;

            target.isReachable = true;
        } else {
            cout << "Target ID: " << target.id << "is NOT reachable\n"
                 << "Coordinates:\n"
                 << "x1: " << target.x1 << "\n"
                 << "y1: " << target.y1 << "\n"
                 << "z1: " << target.z1 << "\n" << endl;

            target.isReachable = false;
        }
    }

    sendSyncTargetsResponse(targetParameters);
}

static void convertTargetParameterToJson(json& j, const Target_Parameters_t& pos) {
    j = json{
            {"id", pos.id},
            {"isReachable", pos.isReachable},
            {"x1", pos.x1},
            {"y1", pos.y1},
            {"z1", pos.z1},
            {"x2", pos.x2},
            {"y2", pos.y2},
            {"z2", pos.z2}
    };
}

static void sendSyncTargetsResponse(std::vector<Target_Parameters_t> targets){
    cout << "[CompV]: Sending Reachable targets JSON" << endl;
    json json_send;

    json json_positions = json::array();
    for (const auto& target : targets) {
        json j;
        convertTargetParameterToJson(j, target);
        json_positions.push_back(j);
    }
    json_send["request"] = SYNC_TARGETS_STR;
    json_send["result"] = COMPV_RESULT_COMPLETE;
    json_send["reason"] = COMPV_REASON_SUCCESS; ///< TODO: Add error handling

    json_send["positions"] = json_positions;

    std::string string_send = json_send.dump();
    Connection_SendTcp(sockfd_compv, &string_send);
}

static void sendApproachResponse(double x1, double y1, double z1, double x2, double y2, double z2){
    cout << "[CompV]: Sending approach response" << endl;
    json json_send;
    json json_position = json{
            {"x1", x1},
            {"y1", y1},
            {"z1", z1},
            {"x2", x2},
            {"y2", y2},
            {"z2", z2}
    };

    json_send["request"] = APPROACH_STR;
    json_send["result"] = COMPV_RESULT_REQUESTED;
    json_send["reason"] = COMPV_REASON_SUCCESS; ///< TODO: Add error handling
    json_send["target_coordinates_wrld"] = json_position;

    std::string string_send = json_send.dump();
    Connection_SendTcp(sockfd_compv, &string_send);
}

static void sendFinalApproachResponse(double x1, double y1, double z1, double x2, double y2, double z2){
    cout << "[CompV]: Sending final approach response" << endl;
    json json_send;
    json json_position = json{
            {"x1", x1},
            {"y1", y1},
            {"z1", z1},
            {"x2", x2},
            {"y2", y2},
            {"z2", z2}
    };

    json_send["request"] = FINAL_APPROACH_STR;
    json_send["result"] = COMPV_RESULT_REQUESTED;
    json_send["reason"] = COMPV_REASON_SUCCESS; ///< TODO: Add error handling
    json_send["target_coordinates_wrld"] = json_position;

    std::string string_send = json_send.dump();
    Connection_SendTcp(sockfd_compv, &string_send);
}

static void handleApproachRequest(const json& json) {
    /*
    cout << "Initiating <Set Pos> sequence" << endl;

    Hyundai_Data_t *eePos_worldFrame = Connection_GetEePosWorldFrame();

    tcp_data_state = TCP_DATA_NEW_AVAILABLE;
    targetPos_camFrame = getJsonPos(&json);
    targetPos_worldFrame = Transform_ConvertFrameTarget2World(&targetPos_camFrame,
                                                              eePos_worldFrame,
                                                              SCISSORS_LENGTH);
*/

    // + get targetParameters and divide them into targetStart_camFrame and targetDir_camFrame
    // + convert them from camFrame to worldFrame
    // + get sortedList
    // + receive currentConfig from hyundai
    // + loop through sortedList with getGikFull and output qWaypoints or send failMessage
    // + get pathApr from IK_getTrajectory
    // o save pathApr for further onltrack increment calculations

    cout << "[CompV]: Received Approach request" << endl;
    //Get current robot EE coords
    Hyundai_Data_t *eeCoords_worldFrame = Connection_GetEePosWorldFrame();

    //In camera frame
    std::vector<Target_Parameters_t> targetParametersVector = getTargetParametersFromJson(&json);
    Target_Parameters_t target;

    if (!targetParametersVector.empty()) {
        target = targetParametersVector[0];

        cout << "\tTarget parameters:\n\tX1 = " << target.x1 << "; Y1 = " << target.y1 << "; Z1 = " << target.z1
        << "\n\tX2 = " << target.x2 << "; Y2 = " << target.y2 << "; Z2 = " << target.z2 << endl;
    } else {
        std::cerr << "\tNo targets received from JSON\n\tAborting sequence" << std::endl;

        sendStatusResponse(APPROACH_STR, COMPV_RESULT_FAIL, COMPV_REASON_JSON_ERR);
        return;
    }

    double *currentConfig = RobotAPI_GetCurrentConfig();

    Cartesian_Pos_t targetStart_camFrame{};
    Cartesian_Pos_t targetDir_camFrame{};
    Cartesian_Pos_t targetStart_worldFrame{};
    Cartesian_Pos_t targetDir_worldFrame{};

    // ToDo: ADOPLOT - refactor into the function (targetParameters_worldFrame to camFrame)
    //Divide targetParameters into targetStart_camFrame and targetDir_camFrame
    targetStart_camFrame.x = target.x1;
    targetStart_camFrame.y = target.y1;
    targetStart_camFrame.z = target.z1;

    targetDir_camFrame.x = target.x2;
    targetDir_camFrame.y = target.y2;
    targetDir_camFrame.z = target.z2;

    //Transform targetParameters to worldFrame
    targetStart_worldFrame = Transform_ConvertFrameTarget2World(&targetStart_camFrame,eeCoords_worldFrame);
    targetDir_worldFrame = Transform_ConvertFrameTarget2World(&targetDir_camFrame,eeCoords_worldFrame);

    //Show targetStart_worldFrame coords
    std::cout << std::fixed << std::showpoint;
    std::cout << std::setprecision(3);
    cout << "\n\tTarget Start:\t\t"
    << "X = " << targetStart_worldFrame.x
    << "; Y = " << targetStart_worldFrame.y
    << "; Z = " << targetStart_worldFrame.z

    << "\tRotX = " << targetStart_worldFrame.rotx
    << "; RotY = " << targetStart_worldFrame.roty
    << "; RotZ = " << targetStart_worldFrame.rotz
    << endl;

    cout << "\tTarget Direction:\t"
         << "X = " << targetDir_worldFrame.x
         << "; Y = " << targetDir_worldFrame.y
         << "; Z = " << targetDir_worldFrame.z

         << "\tRotX = " << targetDir_worldFrame.rotx
         << "; RotY = " << targetDir_worldFrame.roty
         << "; RotZ = " << targetDir_worldFrame.rotz
         << "\n" << endl;


    //------------------------------------------------------------
    //Get sorted list of points on a circle around cutplace
    //------------------------------------------------------------
    double branchStart[3] {0};
    double branchDir[3] {0};
    branchStart[0] = targetStart_worldFrame.x;
    branchStart[1] = targetStart_worldFrame.y;
    branchStart[2] = targetStart_worldFrame.z;
    branchDir[0] = targetDir_worldFrame.x;
    branchDir[1] = targetDir_worldFrame.y;
    branchDir[2] = targetDir_worldFrame.z;

    // Declare outputs
    int code;
    double qWaypoints[18];

    // Get waypoints and success/fail code
    IK_getWaypointsForApproach(branchStart, branchDir, eeCoords_worldFrame, currentConfig, &code, qWaypoints);

    if (code != 1){
        cout << "[CompV]: IK_getWaypointsForApproach: GIK failed, aborting Approach Sequence" << endl;
        sendStatusResponse(APPROACH_STR, COMPV_RESULT_FAIL, COMPV_REASON_UNREACHABLE);
    }
    else{
        //Print for debug
        IK_PrintWaypoints(qWaypoints);

        // Choose 2nd row of qWaypoints[18] for Approach sequence
        double waypointApr[6] {0};
        waypointApr[0] = qWaypoints[1];
        waypointApr[1] = qWaypoints[4];
        waypointApr[2] = qWaypoints[7];
        waypointApr[3] = qWaypoints[10];
        waypointApr[4] = qWaypoints[13];
        waypointApr[5] = qWaypoints[16];
        cout << "Choosing 2nd row of qWaypoints:" << endl;
        for(int i=0; i<6; i++){
            cout << waypointApr[i] << "  ";
        }
        cout << endl;

        //------------------------------------------------------------
        //Calculating trajectory for Approach sequence
        //------------------------------------------------------------
        bool pathAprIsValid {false};
        std::vector<std::array<double,6>> pathCartesian{};

        pathAprIsValid = IK_getTrajectory(currentConfig,waypointApr,PATH_VELOCITY, PATH_STEP_TIME, pathCartesian);

        if (pathAprIsValid){
            cout << "\tPath is valid - Calling Start Approach Sequence" << endl;

            if(!RobotAPI_StartApproachSequence()){
                sendStatusResponse(APPROACH_STR, COMPV_RESULT_FAIL, COMPV_REASON_BUSY);
            } else {
                RobotAPI_SetPath(pathCartesian);
                sendApproachResponse(targetStart_worldFrame.x, targetStart_worldFrame.y, targetStart_worldFrame.z,
                                     targetDir_worldFrame.x,targetDir_worldFrame.y, targetDir_worldFrame.z);
            }
        } else{
            cout << "\tPath is NOT valid" << endl;
            sendStatusResponse(APPROACH_STR, COMPV_RESULT_FAIL, COMPV_REASON_UNREACHABLE);
        }

        //Print for debug
        if (pathCartesian.size() >= 2) {
            cout << "pathCartesian:" << endl;
            std::cout << std::fixed;
            std::cout << std::setprecision(5);
            for (int i = 0; i < 6; i++) {
                cout << pathCartesian[0][i] << "  ";
            }
            cout << endl;
            for (int i = 0; i < 6; i++) {
                cout << pathCartesian[1][i] << "  ";
            }
            cout << endl;
            for (int i = 0; i < 6; i++) {
                cout << pathCartesian[2][i] << "  ";
            }
            cout << endl;
            for (int i = 0; i < 6; i++) {
                cout << pathCartesian[pathCartesian.size() - 2][i] << "  ";
            }
            cout << endl;
            for (int i = 0; i < 6; i++) {
                cout << pathCartesian[pathCartesian.size() - 1][i] << "  ";
            }
            cout << endl;
        }
    }
}

static void handleFinalApproachRequest(const json& json) {
    cout << "Initiating <Final Approach> sequence" << endl;
/*
 * OLD CODE
    Hyundai_Data_t *eePos_worldFrame = Connection_GetEePosWorldFrame();

    tcp_data_state = TCP_DATA_NEW_AVAILABLE;
    targetPos_camFrame = getJsonPos(&json);
    targetPos_worldFrame = Transform_ConvertFrameTarget2World(&targetPos_camFrame,
                                                              eePos_worldFrame,
                                                              SCISSORS_LENGTH);
 * OLD CODE END
*/

    //Get current robot EE coords
    Hyundai_Data_t *eeCoords_worldFrame = Connection_GetEePosWorldFrame();

    std::vector<Target_Parameters_t> targetParametersVector = getTargetParametersFromJson(&json);
    Target_Parameters_t target;

    if (!targetParametersVector.empty()) {
        target = targetParametersVector[0];
    } else {
        std::cerr << "No targets received from JSON\n\tAborting sequence" << std::endl;

        sendStatusResponse(FINAL_APPROACH_STR, COMPV_RESULT_FAIL, COMPV_REASON_JSON_ERR);
        return;
    }

    double *currentConfig = RobotAPI_GetCurrentConfig();

    Cartesian_Pos_t targetStart_camFrame{};
    Cartesian_Pos_t targetDir_camFrame{};
    Cartesian_Pos_t targetStart_worldFrame{};
    Cartesian_Pos_t targetDir_worldFrame{};

    // ToDo: ADOPLOT - refactor into the function (targetParameters_worldFrame to camFrame)
    //Divide targetParameters into targetStart_camFrame and targetDir_camFrame
    targetStart_camFrame.x = target.x1;
    targetStart_camFrame.y = target.y1;
    targetStart_camFrame.z = target.z1;

    //Transform targetParameters to worldFrame
    targetStart_worldFrame = Transform_ConvertFrameTarget2World(&targetStart_camFrame,eeCoords_worldFrame);
    targetDir_worldFrame = Transform_ConvertFrameTarget2World(&targetDir_camFrame,eeCoords_worldFrame);

    //Show targetStart_worldFrame coords
    std::cout << std::fixed << std::showpoint;
    std::cout << std::setprecision(3);
    std::cout << "targetStart \t";
    std::cout << "x=" << targetStart_worldFrame.x;
    std::cout << " y=" << targetStart_worldFrame.y;
    std::cout << " z=" << targetStart_worldFrame.z;
    std::cout << " rotX=" << targetStart_worldFrame.rotx;
    std::cout << " rotY=" << targetStart_worldFrame.roty;
    std::cout << " rotZ=" << targetStart_worldFrame.rotz;
    cout << endl;
    std::cout << std::fixed << std::showpoint;
    std::cout << std::setprecision(3);
    std::cout << "targetDir \t";
    std::cout << "x=" << targetDir_worldFrame.x;
    std::cout << " y=" << targetDir_worldFrame.y;
    std::cout << " z=" << targetDir_worldFrame.z;
    std::cout << " rotX=" << targetDir_worldFrame.rotx;
    std::cout << " rotY=" << targetDir_worldFrame.roty;
    std::cout << " rotZ=" << targetDir_worldFrame.rotz;
    cout << endl;

    double branchStart[3] {0};
    double branchDir[3] {0};
    branchStart[0] = targetStart_worldFrame.x;
    branchStart[1] = targetStart_worldFrame.y;
    branchStart[2] = targetStart_worldFrame.z;
    branchDir[0] = targetDir_worldFrame.x;
    branchDir[1] = targetDir_worldFrame.y;
    branchDir[2] = targetDir_worldFrame.z;

    // Declare outputs
    int code;
    double exitCode;
    double qWaypointCut[6];
    struct1_T solutionInfoApr {}; //only for debug
    //Initialize solver parameters
    struct0_T solverParameters {};
    IK_InitSolverParameters(&solverParameters);

    Matlab_getGikCut(currentConfig,branchStart,&solverParameters, CAM_ANGLE_OFFSET, &exitCode,&solutionInfoApr,qWaypointCut);
    code = static_cast<int>(exitCode+0.1);

    if (code != 1){
        cout << "Matlab_getGikCut: GIK failed, aborting FinalApproach Sequence" << endl;
        sendStatusResponse(FINAL_APPROACH_STR, COMPV_RESULT_FAIL, COMPV_REASON_UNREACHABLE);
    }
    else{
        //Print for debug
        cout << "qWaypointCut: ";
        cout << std::fixed;
        cout << std::setprecision(4);
        for (int i=0;i<6;i++){
            cout << qWaypointCut[i] << "  ";
        }
        cout << endl;


        //------------------------------------------------------------
        //Calculating trajectory for Approach sequence
        //------------------------------------------------------------
        bool pathFinal_IsValid {false};
        std::vector<std::array<double,6>> pathCartesian{};

        pathFinal_IsValid = IK_getTrajectory(currentConfig, qWaypointCut, PATH_VELOCITY, PATH_STEP_TIME, pathCartesian);

        if (pathFinal_IsValid){
            cout << "FinalApproach: Path is valid" << endl;

            if(!RobotAPI_StartFinalApproachSequence()){
                sendStatusResponse(FINAL_APPROACH_STR, COMPV_RESULT_FAIL, COMPV_REASON_BUSY);
            } else {
                RobotAPI_SetPath(pathCartesian);
                sendFinalApproachResponse(targetStart_worldFrame.x, targetStart_worldFrame.y, targetStart_worldFrame.z,
                                          targetDir_worldFrame.x,targetDir_worldFrame.y, targetDir_worldFrame.z);
            }
        } else {
            cout << "FinalApproach: Path is NOT valid" << endl;

            sendStatusResponse(FINAL_APPROACH_STR, COMPV_RESULT_FAIL, COMPV_REASON_UNREACHABLE);

        }

        //Print for debug
        if (pathCartesian.size() >= 2) {
            cout << "pathCartesian:" << endl;
            std::cout << std::fixed;
            std::cout << std::setprecision(5);
            for (int i = 0; i < 6; i++) {
                cout << pathCartesian[0][i] << "  ";
            }
            cout << endl;
            for (int i = 0; i < 6; i++) {
                cout << pathCartesian[1][i] << "  ";
            }
            cout << endl;
            for (int i = 0; i < 6; i++) {
                cout << pathCartesian[2][i] << "  ";
            }
            cout << endl;
            for (int i = 0; i < 6; i++) {
                cout << pathCartesian[pathCartesian.size() - 2][i] << "  ";
            }
            cout << endl;
            for (int i = 0; i < 6; i++) {
                cout << pathCartesian[pathCartesian.size() - 1][i] << "  ";
            }
            cout << endl;
        }

    }

}

// Gets request type from parsed JSON.
// Returns: request type
static CompV_Request_t getJsonRequest(const json* json){
    CompV_Request_t req = COMPV_REQ_INVALID;
    try {
        // recv xyz, immediately sends IN_PROGRESS if not at target pos,or COMPLETE if is at target pos.
        // Then convert frames, then calc incr, then rewrites global variable with incr to be sent to OnLTrack
        if (json->at("request") == APPROACH_STR)
            req = COMPV_REQ_APPROACH;

        // send ENET1 command to robot, robot turns off Onltrack, executes ptp motion to home pos
        // and sends complete via ENET1
        else if (json->at("request") == RETURN_TO_BASE_STR)
            req = COMPV_REQ_RETURN_TO_BASE;

        // same but closes gripper instead of ptp motion
        else if (json->at("request") == CUT_STR)
            req = COMPV_REQ_CUT;

        // same but uses ptp motion position to the storing position
        else if (json->at("request") == STORE_STR)
            req = COMPV_REQ_STORE;

        else if(json->at("request") == SYNC_TARGETS_STR){
            req = COMPV_REQ_SYNC_TARGETS;

        } else if(json->at("request") == SWITCH_BASE_STR){
            req = COMPV_REQ_SWITCH_BASE;

        } else if(json->at("request") == FINAL_APPROACH_STR){
            req = COMPV_REQ_FINAL_APPROACH;

        } else if(json->at("request") == GO_HOME_STR){
            req = COMPV_REQ_GO_HOME;
        } else if(json->at("request") == SAFE_POSITION){
            req = COMPV_REQ_SAFE_POSITION;
        } else if(json->at("request") == GET_ROBOT_STATE_STR){
            req = COMPV_REQ_GET_ROBOT_STATE;
        }
        else
            req = COMPV_REQ_INVALID;
    }
    catch (const nlohmann::json::exception &e) {
#ifdef DEBUG_JSON_ELEMENT
        std::cerr << "message: " << e.what() << "; " << "exception id: " << e.id << std::endl;
#endif
    }
    return req;
}

// Gets position values from JSON
// Returns: cartesian position (x,y,z,rotx,roty,rotz)
static Target_Parameters_t getTargetParametersFromJsonEntry(const json* json) {
    Target_Parameters_t pos{};

    try { pos.id = json->at("id").get<int>(); } catch (...) {std::cerr << "Warning: 'id' missing or wrong type\n";}
    try { pos.isReachable = json->at("isReachable").get<bool>(); } catch (...) {std::cerr << "Warning: 'isReachable' missing or wrong type\n";}
    try { pos.x1 = json->at("x1").get<double>(); } catch (...) {std::cerr << "Warning: 'x1' missing or wrong type\n";}
    try { pos.y1 = json->at("y1").get<double>(); } catch (...) {std::cerr << "Warning: 'y1' missing or wrong type\n";}
    try { pos.z1 = json->at("z1").get<double>(); } catch (...) {std::cerr << "Warning: 'z1' missing or wrong type\n";}
    try { pos.x2 = json->at("x2").get<double>(); } catch (...) {std::cerr << "Warning: 'x2' missing or wrong type\n";}
    try { pos.y2 = json->at("y2").get<double>(); } catch (...) {std::cerr << "Warning: 'y2' missing or wrong type\n";}
    try { pos.z2 = json->at("z2").get<double>(); } catch (...) {std::cerr << "Warning: 'z2' missing or wrong type\n";}

    return pos;
}

static std::vector<Target_Parameters_t> getTargetParametersFromJson(const nlohmann::json* json) {
    std::vector<Target_Parameters_t> positions;

    try {
        const auto& json_positions = json->at("positions");
        if (!json_positions.is_array()) {
            std::cerr << "Error: 'positions' is not an array\n";
            return positions;
        }

        for (const auto& item : json_positions) {
            Target_Parameters_t pos = getTargetParametersFromJsonEntry(&item);
            positions.push_back(pos);
        }

    } catch (const nlohmann::json::exception& e) {
#ifdef DEBUG_JSON_ELEMENT
        std::cerr << "Error parsing 'positions' array: " << e.what() << std::endl;
#endif
    }

    return positions;
}
