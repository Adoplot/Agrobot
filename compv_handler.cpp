#include "compv_handler.h"

#include "connection_handler.h"
#include "onltrack_handler.h"
#include "transform_calc.h"
#include "nlohmann/json.hpp"
#include "robot_api.h"


using std::cout;
using std::cerr;
using std::endl;
using nlohmann::json;

constexpr const char* NONE_STR = "IDLE";
constexpr const char* APPROACH_STR = "APPROACH";
constexpr const char* FINAL_APPROACH_STR = "FINAL_APPROACH";
constexpr const char* CUT_STR = "CUT";
constexpr const char* STORE_STR = "STORE";
constexpr const char* RETURN_TO_BASE_STR = "RETURN_TO_BASE";
constexpr const char* SWITCH_BASE_STR = "SWITCH_BASE";

constexpr const char* COMPV_ANSW_UNREACHABLE = "UNREACHABLE";
constexpr const char* COMPV_ANSW_COMPLETE = "COMPLETE";
constexpr const char* COMPV_ANSW_IN_PROGRESS = "IN_PROGRESS";
constexpr const char* COMPV_ANSW_FAIL = "FAIL";
constexpr const char* COMPV_ANSW_REQUESTED = "REQUESTED";

constexpr const char* COMPV_REQUEST_SYNC_TARGETS = "SYNC_TARGETS";
constexpr const char* COMPV_REQUEST_SET_POSITION = "SET_POS";
constexpr const char* COMPV_REQUEST_CUT = "CUT";

static int sockfd_compv;

static Cartesian_Pos_t targetPos_camFrame{};
static Cartesian_Pos_t targetPos_worldFrame{};

static Tcp_Data_t tcp_data_state = TCP_DATA_NOT_AVAILABLE;  // tracks if new cmd is received from compv

static json jsonParse(const std::string* data);
static CompV_Request_t getJsonRequest(const json* json);
static Cartesian_Pos_t getJsonPos(const json* json);
static std::vector<Cartesian_Pos_t> getJsonPositions(const json* json);

static void onRobotSequenceEvent(Robot_Sequence_t sequence, Robot_Sequence_State_t state);

static void sendUnreachableResponse();
static void sendCompleteResponse();
static void sendInProgressResponse();
static void sendSyncTargetsResponse(std::vector<Cartesian_Pos_t> positions);
static void sendStatusResponse(const char* request, const char* status);

static void handleSyncTargetsRequest(const json& json);
static void handleSetPositionRequest(const json& json);
static void handleFinalApproachRequest(const json& json);
static void handleCutRequest();
static void handleStoreRequest();
static void handleReturnToBaseRequest();
static void handleSwitchBaseRequest();

static const char* sequenceToString(Robot_Sequence_t sequence);
static const char* sequenceStepToString(Robot_Sequence_State_t state);

static const char* sequenceToString(Robot_Sequence_t sequence) {
    switch (sequence) {
        case Robot_Sequence_t::IDLE: return NONE_STR;
        case Robot_Sequence_t::APPROACH: return APPROACH_STR;
        case Robot_Sequence_t::FINAL_APPROACH: return FINAL_APPROACH_STR;
        case Robot_Sequence_t::CUT: return CUT_STR;
        case Robot_Sequence_t::STORE: return STORE_STR;
        case Robot_Sequence_t::RETURN_TO_BASE: return RETURN_TO_BASE_STR;
        case Robot_Sequence_t::SWITCH_BASE: return SWITCH_BASE_STR;
        default: return "UNKNOWN";
    }
}

static const char* sequenceStepToString(Robot_Sequence_State_t state) {
    switch (state) {
        case Robot_Sequence_State_t::INIT: return COMPV_ANSW_FAIL;
        case Robot_Sequence_State_t::REQUESTED: return COMPV_ANSW_REQUESTED;
        case Robot_Sequence_State_t::COMPLETE: return COMPV_ANSW_COMPLETE;
        case Robot_Sequence_State_t::FAIL: return COMPV_ANSW_FAIL;
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
    // Get socket data
    sockfd_compv = Connection_GetSockfd(SOCKTYPE_COMPV);

    // Parse JSON and get request type
    json json = jsonParse(data);
    CompV_Request_t request = getJsonRequest(&json);

    switch (request) {
        case COMPV_REQ_INVALID:
            std::cerr << "CompV_HandleCmd(): Cmd is not valid" << endl;
            break;

        case COMPV_REQ_SYNC_TARGETS:
            handleSyncTargetsRequest(json);
            break;

        case COMPV_REQ_SET_POS:
            handleSetPositionRequest(json);
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

        default:
            cerr << "CompV_HandleCmd(): Unexpected cmd" << endl;
            break;
    }
}

static void handleReturnToBaseRequest(){
    RobotAPI_StartReturnToBaseSequence();
}

static void handleSwitchBaseRequest(){
    RobotAPI_StartSwitchBaseSequence();
}

static void handleStoreRequest(){
    RobotAPI_StartStoreSequence();
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
    RobotAPI_StartCutSequence();
}

static void sendStatusResponse(const char* request, const char* status){
    cout << "Answer to CompV: status response" << endl; //todo elaborate
    json json_send;
    json_send["request"] = request;
    json_send["status"] = status;
    std::string string_send = json_send.dump();
    Connection_SendTcp(sockfd_compv, &string_send);
}

static void handleSyncTargetsRequest(const json& json){
    cout    << "Received Sync Targets request\n"
            << "Checking if targets are reachable" << endl;

    std::vector<Cartesian_Pos_t> positions = getJsonPositions(&json); //cam frame
    Hyundai_Data_t *eePos_worldFrame = Connection_GetEePosWorldFrame();

    for (auto& pos : positions) {
        Cartesian_Pos_t targetWorldFrame = Transform_ConvertFrameTarget2World(&pos,
                                                                              eePos_worldFrame,
                                                                              SCISSORS_LENGTH);
        if(RobotAPI_TargetIsReachable(&pos)){
            cout << "Target ID: " << pos.id << "is reachable\n"
                << "Coordinates:\n"
                << "x: " << pos.x << "\n"
                << "y: " << pos.y << "\n"
                << "z: " << pos.z << "\n" << endl;

            pos.isReachable = true;
        } else {
            cout << "Target ID: " << pos.id << "is NOT reachable\n"
                 << "Coordinates:\n"
                 << "x: " << pos.x << "\n"
                 << "y: " << pos.y << "\n"
                 << "z: " << pos.z << "\n" << endl;

            pos.isReachable = false;
        }
    }

    sendSyncTargetsResponse(positions);
}

static void positionToJson(json& j, const Cartesian_Pos_t& pos) {
    j = json{
            {"id", pos.id},
            {"isReachable", pos.isReachable},
            {"x", pos.x},
            {"y", pos.y},
            {"z", pos.z},
            {"rotx", pos.rotx},
            {"roty", pos.roty},
            {"rotz", pos.rotz}
    };
}

static void sendSyncTargetsResponse(std::vector<Cartesian_Pos_t> positions){
    cout << "Answer to CompV: Reachable targets JSON" << endl;
    json json_send;

    // Serialize vector of Cartesian_Pos_t using positionToJson()
    json json_positions = json::array();
    for (const auto& pos : positions) {
        json j;
        positionToJson(j, pos);
        json_positions.push_back(j);
    }
    json_send["request"] = COMPV_REQUEST_SYNC_TARGETS;
    json_send["status"] = COMPV_ANSW_COMPLETE;
    json_send["positions"] = json_positions;

    std::string string_send = json_send.dump();
    Connection_SendTcp(sockfd_compv, &string_send);
}

static void handleSetPositionRequest(const json& json) {
    /*
    cout << "Initiating <Set Pos> sequence" << endl;

    Hyundai_Data_t *eePos_worldFrame = Connection_GetEePosWorldFrame();

    tcp_data_state = TCP_DATA_NEW_AVAILABLE;
    targetPos_camFrame = getJsonPos(&json);
    targetPos_worldFrame = Transform_ConvertFrameTarget2World(&targetPos_camFrame,
                                                              eePos_worldFrame,
                                                              SCISSORS_LENGTH);
*/
    //TEST PARAMETERS---------------------------------------
    /*
    Hyundai_Data_t *eePos_worldFrame;
    eePos_worldFrame->coord[0] = 0.5715;
    eePos_worldFrame->coord[1] = 0;
    eePos_worldFrame->coord[2] = 0.931;
    eePos_worldFrame->coord[3] = 0;
    eePos_worldFrame->coord[4] = 1.5708;
    eePos_worldFrame->coord[5] = 0;
     */
    typedef struct{
        double x1  {0};
        double y1  {0};
        double z1  {0};
        double x2  {0};
        double y2  {0};
        double z2  {0};
    }Target_Parameters_t;
    Target_Parameters_t targetParameters {0,0,0,0,0,0};
    Cartesian_Pos_t targetStart_camFrame{};
    Cartesian_Pos_t targetDir_camFrame{};
    Cartesian_Pos_t targetStart_worldFrame{};
    Cartesian_Pos_t targetDir_worldFrame{};
    //TEST PARAMETERS END----------------------------------

    //Divide targetParameters into targetStart_camFrame and targetDir_camFrame
    targetStart_camFrame.x = targetParameters.x1;
    targetStart_camFrame.y = targetParameters.y1;
    targetStart_camFrame.z = targetParameters.z1;

    targetDir_camFrame.x = targetParameters.x2;
    targetDir_camFrame.y = targetParameters.y2;
    targetDir_camFrame.z = targetParameters.z2;

    //Get current robot EE coords
    Hyundai_Data_t *eePos_worldFrame = Connection_GetEePosWorldFrame();

    //Transform targetParameters to worldFrame
    targetStart_worldFrame = Transform_ConvertFrameTarget2World(&targetStart_camFrame,eePos_worldFrame);
    targetDir_worldFrame = Transform_ConvertFrameTarget2World(&targetDir_camFrame,eePos_worldFrame);

    //Show targetStart_worldFrame coords
    std::cout << std::fixed << std::showpoint;
    std::cout << std::setprecision(3);
    std::cout << "x=" << targetStart_worldFrame.x;
    std::cout << " y=" << targetStart_worldFrame.y;
    std::cout << " z=" << targetStart_worldFrame.z;
    std::cout << " rotX=" << targetStart_worldFrame.rotx;
    std::cout << " rotY=" << targetStart_worldFrame.roty;
    std::cout << " rotZ=" << targetStart_worldFrame.rotz;
    cout << endl;

    //ToDo: ADOPLOT
    // + get targetParameters and divide them into targetStart_camFrame and targetDir_camFrame
    // + convert them from camFrame to worldFrame
    // get sortedList
    // receive currentConfig from hyundai
    // loop through sortedList with getGikFull and output qWaypoints or send failMessage
    // get pathApr from Transform_getTrajectory
    // save pathApr for further onltrack increment calculations

    RobotAPI_StartApproachSequence();
}

static void handleFinalApproachRequest(const json& json) {
    cout << "Initiating <Final Approach> sequence" << endl;

    Hyundai_Data_t *eePos_worldFrame = Connection_GetEePosWorldFrame();

    tcp_data_state = TCP_DATA_NEW_AVAILABLE;
    targetPos_camFrame = getJsonPos(&json);
    targetPos_worldFrame = Transform_ConvertFrameTarget2World(&targetPos_camFrame,
                                                              eePos_worldFrame,
                                                              SCISSORS_LENGTH);


    RobotAPI_StartFinalApproachSequence();
}

static void sendUnreachableResponse() {
    cout << "Answer to CompV: Unreachable" << endl;
    json json_send;
    json_send["request"] = COMPV_REQUEST_SET_POSITION;
    json_send["status"] = COMPV_ANSW_UNREACHABLE;
    std::string string_send = json_send.dump();
    Connection_SendTcp(sockfd_compv, &string_send);
}

static void sendCompleteResponse() {
    cout << "Answer to CompV: Complete" << endl;
    json json_send;
    json_send["request"] = COMPV_REQUEST_SET_POSITION;
    json_send["status"] = COMPV_ANSW_COMPLETE;
    std::string string_send = json_send.dump();
    Connection_SendTcp(sockfd_compv, &string_send);
}

static void sendInProgressResponse() {
    cout << "Answer to CompV: In Progress" << endl;
    json json_send;
    json_send["request"] = COMPV_REQUEST_SET_POSITION;
    json_send["status"] = COMPV_ANSW_IN_PROGRESS;
    std::string string_send = json_send.dump();
    Connection_SendTcp(sockfd_compv, &string_send);
}

static void sendCutSequenceResponse(){
    cout << "Answer to CompV: In Progress" << endl;
    json json_send;
    json_send["request"] = COMPV_REQUEST_CUT;
    json_send["status"] = COMPV_ANSW_IN_PROGRESS;
    std::string string_send = json_send.dump();
    Connection_SendTcp(sockfd_compv, &string_send);
}

// Gets request type from parsed JSON.
// Returns: request type
static CompV_Request_t getJsonRequest(const json* json){
    CompV_Request_t req = COMPV_REQ_INVALID;
    try {
        // recv xyz, immediately sends IN_PROGRESS if not at target pos,or COMPLETE if is at target pos.
        // Then convert frames, then calc incr, then rewrites global variable with incr to be sent to OnLTrack
        if (json->at("request") == COMPV_REQUEST_SET_POSITION)
            req = COMPV_REQ_SET_POS;

        // send ENET1 command to robot, robot turns off Onltrack, executes ptp motion to home pos
        // and sends complete via ENET1
        else if (json->at("request") == "RETURN_TO_BASE")
            req = COMPV_REQ_RETURN_TO_BASE;

        // same but closes gripper instead of ptp motion
        else if (json->at("request") == "CUT")
            req = COMPV_REQ_CUT;

        // same but uses ptp motion position to the storing position
        else if (json->at("request") == "STORE")
            req = COMPV_REQ_STORE;

        else if(json->at("request") == COMPV_REQUEST_SYNC_TARGETS){
            req = COMPV_REQ_SYNC_TARGETS;
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
static Cartesian_Pos_t getJsonPos(const json* json) {
    Cartesian_Pos_t pos{};

    const std::vector<std::pair<std::string, double&>> jsonMappings = {
        {"x",   pos.x},
        {"y",   pos.y},
        {"z",   pos.z},
        {"rotx",pos.rotx},
        {"roty",pos.roty},
        {"rotz",pos.rotz}
    };

    for (const auto &[key, value]: jsonMappings) {
        try {
            value = json->at(key).get<double>();
        } catch (const nlohmann::json::exception &e) {
#ifdef DEBUG_JSON_ELEMENT
            std::cerr << "message: " << e.what() << "; " << "exception id: " << e.id << std::endl;
#endif
        }
    }

    return pos;
}

static std::vector<Cartesian_Pos_t> getJsonPositions(const nlohmann::json* json) {
    std::vector<Cartesian_Pos_t> positions;

    try {
        for (const auto& item : json->at("positions")) {
            positions.push_back(getJsonPos(&item));
        }
    } catch (const nlohmann::json::exception& e) {
#ifdef DEBUG_JSON_ELEMENT
        std::cerr << "Error parsing positions array: " << e.what() << std::endl;
#endif
    }

    return positions;
}

static void onRobotSequenceEvent(Robot_Sequence_t sequence, Robot_Sequence_State_t state) {
    const char* sequenceName = sequenceToString(sequence);
    const char* currentSequenceState = sequenceStepToString(state);

    sendStatusResponse(sequenceName, currentSequenceState);
}

void CompV_Init(){
    RobotAPI_SetSequenceCallback(onRobotSequenceEvent);
}
