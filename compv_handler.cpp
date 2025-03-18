#include "compv_handler.h"

#include <netinet/in.h>

#include "connection_handler.h"
#include "enet_handler.h"
#include "onltrack_handler.h"
#include "transform_calc.h"
#include "nlohmann/json.hpp"
#include "robot_api.h"


using std::cout;
using std::cerr;
using std::endl;
using nlohmann::json;

static int sockfd_enet1;
constexpr const char* NONE_STR = "NONE";
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
static sockaddr_in sockaddr_enet1;

static Cartesian_Pos_t targetPos_camFrame{};
static Cartesian_Pos_t targetPos_worldFrame{};

static Tcp_Data_t tcp_data_state = TCP_DATA_NOT_AVAILABLE;  // tracks if new cmd is received from compv

static json jsonParse(const std::string* data);
static CompV_Request_t getJsonRequest(const json* json);
static Cartesian_Pos_t getJsonPos(const json* json);
static std::vector<Cartesian_Pos_t> getJsonPositions(const json* json);

static void handleSetPositionRequest(const json& json);
static void sendUnreachableResponse();
static void sendCompleteResponse();
static void sendInProgressResponse();
static void sendRobotCommand(int command, const std::string& action_name);
static void handleSyncTargetsRequest(const json& json);
static void sendSyncTargetsResponse(std::vector<Cartesian_Pos_t> positions);

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
    sockfd_enet1 = Connection_GetSockfd(SOCKTYPE_ENET1);
    sockfd_compv = Connection_GetSockfd(SOCKTYPE_COMPV);
    sockaddr_enet1 = Connection_GetSockAddr(SOCKTYPE_ENET1);

    // Parse JSON and get request type
    json json = jsonParse(data);
    CompV_Request_t request = getJsonRequest(&json);

    switch (request) {
        case REQ_INVALID:
            std::cerr << "CompV_HandleCmd(): Cmd is not valid" << endl;
            break;

        case REQ_SYNC_TARGETS:
            cout << "Requested target synchronization" << endl;
            handleSyncTargetsRequest(json);
            break;

        case REQ_SET_POS:
            handleSetPositionRequest(json);
            break;

        case REQ_RETURN_TO_BASE:
            sendRobotCommand(ENET_RETURN_TO_BASE, "Return To Base");
            break;

        case REQ_CUT:
            sendRobotCommand(ENET_CUT, "Cut");
            break;

        case REQ_STORE:
            sendRobotCommand(ENET_STORE, "Store");
            break;

        default:
            cerr << "CompV_HandleCmd(): Unexpected cmd" << endl;
            break;
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

static void handleSyncTargetsRequest(const json& json){
    cout    << "Received Sync Targets request\n"
            << "Checking if targets are reachable" << endl;

    std::vector<Cartesian_Pos_t> positions = getJsonPositions(&json);

    for (auto& pos : positions) {

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
    cout << "Initiating <Set Pos> sequence" << endl;

    Hyundai_Data_t *eePos_worldFrame = Connection_GetEePosWorldFrame();

    tcp_data_state = TCP_DATA_NEW_AVAILABLE;
    targetPos_camFrame = getJsonPos(&json);
    targetPos_worldFrame = Transform_ConvertFrameTarget2World(&targetPos_camFrame,
                                                              eePos_worldFrame,
                                                              SCISSORS_LENGTH);

    if (!RobotAPI_TargetIsReachable(&targetPos_worldFrame)) {
        sendUnreachableResponse();
        return;
    }

    double distance2target = Transform_CalcDistanceBetweenPoints(eePos_worldFrame, &targetPos_worldFrame);
    bool orientation_reached = Transform_CompareOrientations(ORIENTATION_ACCURACY, eePos_worldFrame, &targetPos_worldFrame);

    if ((distance2target <= POSITIONING_ACCURACY) && orientation_reached) {
        sendCompleteResponse();
    } else {
        sendInProgressResponse();
    }
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

static void sendRobotCommand(int command, const std::string& action_name) {
    cout << "Initiating <" << action_name << "> sequence" << endl;
    char buf_cmd[2]{};
    snprintf(buf_cmd, sizeof(buf_cmd), "%d", command);
    RobotAPI_SendCmd(sockfd_enet1, sockaddr_enet1, buf_cmd, sizeof(buf_cmd));
}

// Gets request type from parsed JSON.
// Returns: request type
static CompV_Request_t getJsonRequest(const json* json){
    CompV_Request_t req = REQ_INVALID;
    try {
        // recv xyz, immediately sends IN_PROGRESS if not at target pos,or COMPLETE if is at target pos.
        // Then convert frames, then calc incr, then rewrites global variable with incr to be sent to OnLTrack
        if (json->at("request") == COMPV_REQUEST_SET_POSITION)
            req = REQ_SET_POS;

        // send ENET1 command to robot, robot turns off Onltrack, executes ptp motion to home pos
        // and sends complete via ENET1
        else if (json->at("request") == "RETURN_TO_BASE")
            req = REQ_RETURN_TO_BASE;

        // same but closes gripper instead of ptp motion
        else if (json->at("request") == "CUT")
            req = REQ_CUT;

        // same but uses ptp motion position to the storing position
        else if (json->at("request") == "STORE")
            req = REQ_STORE;

        else if(json->at("request") == COMPV_REQUEST_SYNC_TARGETS){
            req = REQ_SYNC_TARGETS;
        }
        else
            req = REQ_INVALID;
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
