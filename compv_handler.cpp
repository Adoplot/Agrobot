#include "compv_handler.h"

#include <netinet/in.h>

#include "connection_handler.h"
#include "enet_handler.h"
#include "onltrack_handler.h"
#include "transform_calc.h"
#include "nlohmann/json.hpp"


using std::cout;
using std::cerr;
using std::endl;
using nlohmann::json;

static int sockfd_enet1;
static int sockfd_compv;
static sockaddr_in sockaddr_enet1;

static Cartesian_Pos_t targetPos_camFrame{};
static Cartesian_Pos_t targetPos_worldFrame{};

static Tcp_Data_t tcp_data_state = TCP_DATA_NOT_AVAILABLE;  // tracks if new cmd is received from compv

static json jsonParse(const std::string* data);
static CompV_Request_t getJsonRequest(const json* json);
static Cartesian_Pos_t getJsonPos(const json* json);


// Interface to set value of targetPos_worldFrame
void CompV_SetTargetPosWorldFrame(Cartesian_Pos_t new_value) {
    targetPos_worldFrame = new_value;
};


// Interface to set value of targetPos_camFrame
void CompV_SetTargetPosCamFrame(Cartesian_Pos_t new_value) {
    targetPos_camFrame = new_value;
};


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
void Compv_HandleCmd(const std::string* data){
    //Get socket data for ENET1
    sockfd_enet1 = Connection_GetSockfd(SOCKTYPE_ENET1);
    sockfd_compv = Connection_GetSockfd(SOCKTYPE_COMPV);
    sockaddr_enet1 = Connection_GetSockAddr(SOCKTYPE_ENET1);
    char buf_cmd[2]{};
    double distance2target{0};
    bool orientation_reached{false};

    // Declare jsons and strings to send later
    json json_send{};
    std::string string_send{};

    json json = jsonParse(data);
    CompV_Request_t request = getJsonRequest(&json);
    Hyundai_Data_t *eePos_worldFrame = Connection_GetEePosWorldFrame();

    switch (request) {
        case REQ_INVALID:
            std::cerr << "CompV_HandleCmd(): Cmd is not valid" << endl;
            break;

        case REQ_SET_POS: {
            cout << "Initiating <Set Pos> sequence" << endl;
            tcp_data_state = TCP_DATA_NEW_AVAILABLE;

            targetPos_camFrame = getJsonPos(&json);
            targetPos_worldFrame = Transform_ConvertFrameTarget2World(&targetPos_camFrame,
                                                                        eePos_worldFrame,
                                                                        SCISSORS_LENGTH);

            distance2target = Transform_CalcDistanceBetweenPoints(eePos_worldFrame,&targetPos_worldFrame);
            orientation_reached = Transform_CompareOrientations(ORIENTATION_ACCURACY, eePos_worldFrame, &targetPos_worldFrame);


            if ((distance2target <= POSITIONING_ACCURACY) && orientation_reached) {
                cout << "pos and ori is reached" << endl;
                json_send["request"] = "SET_POS";
                json_send["status"] = "COMPLETE";
                string_send = json_send.dump();
                Connection_SendTcp(sockfd_compv, &string_send);
            }
            else {
                json_send["request"] = "SET_POS";
                json_send["status"] = "IN_PROGRESS";
                string_send = json_send.dump();
                Connection_SendTcp(sockfd_compv, &string_send);
            }

            // ToDo: check if the rotation is complete?
            //  need to work out an algorithm with compv team
            break;
        }
        case REQ_RETURN_TO_BASE:
            cout << "Initiating <Return To Base> sequence" << endl;
            snprintf(buf_cmd,sizeof(buf_cmd),"%d", ENET_RETURN_TO_BASE);     //buffer should be null-terminated. sprintf ensures \0 at the end.
            Connection_SendUdp(sockfd_enet1, sockaddr_enet1, buf_cmd, sizeof(buf_cmd));
            bzero(buf_cmd,sizeof(buf_cmd));
            break;

        case REQ_CUT:
            cout << "Initiating <Cut> sequence" << endl;
            snprintf(buf_cmd,sizeof(buf_cmd),"%d",ENET_CUT);
            Connection_SendUdp(sockfd_enet1, sockaddr_enet1, buf_cmd, sizeof(buf_cmd));
            bzero(buf_cmd,sizeof(buf_cmd));
            break;

        case REQ_STORE:
            cout << "Initiating <Store> sequence" << endl;
            snprintf(buf_cmd,sizeof(buf_cmd),"%d",ENET_STORE);
            Connection_SendUdp(sockfd_enet1, sockaddr_enet1, buf_cmd, sizeof(buf_cmd));
            bzero(buf_cmd,sizeof(buf_cmd));
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


// Gets request type from parsed JSON.
// Returns: request type
static CompV_Request_t getJsonRequest(const json* json){
    CompV_Request_t req = REQ_INVALID;
    try {
        // recv xyz, immediately sends IN_PROGRESS if not at target pos,or COMPLETE if is at target pos.
        // Then convert frames, then calc incr, then rewrites global variable with incr to be sent to OnLTrack
        if (json->at("request") == "SET_POS")
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


