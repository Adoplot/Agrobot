#include <cstdio>
#include <netinet/in.h>

#include "onltrack_handler.h"
#include "compv_handler.h"
#include "connection_handler.h"
#include "transform_calc.h"
#include "robot_api.h"

using std::cout;
using std::cerr;
using std::endl;

typedef enum {
    ONLTRACK_CMD_INIT,
    ONLTRACK_CMD_START = 'S',
    ONLTRACK_CMD_PLAY = 'P',
    ONLTRACK_CMD_END = 'F'
} Onltrack_Cmd_t;

static Hyundai_Data_t sendIncrements{};

static void handleOnltrackStartCmd(const Hyundai_Data_t *eePos_worldFrame);
static void handleOnltrackPlayCmd(const Hyundai_Data_t *eePos_worldFrame);
static void handleOnltrackEndCmd();
static void printOnltrackData(const Hyundai_Data_t *data, bool send_or_recv);
static void zeroingPosIncrements(Hyundai_Data_t *increments);
static void zeroingOriIncrements(Hyundai_Data_t *increments);


// Handles messages received from Hyundai via onltrack
void Onltrack_AnswerHandle(const Hyundai_Data_t* eePos_worldFrame, long buflen){
    switch (eePos_worldFrame->Command) {
        case ONLTRACK_CMD_START:
            handleOnltrackStartCmd(eePos_worldFrame);
            break;

        case ONLTRACK_CMD_PLAY:
            if (Connection_GetCompvSocketState() == TCP_CLIENT_CONNECTED) {
                handleOnltrackPlayCmd(eePos_worldFrame);
            }
            break;

        case ONLTRACK_CMD_END:
            handleOnltrackEndCmd();
            break;

        case ONLTRACK_CMD_INIT:
            //TODO: did not receive cmd yet, just skip
            break;

        default:
            cerr << "Onltrack_AnswerHandle(): Unexpected cmd from OnlTrack received" << endl;
            //TODO: handle?
            break;
    }
}

// Handles Start cmd received from Hyundai via onltrack
// Answers to Start request, turns OnLTrack on
static void handleOnltrackStartCmd(const Hyundai_Data_t *eePos_worldFrame){
    int sockfd_onltrack = Connection_GetSockfd(SOCKTYPE_ONLTRACK);
    sockaddr_in sockaddr_onltrack = Connection_GetSockAddr(SOCKTYPE_ONLTRACK);

    printOnltrackData(eePos_worldFrame, PRINT_RECV);

    // Setup struct for Start answer
    Hyundai_Data_t sendStartCmd{};
    zeroingPosIncrements(&sendStartCmd);
    zeroingOriIncrements(&sendStartCmd);
    sendStartCmd.Command = ONLTRACK_CMD_START;
    sendStartCmd.Count = 0;
    sendStartCmd.State = 1;

    Connection_SendUdp(sockfd_onltrack, sockaddr_onltrack, &sendStartCmd, sizeof(sendStartCmd));

    printOnltrackData(&sendStartCmd, PRINT_SEND);
    printf("On-line tracking is started by Hi5 controller.\n");

    // Send first Play cmd to initiate the request-answer process
    Hyundai_Data_t sendPlayCmd{};
    zeroingPosIncrements(&sendPlayCmd);
    zeroingOriIncrements(&sendPlayCmd);
    sendPlayCmd.Command = ONLTRACK_CMD_PLAY;

    Connection_SendUdp(sockfd_onltrack, sockaddr_onltrack, &sendPlayCmd, sizeof(sendStartCmd));
    printOnltrackData(&sendPlayCmd, PRINT_SEND);

    Connection_SetOnltrackState(ONLTRACK_ON);
}


// Handles Play request received from Hyundai via onltrack
// Computes increments and sends them to Hyundai
static void handleOnltrackPlayCmd(const Hyundai_Data_t *eePos_worldFrame){
    int sockfd_onltrack = Connection_GetSockfd(SOCKTYPE_ONLTRACK);
    sockaddr_in sockaddr_onltrack = Connection_GetSockAddr(SOCKTYPE_ONLTRACK);
    Cartesian_Pos_t pos_increments{0,0,0,0,0,0};
    Cartesian_Pos_t ori_increments{0,0,0,0,0,0};

    Cartesian_Pos_t* targetPos_camFrame = Compv_GetTargetPosCamFrame();
    Cartesian_Pos_t* targetPos_worldFrame = Compv_GetTargetPosWorldFrame();

    //printOnltrackData(eePos_worldFrame, PRINT_RECV);

    // Send zero increments while zeros are received from Compv, so robot does not move
    if (((targetPos_camFrame->x == 0) && (targetPos_camFrame->y == 0) && (targetPos_camFrame->z == 0)) ||
            !RobotAPI_IsApproachSequenceActive() ||
            !RobotAPI_IsFinalApproachSequenceActive()) {

        zeroingPosIncrements(&sendIncrements);
    }
    else {
        pos_increments = Transform_CalculatePositionIncrements(eePos_worldFrame, targetPos_worldFrame);
    }

    double distance2target = Transform_CalcDistanceBetweenPoints(eePos_worldFrame, targetPos_worldFrame);
    bool orientation_reached = Transform_CompareOrientations(ORIENTATION_ACCURACY, eePos_worldFrame, targetPos_worldFrame);

    if ((distance2target <= POSITIONING_ACCURACY) && orientation_reached) {
        RobotAPI_EndSequence(Robot_Sequence_Result_t::SUCCESS);
    } else {
        // Send increments to hyundai

        sendIncrements.coord[0] = pos_increments.x;
        sendIncrements.coord[1] = pos_increments.y;
        sendIncrements.coord[2] = pos_increments.z;
        sendIncrements.coord[3] = ori_increments.rotx;
        sendIncrements.coord[4] = ori_increments.roty;
        sendIncrements.coord[5] = ori_increments.rotz;

        sendIncrements.Command = ONLTRACK_CMD_PLAY;

        Connection_SendUdp(sockfd_onltrack, sockaddr_onltrack, &sendIncrements, sizeof(sendIncrements));
    }

    for(int i = 0; i < 3; i++){
        if(sendIncrements.coord[i] > 0.5){
            printOnltrackData(&sendIncrements, PRINT_SEND);
        }
    }

    //printOnltrackData(&sendIncrements, PRINT_SEND);
}


// Handles F request received from Hyundai via onltrack
// Turns off OnLTrack
static void handleOnltrackEndCmd(){
    printf("On-line tracking is finished by Hi5 controller.\n");
    // Clear the value of targetPos_worldFrame, so after onltrack reset robot would wait for new value
    Cartesian_Pos_t zeros{0,0,0,0,0,0};
    CompV_SetTargetPosCamFrame(zeros);
    CompV_SetTargetPosWorldFrame(zeros);

    Connection_SetOnltrackState(ONLTRACK_OFF);
}


// Prints sendRobotDataUDP and recvRobotDataUDP in mm and rad for debugging
static void printOnltrackData(const Hyundai_Data_t* data, bool send_or_recv)
{
    if(send_or_recv == PRINT_SEND){
        printf("send>> Command: %c, Count: %d, mm,rad, [X: %.3f, Y: %.3f, Z: %.3f, Rx: %.3f, Ry: %.3f, Rz: %.3f]\n",
           data->Command,
           data->Count,
           data->coord[0] * 1000, // m -> mm
           data->coord[1] * 1000,
           data->coord[2] * 1000,
           data->coord[3], // rad
           data->coord[4],
           data->coord[5]);
    }
    if(send_or_recv == PRINT_RECV){
        printf("recv>> Command: %c, Count: %d, mm,rad, [X: %.3f, Y: %.3f, Z: %.3f, Rx: %.3f, Ry: %.3f, Rz: %.3f]\n",
           data->Command,
           data->Count,
           data->coord[0] * 1000, // m -> mm
           data->coord[1] * 1000,
           data->coord[2] * 1000,
           data->coord[3], // rad
           data->coord[4],
           data->coord[5]);
    }
}


// Writes zeros to position increments
static void zeroingPosIncrements(Hyundai_Data_t *increments) {
    increments->coord[0] = 0;
    increments->coord[1] = 0;
    increments->coord[2] = 0;
}


// Writes zeros to orientation increments
static void zeroingOriIncrements(Hyundai_Data_t *increments) {
    increments->coord[3] = 0;
    increments->coord[4] = 0;
    increments->coord[5] = 0;
}