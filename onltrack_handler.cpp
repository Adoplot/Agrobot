#include <cstdio>
#include <netinet/in.h>

#include "onltrack_handler.h"
#include "compv_handler.h"
#include "connection_handler.h"
#include "transform_calc.h"
#include "robot_api.h"
#include <fstream>
#include <iomanip>  // std::setprecision()
#include "Matlab_ik.h"

using std::cout;
using std::cerr;
using std::endl;

typedef enum {
    ONLTRACK_CMD_INIT,
    ONLTRACK_CMD_START = 'S',
    ONLTRACK_CMD_PLAY = 'P',
    ONLTRACK_CMD_END = 'F'
} Onltrack_Cmd_t;

static int pathIndexCounter {0}; //todo: implement reset before starting new sequence

std::ofstream fs("/home/adoplot/CLionProjects/Agrobot/log_increments.txt");

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
    double increments[6] {};
    bool incrementsIsValid {false};
    Cartesian_Pos_t pos_increments{0,0,0,0,0,0};
    bool pathEndReached = false;
    bool collisionDetected = false;

    Cartesian_Pos_t* targetPos_worldFrame = Compv_GetTargetPosWorldFrame();

    //printOnltrackData(eePos_worldFrame, PRINT_RECV);
    auto robotPath = RobotAPI_GetPathCopy();

    // Send zero increments while robot is not performing approach or final approach
    if (    !RobotAPI_IsApproachSequenceActive() &&
            !RobotAPI_IsFinalApproachSequenceActive()) {

        zeroingPosIncrements(&sendIncrements);
    } else {

        if (pathIndexCounter < PATH_STEP_NUM) {
            incrementsIsValid = Transform_getIncrements(robotPath, PATH_STEP_NUM,
                                                        pathIndexCounter, eePos_worldFrame, increments);

            //Get latest robot configuration
            double *robotConfig = RobotAPI_GetCurrentConfig();

            //Check for axis limits
            for (int i = 0; i < 6; i++) {
                if (!IK_AxisInLimits(robotConfig[i], i)) {
                    cerr << "Axis [" << i+1 << "] collision detected" << endl;
                    collisionDetected = true;
                }
            }

            //Check for self-collision
            bool isSelfColliding;
            double collPairs[2];
            Matlab_checkCollision(robotConfig, &isSelfColliding, collPairs);
            if (isSelfColliding) {
                cerr << "Self collision detected between axis " << collPairs[0] << " and " << collPairs[1] << endl;
                collisionDetected = true;
            }

            // If something wrong - stop the motion
            if (collisionDetected || !incrementsIsValid) {
                zeroingPosIncrements(&sendIncrements);
                zeroingOriIncrements(&sendIncrements);

            } else {
                pos_increments.x = increments[0];
                pos_increments.y = increments[1];
                pos_increments.z = increments[2];
                pos_increments.rotx = increments[3];
                pos_increments.roty = increments[4];
                pos_increments.rotz = increments[5];
            }

            pathIndexCounter++;

        } else {
            pathEndReached = true;
            zeroingPosIncrements(&sendIncrements);
            zeroingOriIncrements(&sendIncrements);
        }
    }

    // Send increments to hyundai
    sendIncrements.coord[0] = pos_increments.x;
    sendIncrements.coord[1] = pos_increments.y;
    sendIncrements.coord[2] = pos_increments.z;
    sendIncrements.coord[3] = pos_increments.rotx;
    sendIncrements.coord[4] = pos_increments.roty;
    sendIncrements.coord[5] = pos_increments.rotz;

    //Logging into log_increments.txt
    if (!fs) {
        std::cerr << "Cannot open the output file." << std::endl;
    } else {
        fs << std::fixed << std::setprecision(6) << endl;
        fs << sendIncrements.coord[0] << "\t";
        fs << sendIncrements.coord[1] << "\t";
        fs << sendIncrements.coord[2] << "\t";
        fs << sendIncrements.coord[3] << "\t";
        fs << sendIncrements.coord[4] << "\t";
        fs << sendIncrements.coord[5];
        fs << std::endl;
    }

    if (pathEndReached) {
        cout << "Target reached\n" << endl;
        pathIndexCounter = 0;
        RobotAPI_EndSequence(Robot_Sequence_Result_t::SUCCESS);

    } else if (collisionDetected || !incrementsIsValid) {
        cout << "Target unreachable, ending sequence\n" << endl;
        pathIndexCounter = 0;
        RobotAPI_EndSequence(Robot_Sequence_Result_t::UNREACHABLE);
    }

    sendIncrements.Command = ONLTRACK_CMD_PLAY;

    Connection_SendUdp(sockfd_onltrack, sockaddr_onltrack, &sendIncrements, sizeof(sendIncrements));

    for (int i = 0; i < 3; i++) {
        if (sendIncrements.coord[i] > 0.5) {
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

    // Close log_increments.txt
    fs.close();
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