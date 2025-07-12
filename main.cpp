#include <iostream>
#include "connection_handler.h"
#include "compv_handler.h"
#include "robot_api.h"
#include <iomanip>
#include "ik_wrapper.h"
#include "tests.h"

using std::cout;
using std::cerr;
using std::endl;

/*
{"request": "RETURN_TO_BASE"}
{"request": "STORE"}
{"request": "CUT"}
{"request": "GET_COORDS"}

{"request": "SET_POS", "x": 0, "y": 0, "z": 0.4}
{"request": "SET_POS", "x": 0.11, "y": 0, "z": 0.290}
{"request": "SET_POS", "x": 0, "y": 0, "z": 0.315}
{"request": "SET_POS", "rotx": 0, "roty": 0, "rotz": 0.57}
{"request": "SET_POS", "rotx": -0.7, "roty": 0, "rotz": 0}
{"request": "SET_POS", "rotx": 1.4, "roty": 0, "rotz": 0}
{"request": "SET_POS", "x": 0, "y": 0, "z": 0.315, "rotx": -0.05, "roty": 0, "rotz": -0.05}
 */

/*
vstart  [1.1,   0,  0.8]
vend    [1.1,  -1,  0.8]
{"request" : "SYNC_TARGETS","positions": [{"id": 1,"isReachable": false,"x1": -0.0247,"y1": -0.2511,"z1": 0.4641,"x2": 0.9704,"y2": -0.3361,"z2": 0.5151},{"id": 2,"isReachable": false,"x1": 10.5,"y1": 20.3,"z1": 30.1,"x2": 10.5,"y2": 20.3,"z2": 30.1}]}
{"request" : "APPROACH","positions": [{"id": 1,"isReachable": true,"x1": -0.0247,"y1": -0.2511,"z1": 0.4641,"x2": 0.9704,"y2": -0.3361,"z2": 0.5151}]}

{"request" : "APPROACH","positions": [{"id": 1,"isReachable": true,"x1": 0.014,"y1": 0.016,"z1": 0.484,"x2": -0.053,"y2": 0.029,"z2": 0.507}]}
{"request" : "FINAL_APPROACH","positions": [{"id": 1,"isReachable": true,"x1": 0.014,"y1": 0.016,"z1": 0.484,"x2": -0.053,"y2": 0.029,"z2": 0.507}]}

{"request" : "APPROACH","positions": [{"id": 1,"isReachable": true,"x1":-0.04486518353223801,"x2":0.0023883781395852566,"y1":0.10394474118947983,"y2":0.11216890811920166,"z1":0.47699999809265137,"z2":0.45899999141693115}]}
{"request" : "FINAL_APPROACH","positions": [{"id": 1,"isReachable": true,"x1":-0.04486518353223801,"x2":0.0023883781395852566,"y1":0.10394474118947983,"y2":0.11216890811920166,"z1":0.47699999809265137,"z2":0.45899999141693115}]}


 {"request" : "SWITCH_BASE_NEXT"}
{"request" : "GET_ROBOT_STATE"}
{"request" : "GO_HOME"}

robot_configuration 0.372 66.433 -35.118 4.085 53.841 -2.503
 */


int main(void)
{
    //Test_Run();
    Connection_Init();

    while(1){
        Connection_ProcessAction();
        RobotAPI_ProcessAction();
    }
};

