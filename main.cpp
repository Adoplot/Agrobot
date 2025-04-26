#include <iostream>
#include "connection_handler.h"
#include "compv_handler.h"
#include "robot_api.h"
#include <iomanip>
#include "ik_wrapper.h"

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
{"request" : "SET_POS","positions": [{"id": 1,"isReachable": false,"x1": -0.0247,"y1": -0.2511,"z1": 0.4641,"x2": 0.9704,"y2": -0.3361,"z2": 0.5151}]}

{"request" : "SWITCH_BASE_NEXT"}
{"request" : "GO_HOME"}

robot_configuration 0.372 66.433 -35.118 4.085 53.841 -2.503
 */


int main(void)
{
    CompV_Init();
    Connection_Init();

    while(1){
        Connection_ProcessAction();
        RobotAPI_ProcessAction();
    }
};

