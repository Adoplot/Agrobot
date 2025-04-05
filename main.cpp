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


int main(void)
{
    //Cartesian_Pos_t *test{};
    //std::cout << RobotAPI_TargetIsReachable(test) << endl;

    CompV_Init();
    Connection_Init();

    while(1){
        Connection_ProcessAction();
        RobotAPI_ProcessAction();
    }
};

