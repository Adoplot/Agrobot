#include <iostream>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <thread>
#include <map>
#include <cmath>
#include <fcntl.h>
#include <poll.h>

#include "connection_handler.h"
#include "onltrack_handler.h"

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
    Connection_Init();


    while(1){
        Connection_ProcessAction();
    }
};

