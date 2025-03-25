#include "Matlab_ik_types.h"
#include "Matlab_ik.h"
#include "connection_handler.h"
#include "robot_api.h"
#include <netinet/in.h>
#include <iomanip>
#include "ik_wrapper.h"


/*
 * Calculates waypoints for Approach sequence.
 * Calculates circle around the cutplace, calculates sorted list of points for Approach sequence, chooses valid
 * waypoints for Approach and FinalApr sequence.
 * Input:
 *
 * Output:
 * code         - int, exit code: 0 - Apr fail, 1 - Success, 2 - FinalApr fail
 * qWaypoints   - 3x6 array with 3 robot configurations: 1row - current, 2 - Approach, 3 - FinalApr. (in radians)
 */
void IK_getWaypointsForApproach(const double branchStart[3], const double branchDir[3], const double eePos_worldFrame[3],
                                const double currentConfig[6], int *code, double qWaypoints[18]) {

    coder::array<double, 2U> sortedList;
    double listLength {0};  //redundant

    //Get sortedList with points on a circle around the cutting place
    Matlab_getSortedCirclePointList(CIRCLE_RADIUS, branchStart, branchDir, CIRCLE_POINT_NUM, eePos_worldFrame, sortedList, &listLength);

    double exitCode {0};
    int n {0};
    struct1_T solutionInfoApr {}; //only for debug
    //Initialize solver parameters
    struct0_T solverParameters {};
    IK_InitSolverParameters(&solverParameters);

    //Loop through sortedList points and try to find valid waypoints
    //Loop through n points until found valid robot waypoints or until half of the points were checked.
    while ((*code != 1) && (n < CIRCLE_POINT_NUM/2) ){
        //Prepare targetApr from n_th row of sortedList
        double targetApr[8] {0};
        for(int i=0; i<8;i++){
            targetApr[i] = sortedList.at(n, i);
        }
        //Check if robot can reach both waypoints (on circle and cutplace) and if so, output waypoints
        Matlab_getGikFull(currentConfig, targetApr, branchStart, &solverParameters,&exitCode,&solutionInfoApr,qWaypoints);
        *code = static_cast<int>(exitCode+0.1);  //to safely cast double to int, because (0.9999 -> 0)
        n++;
    }

    if (*code == 1){
        std::cout << "found solution on n = " << n << std::endl;
    }
    else{
        std::cout << "has not found solution for first " << n << " points" << std::endl;
    }
}


// Prints to console values in qWaypoints[18]. There are 3 rows and 6 columns.
void IK_PrintWaypoints(const double qWaypoints[18]){
    std::cout << std::fixed;
    std::cout << std::setprecision(4);
    int rows = 3;
    int cols = 6;

    for (int i=0; i< rows; i++){
        for (int j=0; j< cols; j++){
            std::cout << qWaypoints[j*rows+i] << "\t";
        }
        std::cout << std::endl;
    }
}


// ToDo: potentially static
void IK_InitSolverParameters(struct0_T *solverParameters){
    solverParameters->maxIterations          = SOLVER_MAX_ITERATIONS;
    solverParameters->maxTime                = SOLVER_MAXTIME;
    solverParameters->enforceJointLimits     = SOLVER_ENFORCE_JOINT_LIMITS;
    solverParameters->allowRandomRestarts    = SOLVER_ALLOW_RANDOM_RESTARTS;
    solverParameters->stepTolerance          = SOLVER_STEP_TOLERANCE;
    solverParameters->positionTolerance      = SOLVER_POSITION_TOLERANCE;
    solverParameters->orientationTolerance   = SOLVER_ORIENTATION_TOLERANCE;
}