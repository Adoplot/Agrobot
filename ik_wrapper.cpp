#include "Matlab_ik_types.h"
#include "Matlab_ik.h"
#include "connection_handler.h"
#include "robot_api.h"
#include <netinet/in.h>
#include <iomanip>
#include "ik_wrapper.h"

//TODO: Adoplot - add cartesian constraint for both GIKs (to not collide with robot's platform)
//TODO: Adoplot - make path generation with controlled speed (variable-length path vector)

/*
 * Calculates Cartesian trajectory from one config to another, based on speed and number of steps
 * Input:
 * currentConfig    - 1x6 array, start configuration of the robot (in radians)
 * waypoint         - 1x6 array, final configuration of the robot (in radians)
 * Output:
 * pathCartesian    - Nx6 array, trajectory of the robot end-effector, where every row is a Cartesian coordinate [xyzABC]
 * true             - bool, trajectory is valid
 * false            - bool, trajectory is NOT valid (axis limit violation or self-collision detected)
 */
bool IK_getTrajectory(const double currentConfig[6], const double waypoint[6], const double velocity,
                      const double step_time, std::vector<std::array<double, 6>> &pathCartesian){
    // Clear previous path
    pathCartesian.clear();

    std::vector<std::array<double, 6>> pathConfig;
    std::array<double, 6> coords;
    double config[6] {0};
    double robotSE3[16];
    double pos[3];
    double ori[3];
    bool isValid {true};

    isValid = IK_InterpolatePath(currentConfig, waypoint, velocity, step_time, pathConfig);

    if (!isValid){
        return false;
    } else {
        for (int i=0; i < pathConfig.size()-1; i++) {
            for (int j=0; j<6; j++) {
                config[j] = pathConfig[i][j];
            }
            Matlab_getForwardKinematics(config, ROBOTEE, robotSE3, pos, ori);

            for (int k = 0; k < 3; k++){
                coords[k] = pos[k];
                coords[k+3] = ori[k];
            }
            pathCartesian.push_back(coords);
        }

        return true;
    }
}


/*
 * Interpolates between two robot configurations in configuration-space. Checks for axis limit violations and robot self
 * collisions.
 * Inputs:
 *
 * Outputs:
 * pathConfig   - Nx6 array, robot configurations representing trajectory in configuration-space
 * true         - bool, trajectory is valid, no axis limit violations and no self-collisions
 * false        - bool, NOT valid trajectory, caused by invalid arguments, axis limit violation or self-collision
 *
 */
bool IK_InterpolatePath(const double q_start[6], const double q_end[6], const double velocity,
                        const double step_time, std::vector<std::array<double,6>> &pathConfig){

    if (velocity <= 0) {
        std::cerr << "IK_InterpolatePath: velocity should be > 0" << std::endl;
        return false;
    }
    if (step_time <= 0) {
        std::cerr << "IK_InterpolatePath: step_time should be > 0" << std::endl;
        return false;
    }

    pathConfig.clear();
    bool axisLimitsOK {true};   //default is true, if any axis is out of limits at any point - value changes to false
    bool noCollision {true};    //default is true, if robot at any point is in collision - value changes to false
    double config[6] {0};
    bool isSelfColliding {false};
    double collisionPairs[2] {0};

    double delta[6] {0};
    double distance {0};
    for (int i = 0; i < 6; i++) {
        delta[i] = q_end[i] - q_start[i];
        distance += delta[i] * delta[i];  // Sum of squares
    }
    distance = sqrt(distance);

    double total_time = distance / velocity;
    int totalSteps = std::max(2, static_cast<int>(std::round(total_time / step_time)));

    // Iterate through all steps and compute interpolation with axis limit and collision checks
    for (int i = 0; i < totalSteps; i++) {
        double alpha = static_cast<double>(i) / (totalSteps - 1);
        std::array<double, 6> waypoint{};
        for (int j = 0; j < 6; j++) {
            waypoint[j] = q_start[j] + alpha * delta[j];
            config[j] = q_start[j] + alpha * delta[j];      // need double[6] for codegen checkCollision()

            //Check axis limits
            if (! IK_AxisInLimits(waypoint[j], j)){
                axisLimitsOK = false;
                std::cerr << "IK_InterpolatePath: axis out of limits " << std::endl;
                std::cerr << waypoint[j] << " " << j+1 << std::endl;
            }
        }

        //Check collision for each configuration
        Matlab_checkCollision(config, &isSelfColliding,collisionPairs);
        if (isSelfColliding){
            noCollision = false;
        }

        pathConfig.push_back(waypoint);

    }

    // This is considered cerr error, because Matlab's GIK solver should have checked axisLimits and self-collisions when computing waypoints
    if (!axisLimitsOK){
        std::cout << "IK_InterpolatePath: axis limit violation detected" << std::endl;
        std::cerr << "IK_InterpolatePath: axis limit violation detected" << std::endl;
    }
    if (!noCollision){
        std::cout << "IK_InterpolatePath: self-collision detected" << std::endl;
        std::cerr << "IK_InterpolatePath: self-collision detected" << std::endl;
    }

    // If no axis limit violations AND no collisions - return true
    if (axisLimitsOK && noCollision){
        return true;
    } else{
        return false;
    }
}


/*
 * Checks if axis is in its limits
 *      Input:
 * axisValue    -   double, axis angle value (in radians)
 * axisNum      -   int, axis number starting from 0 to 5
 *      Output:
 * true     - axis is in its limits
 * false    - axis exceeded its limits / fault, axisNum is out of bounds (0-5)
 */
bool IK_AxisInLimits(const double axisValue, const int axisNum){
    // Check if axisNum is in bounds (0-5)
    if ((axisNum > 5) || (axisNum < 0)){
        std::cerr << "IK_AxisInLimits(): axisNum is out of bounds! axisNum = " << axisNum+1 << std::endl;
        return false;
    }
    const double minLimits[6] = {Transform_Deg2Rad(AXIS_MIN_A1), Transform_Deg2Rad(AXIS_MIN_A2), Transform_Deg2Rad(AXIS_MIN_A3),
                                 Transform_Deg2Rad(AXIS_MIN_A4), Transform_Deg2Rad(AXIS_MIN_A5), Transform_Deg2Rad(AXIS_MIN_A6)};
    const double maxLimits[6] = {Transform_Deg2Rad(AXIS_MAX_A1), Transform_Deg2Rad(AXIS_MAX_A2), Transform_Deg2Rad(AXIS_MAX_A3),
                                 Transform_Deg2Rad(AXIS_MAX_A4), Transform_Deg2Rad(AXIS_MAX_A5), Transform_Deg2Rad(AXIS_MAX_A6)};
    if (axisValue < minLimits[axisNum] || axisValue > maxLimits[axisNum]){
        return false;
    }
    else{
        return true;
    }
}


/*
 * Calculates waypoints for Approach sequence.
 * Calculates circle around the cutplace, calculates sorted list of points for Approach sequence, chooses valid
 * waypoints for Approach and FinalApr sequence.
 *      Input:
 * branchStart          - 1x3 array, cutplace position in world frame [xyz]
 * branchDir            - 1x3 array, branch direction vector in world frame [xyz]
 * eeCoords_worldFrame  - struct, robotEE coords in world frame from Hyundai controller
 * currentConfig        - 1x6 array, current configuration of the robot from Hyundai controller [A1-A6]
 *      Output:
 * code         - int, exit code: 0 - Apr fail, 1 - Success, 2 - FinalApr fail
 * qWaypoints   - 3x6 array with 3 robot configurations: 1row - current, 2 - Approach, 3 - FinalApr. (in radians)
 */
void IK_getWaypointsForApproach(const double branchStart[3], const double branchDir[3], const Hyundai_Data_t *eeCoords_worldFrame,
                                const double currentConfig[6], int *code, double qWaypoints[18]) {

    coder::array<double, 2U> sortedList;
    double listLength {0};  //redundant
    double coords_bodyEE_worldFrame[6];

    // EE -> toolEE both in world frame
    coords_bodyEE_worldFrame[0] = eeCoords_worldFrame->coord[0];
    coords_bodyEE_worldFrame[1] = eeCoords_worldFrame->coord[1];
    coords_bodyEE_worldFrame[2] = eeCoords_worldFrame->coord[2];
    coords_bodyEE_worldFrame[3] = eeCoords_worldFrame->coord[3];
    coords_bodyEE_worldFrame[4] = eeCoords_worldFrame->coord[4];
    coords_bodyEE_worldFrame[5] = eeCoords_worldFrame->coord[5];


    //Get sortedList with points on a circle around the cutting place
    Matlab_getSortedCirclePointList(CIRCLE_RADIUS, branchStart, branchDir, CIRCLE_POINT_NUM, coords_bodyEE_worldFrame,
                                    CAM_ANGLE_OFFSET, DIST_WEIGHT_POS, DIST_WEIGHT_ORI, sortedList, &listLength);

    //Todo: comment out if not debugging
    IK_PrintSortedPointList(&sortedList);

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
        Matlab_getGikFull(currentConfig, targetApr, branchStart, &solverParameters, CAM_ANGLE_OFFSET, &exitCode,&solutionInfoApr,qWaypoints);
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


// Prints to console values in sortedList.
void IK_PrintSortedPointList(coder::array<double, 2U> *sortedList){
    const int rows = sortedList->size(0);
    const int cols = sortedList->size(1);
    std::cout << std::fixed;
    std::cout << std::setprecision(4);

    for (int i=0; i< rows; i++){
        for (int j=0; j< cols; j++){
            std::cout << sortedList->at(i,j) << "\t";
        }
        std::cout << std::endl;
    }
}


// Initializes GIK solver parameter values
void IK_InitSolverParameters(struct0_T *solverParameters){
    solverParameters->maxIterations          = SOLVER_MAX_ITERATIONS;
    solverParameters->maxTime                = SOLVER_MAXTIME;
    solverParameters->enforceJointLimits     = SOLVER_ENFORCE_JOINT_LIMITS;
    solverParameters->allowRandomRestarts    = SOLVER_ALLOW_RANDOM_RESTARTS;
    solverParameters->stepTolerance          = SOLVER_STEP_TOLERANCE;
    solverParameters->positionTolerance      = SOLVER_POSITION_TOLERANCE;
    solverParameters->orientationTolerance   = SOLVER_ORIENTATION_TOLERANCE;
}