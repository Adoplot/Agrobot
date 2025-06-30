#include "tests.h"
#include "ik_wrapper.h"
#include <vector>
#include <array>
#include "Matlab_ik_types.h"
#include "Matlab_ik.h"
#include <iomanip>

using std::cout;
using std::cerr;
using std::endl;

static void test_IK_InterpolatePath();
static void test_IK_getTrajectory();
static void test_Matlab_getSortedCirclePointList();
static void test_IK_getWaypointsForApproach();
static void test_Matlab_getGikCut();

// Put your test code here
void Test_Run(){
    cout << "=============TESTS===============" << endl;
    //test_IK_InterpolatePath();
    //test_IK_getTrajectory();
    //test_Matlab_getSortedCirclePointList();
    //test_IK_getWaypointsForApproach();
    test_Matlab_getGikCut();
    cout << "===========TESTS END=============" << endl;
}


void test_IK_InterpolatePath(){
    std::vector<std::array<double, 6>> pathConfig;
    double q_start[6] {0,0,0,0,0,0};
    double q_end[6] {1,1,1,1,1,1};
    double velocity {1};
    double step_time {0.005};

    bool pathIsValid = IK_InterpolatePath(q_start, q_end, velocity,step_time, pathConfig);

    for (int i=0; i<6; i++) { cout << pathConfig[0][i] << " ";}
    cout << endl;
    for (int i=0; i<6; i++) { cout << pathConfig[1][i] << " ";}
    cout << endl;
    for (int i=0; i<6; i++) { cout << pathConfig[2][i] << " ";}
    cout << endl;
    for (int i=0; i<6; i++) { cout << pathConfig[pathConfig.size()-3][i] << " ";}
    cout << endl;
    for (int i=0; i<6; i++) { cout << pathConfig[pathConfig.size()-2][i] << " ";}
    cout << endl;
    for (int i=0; i<6; i++) { cout << pathConfig[pathConfig.size()-1][i] << " ";}
    cout << endl;

    cout << "pathIsValid = " << pathIsValid << endl;
    cout << "pathConfig.size() = " << pathConfig.size() << endl;
}


void test_IK_getTrajectory(){
    double currentConfig[6] {0,0,0,0,0,0};
    double waypoint[6] {1,1,1,1,1,1};
    double velocity {1};
    double step_time {0.005};
    std::vector<std::array<double, 6>> pathCartesian;


    bool isValid = IK_getTrajectory(currentConfig, waypoint, velocity, step_time, pathCartesian);

    for (int i=0; i<6; i++) { cout << pathCartesian[0][i] << " ";}
    cout << endl;
    for (int i=0; i<6; i++) { cout << pathCartesian[1][i] << " ";}
    cout << endl;
    for (int i=0; i<6; i++) { cout << pathCartesian[2][i] << " ";}
    cout << endl;
    for (int i=0; i<6; i++) { cout << pathCartesian[pathCartesian.size()-3][i] << " ";}
    cout << endl;
    for (int i=0; i<6; i++) { cout << pathCartesian[pathCartesian.size()-2][i] << " ";}
    cout << endl;
    for (int i=0; i<6; i++) { cout << pathCartesian[pathCartesian.size()-1][i] << " ";}
    cout << endl;

    cout << "isValid = " << isValid << endl;
    cout << "pathConfig.size() = " << pathCartesian.size() << endl;
}


void test_Matlab_getSortedCirclePointList(){
    double branchStart[3] {1.2, -0.4, 0.4};
    double branchDir[3] {1.2, 0.8, 0.8};
    double coords_bodyEE_worldFrame[6] {0.5694, -0.0013, 0.4871, 2.3865, 1.4531, 2.3318};

    coder::array<double, 2U> sortedList;
    double listLength {0};

    Matlab_getSortedCirclePointList(CIRCLE_RADIUS, branchStart, branchDir, CIRCLE_POINT_NUM, coords_bodyEE_worldFrame,
                                    CAM_ANGLE_OFFSET, DIST_WEIGHT_POS, DIST_WEIGHT_ORI, sortedList, &listLength);

    IK_PrintSortedPointList(&sortedList);
    cout << endl;
    cout << listLength << endl;

    /*
1.0000	-0.4000	0.4000	0.5662	0.0919	0.8086	0.1312	0.7696
1.0382	-0.3628	0.2885	0.7883	0.1279	0.5940	0.0964	0.8455
1.0382	-0.4372	0.5115	0.2886	0.0468	0.9440	0.1532	1.0985
1.1382	-0.3398	0.2195	0.9333	0.1515	0.3214	0.0522	1.2269
1.1382	-0.4602	0.5805	0.0172	0.0028	-0.9869	-0.1602	1.5014
1.2618	-0.3398	0.2195	0.9869	0.1602	0.0172	0.0028	1.6366
1.2618	-0.4602	0.5805	0.3214	0.0522	-0.9333	-0.1515	1.9110
1.3618	-0.3628	0.2885	0.9440	0.1532	-0.2886	-0.0468	2.0248
1.0000	-0.4000	0.4000	0.0919	-0.5662	0.1312	-0.8086	2.0502
1.0382	-0.4372	0.5115	0.1279	-0.7883	0.0964	-0.5940	2.0905
1.0382	-0.3628	0.2885	0.0468	-0.2886	0.1532	-0.9440	2.0996
1.1382	-0.4602	0.5805	0.1515	-0.9333	0.0522	-0.3214	2.1915
1.1382	-0.3398	0.2195	0.0028	-0.0172	-0.1602	0.9869	2.2167
1.3618	-0.4372	0.5115	0.5940	0.0964	-0.7883	-0.1279	2.2913
1.2618	-0.4602	0.5805	0.1602	-0.9869	0.0028	-0.0172	2.3069
1.2618	-0.3398	0.2195	0.0522	-0.3214	-0.1515	0.9333	2.3527
1.4000	-0.4000	0.4000	0.8086	0.1312	-0.5662	-0.0919	2.3685
1.3618	-0.4372	0.5115	0.1532	-0.9440	-0.0468	0.2886	2.4017
1.4000	-0.4000	0.4000	0.1312	-0.8086	-0.0919	0.5662	2.4556
1.3618	-0.3628	0.2885	0.0964	-0.5940	-0.1279	0.7883	2.4606
     */
}


void test_IK_getWaypointsForApproach(){
    double branchStart[3] {1.2, -0.4, 0.4};
    double branchDir[3] {1.2, 0.8, 0.8};
    Hyundai_Data_t eeCoords_worldFrame {};
    eeCoords_worldFrame.coord[0] = 0.5694;
    eeCoords_worldFrame.coord[1] = -0.0013;
    eeCoords_worldFrame.coord[2] = 0.4871;
    eeCoords_worldFrame.coord[3] = 2.3865;
    eeCoords_worldFrame.coord[4] = 1.4531;
    eeCoords_worldFrame.coord[5] = 2.3318;
    double currentConfig[6] {Transform_Deg2Rad(0.373), Transform_Deg2Rad(66.433), Transform_Deg2Rad(-35.118),
                             Transform_Deg2Rad(4.085), Transform_Deg2Rad(53.841), Transform_Deg2Rad(-2.503)};
    int code {};
    double qWaypoints[18] {};

    IK_getWaypointsForApproach(branchStart, branchDir, &eeCoords_worldFrame,
    currentConfig, &code, qWaypoints);

    IK_PrintWaypoints(qWaypoints);
}


void test_Matlab_getGikCut(){
    double branchStart[3] {1.2, -0.4, 0.4};
    double branchDir[3] {1.2, 0.8, 0.8};
    double exitCode {0};
    struct1_T solutionInfoApr {};
    struct0_T solverParameters {};
    IK_InitSolverParameters(&solverParameters);
    double qWaypointCut[6];
    double currentConfig[6] {-0.5515, 0.6579, 0.3983, -1.2941, 0.5806, -1.3366};

    Matlab_getGikCut(currentConfig,branchStart,&solverParameters, CAM_ANGLE_OFFSET, &exitCode,&solutionInfoApr,qWaypointCut);

    cout << "qWaypointCut: ";
    cout << std::fixed;
    cout << std::setprecision(4);
    for (int i=0;i<6;i++){
        cout << qWaypointCut[i] << "  ";
    }
    cout << endl;

}