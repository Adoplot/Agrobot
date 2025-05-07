#include "tests.h"
#include "ik_wrapper.h"
#include <vector>
#include <array>

using std::cout;
using std::cerr;
using std::endl;

static void test_IK_InterpolatePath();
static void test_IK_getTrajectory();
static void test_Transform_getIncrements();

// Put your test code here
void Test_Run(){
    cout << "=============TESTS===============" << endl;
    test_IK_InterpolatePath();
    test_IK_getTrajectory();
    test_Transform_getIncrements();
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


void test_Transform_getIncrements(){

}