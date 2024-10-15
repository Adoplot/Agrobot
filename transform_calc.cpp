#include <iostream>
#include <Dense>
#include <cmath>
#include <iomanip>

#include "transform_calc.h"

#include <fstream>

#include "connection_handler.h"

using Eigen::Quaterniond;
using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::AngleAxisd;
using Eigen::Vector3d;
using std::cout;
using std::cerr;
using std::endl;

static Quaterniond quat_conjugate(const Quaterniond q);
static Quaterniond convertEuler2Quat(const double rotZ, const double rotY, const double rotX);
static Cartesian_Pos_t convertQuat2Euler(Quaterniond q);


// Converts target frame to world frame, taking into account scissors length
// Returns: Cartesian coordinates of the target in world frame
Cartesian_Pos_t Transform_ConvertFrameTarget2World(const Cartesian_Pos_t* targetPos_camFrame,
                                                   const Hyundai_Data_t* eePos_worldFrame,
                                                   double scissors_length) {
    Cartesian_Pos_t targetPos_worldFrame{};

    Matrix4d H1;        //  ee   in world frame
    Matrix4d H2;        //  cam  in ee frame
    Matrix4d H3;        //target in cam   frame
    Matrix4d H;         //target in world frame
    Matrix3d H1_rot;    // H1 rotation matrix
    Vector3d H1_tran;   // H1 translation vector
    Matrix3d H3_rot;    // H3 rotation matrix
    Vector3d H3_tran;   // H3 translation vector

    // H1: world -> ee
    // Creating homogenous transformation matrix H1 from received Hyundai data using ZYX convention
    H1_tran << eePos_worldFrame->coord[0], eePos_worldFrame->coord[1], eePos_worldFrame->coord[2];
    H1_rot = AngleAxisd(eePos_worldFrame->coord[5], Vector3d::UnitZ())
             * AngleAxisd(eePos_worldFrame->coord[4], Vector3d::UnitY())
             * AngleAxisd(eePos_worldFrame->coord[3], Vector3d::UnitX());
    H1.setIdentity();
    H1.block<3,3>(0,0) = H1_rot;
    H1.block<3,1>(0,3) = H1_tran;

    // ee -> cam  (ZYX notation z=-90 deg)
    H2 <<   0, 1, 0, CAMERA_POS_X,    //-0.04
           -1, 0, 0, CAMERA_POS_Y,    //0.033
            0, 0, 1, -scissors_length,
            0, 0, 0, 1;

    // H3: cam -> target
    // Creating homogenous transformation matrix H3 from received Compv data
    // using ZYX convention
    H3_tran << targetPos_camFrame->x, targetPos_camFrame->y, targetPos_camFrame->z;
    H3_rot = AngleAxisd(targetPos_camFrame->rotz, Vector3d::UnitZ())
             * AngleAxisd(targetPos_camFrame->roty, Vector3d::UnitY())
             * AngleAxisd(targetPos_camFrame->rotx, Vector3d::UnitX());
    H3.setIdentity();
    H3.block<3,3>(0,0) = H3_rot;
    H3.block<3,1>(0,3) = H3_tran;

    H = H1*H2*H3;

#ifdef DEBUG_H_MATRICES
    //Print H values to log file
    std::ofstream fs("/home/adoplot/ClionProjects/AgrobotHH7/log.txt");

    if(!fs)
    {
        std::cerr<<"Cannot open the output file."<<std::endl;
    }
    else {
        fs << "H1\n" << H1 << std::endl;
        fs << "H2\n" << H2 << std::endl;
        fs << "H3\n" << H3 << std::endl;
        fs << "H1*H2*H3\n" << H << std::endl;
        fs.close();
    }
#endif

    targetPos_worldFrame.rotx = std::atan2(H(2, 1), H(2, 2));
    targetPos_worldFrame.roty = std::atan2(-H(2, 0), std::sqrt(H(2, 1) * H(2, 1) + H(2, 2) * H(2, 2)));
    targetPos_worldFrame.rotz = std::atan2(H(1, 0), H(0, 0));
    targetPos_worldFrame.x = H(0,3);
    targetPos_worldFrame.y = H(1,3);
    targetPos_worldFrame.z = H(2,3);

    return targetPos_worldFrame;
}


// Converts camera frame to world frame, taking into account scissors length
// Returns: Cartesian coordinates of the camera in world frame
Cartesian_Pos_t convertFrameCam2World(const Hyundai_Data_t* eePos_worldFrame,
                                                   double scissors_length) {
    Cartesian_Pos_t camPos_worldFrame{};

    Matrix4d H1;        //  ee   in world  frame
    Matrix4d H2;        //  cam  in ee     frame
    Matrix4d H;         //target in world frame
    Matrix3d H1_rot;    // H1 rotation matrix
    Vector3d H1_tran;   // H1 translation vector

    // H1: world -> ee
    // Creating homogenous transformation matrix H1 from received Hyundai data
    // using ZYX convention
    H1_tran << eePos_worldFrame->coord[0], eePos_worldFrame->coord[1], eePos_worldFrame->coord[2];
    H1_rot = AngleAxisd(eePos_worldFrame->coord[5], Vector3d::UnitZ())
             * AngleAxisd(eePos_worldFrame->coord[4], Vector3d::UnitY())
             * AngleAxisd(eePos_worldFrame->coord[3], Vector3d::UnitX());
    H1.setIdentity();
    H1.block<3,3>(0,0) = H1_rot;
    H1.block<3,1>(0,3) = H1_tran;

    // ee -> cam  (ZYX notation z=-90 deg)
    H2 <<   0, 1, 0, CAMERA_POS_X,    //-0.04
           -1, 0, 0, CAMERA_POS_Y,    //0.033
            0, 0, 1, -scissors_length,
            0, 0, 0, 1;

    H = H1*H2;

#ifdef DEBUG_H_MATRICES
    //Print H values to log file
    std::ofstream fs("/home/adoplot/ClionProjects/AgrobotHH7/log.txt");

    if(!fs)
    {
        std::cerr<<"Cannot open the output file."<<std::endl;
    }
    else {
        fs << "H1\n" << H1 << std::endl;
        fs << "H2\n" << H2 << std::endl;
        fs << "H1*H2\n" << H << std::endl;
        fs.close();
    }
#endif

    camPos_worldFrame.rotx = std::atan2(H(2, 1), H(2, 2));
    camPos_worldFrame.roty = std::atan2(-H(2, 0), std::sqrt(H(2, 1) * H(2, 1) + H(2, 2) * H(2, 2)));
    camPos_worldFrame.rotz = std::atan2(H(1, 0), H(0, 0));
    camPos_worldFrame.x = H(0,3);
    camPos_worldFrame.y = H(1,3);
    camPos_worldFrame.z = H(2,3);

    return camPos_worldFrame;
}


// Calculates position increments to get from current position to the target pos. Uses lerp().
// Returns: increment of Euler angles
Cartesian_Pos_t Transform_CalculatePositionIncrements(const Hyundai_Data_t *eePos_worldFrame,
                                                      const Cartesian_Pos_t *targetPos_worldFrame) {
    Cartesian_Pos_t increments{};

    double vinX = eePos_worldFrame->coord[0];
    double vinY = eePos_worldFrame->coord[1];
    double vinZ = eePos_worldFrame->coord[2];

    double voutX = targetPos_worldFrame->x;
    double voutY = targetPos_worldFrame->y;
    double voutZ = targetPos_worldFrame->z;

    // Linear interpolation between eePos_worldFrame and targetPos_worldFrame
    double vresX = std::lerp(vinX, voutX, LERP_INTERP_FACTOR);
    double vresY = std::lerp(vinY, voutY, LERP_INTERP_FACTOR);
    double vresZ = std::lerp(vinZ, voutZ, LERP_INTERP_FACTOR);

    // Calculate increments
    double vincrX = vresX - vinX;
    double vincrY = vresY - vinY;
    double vincrZ = vresZ - vinZ;

#ifdef DEBUG_LERP
    std::cout << " vincrX = \t" << vincrX;
    std::cout << " vincrY = \t" << vincrY;
    std::cout << " vincrZ = \t" << vincrZ << std::endl;
    std::cout << " vresX = \t" << vresX;
    std::cout << " vresY = \t" << vresY;
    std::cout << " vresZ = \t" << vresZ << std::endl;
#endif

    increments.x = vincrX;
    increments.y = vincrY;
    increments.z = vincrZ;

    return increments;
}


// Calculates orientation increments to get from current orientation to the target ori. Uses slerp() with quaternions.
// Returns: increment of Euler angles
Cartesian_Pos_t Transform_CalculateOrientationIncrements(const Hyundai_Data_t *eePos_worldFrame,
                                                         const Cartesian_Pos_t *targetPos_worldFrame) {
    Cartesian_Pos_t increments{};
    Quaterniond qin, qout, qres, qincr;

    // Convert Euler angles to Quaternions for further interpolation
    Cartesian_Pos_t camPos_worldFrame = convertFrameCam2World(eePos_worldFrame,SCISSORS_LENGTH);

    // Convert Euler angles to start and finish quaternions
    qin = convertEuler2Quat(camPos_worldFrame.rotz, camPos_worldFrame.roty, camPos_worldFrame.rotx);
    qout = convertEuler2Quat(targetPos_worldFrame->rotz, targetPos_worldFrame->roty,
                              targetPos_worldFrame->rotx);

    // Check if quat chose the shortest path. If not, change quat to shortest.
    if (qin.dot(qout) < 0.0) {
        qout = quat_conjugate(qout);
        qout.normalize();
    }

    qres = qin.slerp(SLERP_INTERP_FACTOR, qout);    //slerp from current to target rotation
    qres.normalize();           //to insure quat is unit quat, as it can drift occasionally

    qincr = qres * qin.inverse();   // calculate difference between qin and qres (increment)
    qincr.normalize();

    Cartesian_Pos_t eul_incr = convertQuat2Euler(qincr);   //convert increment quat to inc euler

#ifdef DEBUG_QUATERNIONS
    //std::cout << "targ RotX = " << recvRobotDataTCPframed->rotX << " \ttarg RotY = " << recvRobotDataTCPframed->rotY << "\ttarg RotZ = " << recvRobotDataTCPframed->rotZ << std::endl;
    //std::cout << "sent RotX = " << euler_to_send.rotX << " \tsent RotY = " << euler_to_send.rotY << "\tsent RotZ = " << euler_to_send.rotZ << std::endl;
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "qin=  \t" << qin << "\n";
    std::cout << "qres= \t" << qres << "\n";
    std::cout << "qincr= \t" << qincr << "\n";
    std::cout << "qout= \t" << qout << "\n";
    std::cout << "----------------------------------------\n";
#endif

    // Write increment Euler values to send
    increments.rotx = eul_incr.rotx;
    increments.roty = eul_incr.roty;
    increments.rotz = eul_incr.rotz;

    return increments;
}


// Conjugates quaternion
// Returns: conjugate of a quaternion
Quaterniond quat_conjugate(const Quaterniond q){
    double x = -q.x();
    double y = -q.y();
    double z = -q.z();
    double w = -q.w();
    Quaterniond q_conj(w,x,y,z);
    return q_conj;
}


// Converts Euler ZYX to quaternion, angles should be in RADIANS
// Returns: quaternion
Quaterniond convertEuler2Quat(const double rotZ, const double rotY, const double rotX) {
    Matrix3d m;
    Quaterniond q;
    m =     AngleAxisd(rotZ, Vector3d::UnitZ())
            * AngleAxisd(rotY, Vector3d::UnitY())
            * AngleAxisd(rotX, Vector3d::UnitX());
    q = m;
    q.normalize();
    return q;
}


// Converts quaternion Euler ZYX, output angles are in RADIANS
// Returns: Euler angles
Cartesian_Pos_t convertQuat2Euler(Quaterniond q) {
    Cartesian_Pos_t eul;
    Matrix3d mat;
    //q.normalize();
    mat = q.toRotationMatrix();
    eul.rotx = std::atan2(mat(2, 1), mat(2, 2));
    eul.roty = std::atan2(-mat(2, 0), std::sqrt(mat(2, 1) * mat(2, 1) + mat(2, 2) * mat(2, 2)));
    eul.rotz = std::atan2(mat(1, 0), mat(0, 0));

    return eul;
}


// Calculates distance between two points in 3D space
// Returns: distance in meters
double Transform_CalcDistanceBetweenPoints(const Hyundai_Data_t *eePos_worldFrame, const Cartesian_Pos_t *targetPos_worldFrame){
    double x1,x2,y1,y2,z1,z2;
    x1 = eePos_worldFrame->coord[0];
    y1 = eePos_worldFrame->coord[1];
    z1 = eePos_worldFrame->coord[2];
    x2 = targetPos_worldFrame->x;
    y2 = targetPos_worldFrame->y;
    z2 = targetPos_worldFrame->z;
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
}


// Compares two orientations in quaternion form with a defined precision
// Returns: true - orientations are the same, false - not the same
bool Transform_CompareOrientations(double precision,
                                    const Hyundai_Data_t *eePos_worldFrame,
                                    const Cartesian_Pos_t *targetPos_worldFrame) {

    Cartesian_Pos_t camPos_worldFrame = convertFrameCam2World(eePos_worldFrame,SCISSORS_LENGTH);
    Quaterniond q1 = convertEuler2Quat(camPos_worldFrame.rotz, camPos_worldFrame.roty, camPos_worldFrame.rotx);
    Quaterniond q2 = convertEuler2Quat(targetPos_worldFrame->rotz, targetPos_worldFrame->roty,
                                                                    targetPos_worldFrame->rotx);
    double w1 = q1.w();
    double x1 = q1.x();
    double y1 = q1.y();
    double z1 = q1.z();
    double w2 = q2.w();
    double x2 = q2.x();
    double y2 = q2.y();
    double z2 = q2.z();

    // Compare q1 with q2
    bool directComparison = (std::fabs(w1 - w2) < precision) &&
                            (std::fabs(x1 - x2) < precision) &&
                            (std::fabs(y1 - y2) < precision) &&
                            (std::fabs(z1 - z2) < precision);
    // Compare q1 with -q2
    bool oppositeComparison = (std::fabs(w1 + w2) < precision) &&
                              (std::fabs(x1 + x2) < precision) &&
                              (std::fabs(y1 + y2) < precision) &&
                              (std::fabs(z1 + z2) < precision);

    return directComparison || oppositeComparison;
}