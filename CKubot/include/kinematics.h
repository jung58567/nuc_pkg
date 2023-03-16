#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include "ros/ros.h"
#include "eigen3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

#define PI      M_PI
#define R2D     180./PI
#define D2R     PI/180.

class Kinematics {
public:
    int sign(double a);
    Eigen::Matrix3d transposeMat(Matrix3d mat);

    MatrixXd rotMatX(double q);
    MatrixXd rotMatY(double q);
    MatrixXd rotMatZ(double q);

    MatrixXd getTransformI0();
    MatrixXd getTransform6E();
    MatrixXd jointToTransform01(VectorXd q);
    MatrixXd jointToTransform01_R(VectorXd q);
    MatrixXd jointToTransform12(VectorXd q);
    MatrixXd jointToTransform23(VectorXd q);
    MatrixXd jointToTransform34(VectorXd q);
    MatrixXd jointToTransform45(VectorXd q);
    MatrixXd jointToTransform56(VectorXd q);

    MatrixXd jointToPosition(VectorXd q);
    MatrixXd jointToPosition_R(VectorXd q);
    Matrix3d jointToRotMat(VectorXd q);
    Matrix3d jointToRotMat_R(VectorXd q);
    VectorXd rotToEuler(MatrixXd rotMat);

    MatrixXd jointToPosJac(VectorXd q);
    MatrixXd jointToPosJac_R(VectorXd q);
    MatrixXd jointToRotJac(VectorXd q);
    MatrixXd jointToRotJac_R(VectorXd q);
    
    MatrixXd pseudoInverseMat(MatrixXd A, double lambda);
    VectorXd rotMatToRotVec(MatrixXd C);
    MatrixXd angleAxisToRotMat(VectorXd rotVec);
    MatrixXd jointToGeoJac(VectorXd q);
    MatrixXd jointToGeoJac_R(VectorXd q);
    VectorXd inverseKinematics(Vector3d r_des, Matrix3d C_des, VectorXd q0, double tol);
    VectorXd inverseKinematics_R(Vector3d r_des, Matrix3d C_des, VectorXd q0, double tol);
    VectorXd Geometric_IK_L(VectorXd GB_cfg, VectorXd GF_cfg);
    VectorXd Geometric_IK_R(VectorXd GB_cfg, VectorXd GF_cfg);

    VectorXd init_pose();
};

#endif