#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "kinematics.h"

class Kinematics KM;

int Kinematics::sign(double a){
    if(a>=0) return 1;
    return -1;
}

Matrix3d Kinematics::transposeMat(Matrix3d mat) {
    Matrix3d tmp_m;

    tmp_m << mat(0, 0), mat(1, 0), mat(2, 0), \
             mat(0, 1), mat(1, 1), mat(2, 1), \
             mat(0, 2), mat(1, 2), mat(2, 2);
 
    return tmp_m;
}

MatrixXd Kinematics::rotMatX(double q) {
    MatrixXd tmp_m(3,3);
    tmp_m << 1, 0, 0,\
             0, cos(q), -sin(q),\
             0, sin(q), cos(q);
    return tmp_m;
}

MatrixXd Kinematics::rotMatY(double q) {
    MatrixXd tmp_m(3,3);
    tmp_m << cos(q), 0, sin(q),\
             0 , 1, 0 ,\
             -sin(q), 0, cos(q);
    return tmp_m;
}

MatrixXd Kinematics::rotMatZ(double q) {
    MatrixXd tmp_m(3,3);
    tmp_m << cos(q), -sin(q), 0,\
             sin(q), cos(q), 0, \
             0, 0, 1;
    return tmp_m;
}

MatrixXd Kinematics::getTransformI0()
{
//  Frame 0 to Frame I

    MatrixXd tmp_m(4,4);
  
// Alternative representations 
// 1. tmp_m = MatrixXd::Identity(4,4);
// 2. tmp_m(m-1,n-1) = m,n element where m,n>=1;

    tmp_m << 1, 0, 0, 0, \
             0, 1, 0, 0, \
             0, 0, 1, 0, \
             0, 0, 0, 1;
    
    return tmp_m;
}

MatrixXd Kinematics::getTransform6E()
{
//  Frame E to Frame 6
    
    MatrixXd tmp_m(4,4);
            
    tmp_m << 1, 0, 0,  0, \
             0, 1, 0,  0, \
             0, 0, 1, -0.037, \
             0, 0, 0,  1;
    
    return tmp_m;
}

MatrixXd Kinematics::jointToTransform01(VectorXd q)
{
//  Frame 1 to Frame 0
// q: generalized coordinates, q = [q1;q2;q3;q4;q5;q6]    

    MatrixXd tmp_m(4,4);
    double qq = q(0);
   
    tmp_m << cos(qq), -sin(qq), 0, 0, \
             sin(qq),  cos(qq), 0, 0.05, \
             0,        0,       1, -0.08, \
             0,        0,       0, 1;
    
    return tmp_m;    
}

MatrixXd Kinematics::jointToTransform01_R(VectorXd q)
{
//  Frame 1 to Frame 0
// q: generalized coordinates, q = [q1;q2;q3;q4;q5;q6]    

    MatrixXd tmp_m(4,4);
    double qq = q(0);
   
    tmp_m << cos(qq), -sin(qq), 0, 0, \
             sin(qq),  cos(qq), 0, -0.05, \
             0,        0,       1, -0.08, \
             0,        0,       0, 1;
    
    return tmp_m;    
}

MatrixXd Kinematics::jointToTransform12(VectorXd q)
{
//  Frame 2 to Frame 1

    MatrixXd tmp_m(4,4);
    double qq = q(1);

    tmp_m << 1, 0,        0,       0, \
             0, cos(qq), -sin(qq), 0, \
             0, sin(qq),  cos(qq), 0, \
             0      , 0,  0      , 1;
    
    return tmp_m;
}

MatrixXd Kinematics::jointToTransform23(VectorXd q)
{
//  Frame 3 to Frame 2
    MatrixXd tmp_m(4,4);
    double qq = q(2);

    tmp_m << cos(qq), 0, sin(qq), 0, \
             0      , 1, 0      , 0, \
            -sin(qq), 0, cos(qq), 0, \
             0      , 0, 0      , 1;
    
    return tmp_m;
}

MatrixXd Kinematics::jointToTransform34(VectorXd q)
{
//  Frame 4 to Frame 3
    MatrixXd tmp_m(4,4);
    double qq = q(3);
    
    tmp_m << cos(qq), 0, sin(qq), 0, \
             0      , 1, 0      , 0, \
            -sin(qq), 0, cos(qq), -0.133, \
             0      , 0, 0      , 1;
    
    return tmp_m;
}

MatrixXd Kinematics::jointToTransform45(VectorXd q)
{
//  Frame 5 to Frame 4
    MatrixXd tmp_m(4,4);
    double qq = q(4);

    tmp_m << cos(qq), 0, sin(qq), 0, \
             0      , 1, 0      , 0, \
            -sin(qq), 0, cos(qq), -0.138, \
             0      , 0, 0      , 1;
    
    return tmp_m;
}

MatrixXd Kinematics::jointToTransform56(VectorXd q)
{
//  Frame 5 to Frame 4
    MatrixXd tmp_m(4,4);
    double qq = q(5);

    tmp_m << 1, 0,        0,       0, \
             0, cos(qq), -sin(qq), 0, \
             0, sin(qq),  cos(qq), 0, \
             0      , 0,  0      , 1;

    return tmp_m;
}

MatrixXd Kinematics::jointToPosition(VectorXd q)
{
//  Extract position vector(3x1) 
    
    VectorXd tmp_v = VectorXd::Zero(3);
    MatrixXd tmp_m(4,4);
    
    tmp_m = getTransformI0()*
            jointToTransform01(q)* 
            jointToTransform12(q)* 
            jointToTransform23(q)* 
            jointToTransform34(q)* 
            jointToTransform45(q)* 
            jointToTransform56(q)* 
            getTransform6E();
        
//    tmp_v = tmp_m.block(0,3,3,1);

    tmp_v(0) = tmp_m(0,3);
    tmp_v(1) = tmp_m(1,3);
    tmp_v(2) = tmp_m(2,3);
    
    return tmp_v;
}

MatrixXd Kinematics::jointToPosition_R(VectorXd q)
{
//  Extract position vector(3x1) 
    
    VectorXd tmp_v = VectorXd::Zero(3);
    MatrixXd tmp_m(4,4);
    
    tmp_m = getTransformI0()*
            jointToTransform01_R(q)* 
            jointToTransform12(q)* 
            jointToTransform23(q)* 
            jointToTransform34(q)* 
            jointToTransform45(q)* 
            jointToTransform56(q)* 
            getTransform6E();
        
//    tmp_v = tmp_m.block(0,3,3,1);

    tmp_v(0) = tmp_m(0,3);
    tmp_v(1) = tmp_m(1,3);
    tmp_v(2) = tmp_m(2,3);
    
    return tmp_v;
}

Matrix3d Kinematics::jointToRotMat(VectorXd q)
{
    Matrix3d tmp_m;
    MatrixXd T_IE(4,4);
    
    T_IE =  getTransformI0()*
            jointToTransform01(q)* 
            jointToTransform12(q)* 
            jointToTransform23(q)* 
            jointToTransform34(q)* 
            jointToTransform45(q)* 
            jointToTransform56(q)* 
            getTransform6E();
    
    tmp_m = T_IE.block(0,0,3,3);
    
    return tmp_m;
}

Matrix3d Kinematics::jointToRotMat_R(VectorXd q)
{
    Matrix3d tmp_m;
    MatrixXd T_IE(4,4);
    
    T_IE =  getTransformI0()*
            jointToTransform01_R(q)* 
            jointToTransform12(q)* 
            jointToTransform23(q)* 
            jointToTransform34(q)* 
            jointToTransform45(q)* 
            jointToTransform56(q)* 
            getTransform6E();
    
    tmp_m = T_IE.block(0,0,3,3);
    
    return tmp_m;
}

VectorXd Kinematics::rotToEuler(MatrixXd rotMat)
{
    // ZYX Euler Angle - yaw-pitch-roll
    Vector3d tmp_v;
    
    tmp_v(0) = atan2(rotMat(1,0),rotMat(0,0));
    tmp_v(1) = atan2(-rotMat(2,0),sqrt(pow(rotMat(2,1),2)+pow(rotMat(2,2),2)));
    tmp_v(2) = atan2(rotMat(2,1),rotMat(2,2));
            
    // std::cout << tmp_v << endl;
    
    return tmp_v;
}

MatrixXd Kinematics::jointToPosJac(VectorXd q)
{
    // Input: vector of generalized coordinates (joint angles)
    // Output: J_P, Jacobian of the end-effector translation which maps joint velocities to end-effector linear velocities in I frame.
    MatrixXd J_P = MatrixXd::Zero(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4), T_IE(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d r_I_I1, r_I_I2, r_I_I3, r_I_I4, r_I_I5, r_I_I6;
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;
    Vector3d n_I_1,n_I_2,n_I_3,n_I_4,n_I_5,n_I_6;
    Vector3d r_I_IE;


    //* Compute the relative homogeneous transformation matrices.
    T_I0 = getTransformI0();
    T_01 = jointToTransform01(q);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();

    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0*T_01;
    T_I2 = T_I1*T_12;
    T_I3 = T_I2*T_23;
    T_I4 = T_I3*T_34;
    T_I5 = T_I4*T_45;
    T_I6 = T_I5*T_56;

    //* Extract the rotation matrices from each homogeneous transformation matrix. Use sub-matrix of EIGEN. https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = T_I2.block(0,0,3,3);
    R_I3 = T_I3.block(0,0,3,3);
    R_I4 = T_I4.block(0,0,3,3);
    R_I5 = T_I5.block(0,0,3,3);
    R_I6 = T_I6.block(0,0,3,3);

    //* Extract the position vectors from each homogeneous transformation matrix. Use sub-matrix of EIGEN.
    r_I_I1 = T_I1.block(0,3,3,1);
    r_I_I2 = T_I2.block(0,3,3,1);
    r_I_I3 = T_I3.block(0,3,3,1);
    r_I_I4 = T_I4.block(0,3,3,1);
    r_I_I5 = T_I5.block(0,3,3,1);
    r_I_I6 = T_I6.block(0,3,3,1);

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;

    //* Compute the unit vectors for the inertial frame I.
    n_I_1 = R_I1*n_1;
    n_I_2 = R_I2*n_2;
    n_I_3 = R_I3*n_3;
    n_I_4 = R_I4*n_4;
    n_I_5 = R_I5*n_5;
    n_I_6 = R_I6*n_6;

    //* Compute the end-effector position vector.
    T_IE = T_I6*T_6E;
    r_I_IE = T_IE.block(0,3,3,1);


    //* Compute the translational Jacobian. Use cross of EIGEN.
    J_P.col(0) << n_I_1.cross(r_I_IE-r_I_I1);
    J_P.col(1) << n_I_2.cross(r_I_IE-r_I_I2);
    J_P.col(2) << n_I_3.cross(r_I_IE-r_I_I3);
    J_P.col(3) << n_I_4.cross(r_I_IE-r_I_I4);
    J_P.col(4) << n_I_5.cross(r_I_IE-r_I_I5);
    J_P.col(5) << n_I_6.cross(r_I_IE-r_I_I6);

    // std::cout << "Test, JP:" << std::endl << J_P << std::endl;

    return J_P;
}

MatrixXd Kinematics::jointToPosJac_R(VectorXd q)
{
    // Input: vector of generalized coordinates (joint angles)
    // Output: J_P, Jacobian of the end-effector translation which maps joint velocities to end-effector linear velocities in I frame.
    MatrixXd J_P = MatrixXd::Zero(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4), T_IE(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d r_I_I1, r_I_I2, r_I_I3, r_I_I4, r_I_I5, r_I_I6;
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;
    Vector3d n_I_1,n_I_2,n_I_3,n_I_4,n_I_5,n_I_6;
    Vector3d r_I_IE;


    //* Compute the relative homogeneous transformation matrices.
    T_I0 = getTransformI0();
    T_01 = jointToTransform01_R(q);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();

    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0*T_01;
    T_I2 = T_I1*T_12;
    T_I3 = T_I2*T_23;
    T_I4 = T_I3*T_34;
    T_I5 = T_I4*T_45;
    T_I6 = T_I5*T_56;

    //* Extract the rotation matrices from each homogeneous transformation matrix. Use sub-matrix of EIGEN. https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = T_I2.block(0,0,3,3);
    R_I3 = T_I3.block(0,0,3,3);
    R_I4 = T_I4.block(0,0,3,3);
    R_I5 = T_I5.block(0,0,3,3);
    R_I6 = T_I6.block(0,0,3,3);

    //* Extract the position vectors from each homogeneous transformation matrix. Use sub-matrix of EIGEN.
    r_I_I1 = T_I1.block(0,3,3,1);
    r_I_I2 = T_I2.block(0,3,3,1);
    r_I_I3 = T_I3.block(0,3,3,1);
    r_I_I4 = T_I4.block(0,3,3,1);
    r_I_I5 = T_I5.block(0,3,3,1);
    r_I_I6 = T_I6.block(0,3,3,1);

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;

    //* Compute the unit vectors for the inertial frame I.
    n_I_1 = R_I1*n_1;
    n_I_2 = R_I2*n_2;
    n_I_3 = R_I3*n_3;
    n_I_4 = R_I4*n_4;
    n_I_5 = R_I5*n_5;
    n_I_6 = R_I6*n_6;

    //* Compute the end-effector position vector.
    T_IE = T_I6*T_6E;
    r_I_IE = T_IE.block(0,3,3,1);


    //* Compute the translational Jacobian. Use cross of EIGEN.
    J_P.col(0) << n_I_1.cross(r_I_IE-r_I_I1);
    J_P.col(1) << n_I_2.cross(r_I_IE-r_I_I2);
    J_P.col(2) << n_I_3.cross(r_I_IE-r_I_I3);
    J_P.col(3) << n_I_4.cross(r_I_IE-r_I_I4);
    J_P.col(4) << n_I_5.cross(r_I_IE-r_I_I5);
    J_P.col(5) << n_I_6.cross(r_I_IE-r_I_I6);

    // std::cout << "Test, JP:" << std::endl << J_P << std::endl;

    return J_P;
}

MatrixXd Kinematics::jointToRotJac(VectorXd q)
{
   // Input: vector of generalized coordinates (joint angles)
    // Output: J_R, Jacobian of the end-effector orientation which maps joint velocities to end-effector angular velocities in I frame.
    MatrixXd J_R(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;

    //* Compute the relative homogeneous transformation matrices.
    T_I0 = getTransformI0();
    T_01 = jointToTransform01(q);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();

    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0*T_01;
    T_I2 = T_I1*T_12;
    T_I3 = T_I2*T_23;
    T_I4 = T_I3*T_34;
    T_I5 = T_I4*T_45;
    T_I6 = T_I5*T_56;

    //* Extract the rotation matrices from each homogeneous transformation matrix.
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = T_I2.block(0,0,3,3);
    R_I3 = T_I3.block(0,0,3,3);
    R_I4 = T_I4.block(0,0,3,3);
    R_I5 = T_I5.block(0,0,3,3);
    R_I6 = T_I6.block(0,0,3,3);

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;

    //* Compute the rotational Jacobian.
    J_R.col(0) << R_I1*n_1;
    J_R.col(1) << R_I2*n_2;
    J_R.col(2) << R_I3*n_3;
    J_R.col(3) << R_I4*n_4;
    J_R.col(4) << R_I5*n_5;
    J_R.col(5) << R_I6*n_6;

    // std::cout << "Test, J_R:" << std::endl << J_R << std::endl;

    return J_R;
}

MatrixXd Kinematics::jointToRotJac_R(VectorXd q)
{
   // Input: vector of generalized coordinates (joint angles)
    // Output: J_R, Jacobian of the end-effector orientation which maps joint velocities to end-effector angular velocities in I frame.
    MatrixXd J_R(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;

    //* Compute the relative homogeneous transformation matrices.
    T_I0 = getTransformI0();
    T_01 = jointToTransform01_R(q);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();

    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0*T_01;
    T_I2 = T_I1*T_12;
    T_I3 = T_I2*T_23;
    T_I4 = T_I3*T_34;
    T_I5 = T_I4*T_45;
    T_I6 = T_I5*T_56;

    //* Extract the rotation matrices from each homogeneous transformation matrix.
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = T_I2.block(0,0,3,3);
    R_I3 = T_I3.block(0,0,3,3);
    R_I4 = T_I4.block(0,0,3,3);
    R_I5 = T_I5.block(0,0,3,3);
    R_I6 = T_I6.block(0,0,3,3);

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;

    //* Compute the rotational Jacobian.
    J_R.col(0) << R_I1*n_1;
    J_R.col(1) << R_I2*n_2;
    J_R.col(2) << R_I3*n_3;
    J_R.col(3) << R_I4*n_4;
    J_R.col(4) << R_I5*n_5;
    J_R.col(5) << R_I6*n_6;

    // std::cout << "Test, J_R:" << std::endl << J_R << std::endl;

    return J_R;
}

MatrixXd Kinematics::pseudoInverseMat(MatrixXd A, double lambda)
{
    // Input: Any m-by-n matrix
    // Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula
    MatrixXd pinvA;
    MatrixXd tmp_m;
    
    int m = A.rows();
    int n = A.cols();
    
    MatrixXd A_T = A.transpose();

    if(m >= n){
        tmp_m = A_T*A + lambda*lambda*(MatrixXd::Identity(n,n));
        pinvA = tmp_m.inverse()*A_T;
        // std::cout << "pinvAl : " << pinvA << std::endl;
    }
    
    else{
        tmp_m = A*A_T + lambda*lambda*(MatrixXd::Identity(m,m));
        pinvA = A_T*tmp_m.inverse();
        // std::cout << "pinvAr : " << pinvA << std::endl;
    }
    
    return pinvA;
}

VectorXd Kinematics::rotMatToRotVec(MatrixXd C)
{
    // Input: a rotation matrix C
    // Output: the rotational vector which describes the rotation C
    VectorXd phi(3), n(3);
    double th;
    
    th = acos((C(0,0)+C(1,1)+C(2,2)-1)/2);
    
    if(fabs(th)<0.001){
         n << 0,0,0;
    }
    else{
        n << C(2,1)-C(1,2), \
             C(0,2)-C(2,0), \
             C(1,0)-C(0,1);
        n = n/(2*sin(th));
    }

    phi = th*n;

    return phi;
}

MatrixXd Kinematics::angleAxisToRotMat(VectorXd rotVec)
{
    VectorXd n(3);
    double nx, ny, nz, s, c, th;
    MatrixXd tmp_m(3,3);
    
    n = rotVec/rotVec.norm();
    th = rotVec.norm();
    
    nx = n(0);
    ny = n(1);
    nz = n(2);
    s = sin(th);
    c = cos(th);
    
    tmp_m <<
        nx*nx*(1.0-c)+c, nx*ny*(1.0-c)-nz*s, nx*nz*(1.0-c)+ny*s, \
        nx*ny*(1.0-c)+nz*s, ny*ny*(1.0-c)+c, ny*nz*(1.0-c)-nx*s, \
        nx*nz*(1.0-c)-ny*s, ny*nz*(1.0-c)+nx*s, nz*nz*(1.0-c)+c;
                   
    return tmp_m;
}

MatrixXd Kinematics::jointToGeoJac(VectorXd q)
{
    MatrixXd tmp_m = MatrixXd::Zero(6,6);
    tmp_m << jointToPosJac(q), jointToRotJac(q);

    return tmp_m;
}

MatrixXd Kinematics::jointToGeoJac_R(VectorXd q)
{
    MatrixXd tmp_m = MatrixXd::Zero(6,6);
    tmp_m << jointToPosJac_R(q), jointToRotJac_R(q);

    return tmp_m;
}

VectorXd Kinematics::inverseKinematics(Vector3d r_des, Matrix3d C_des, VectorXd q0, double tol)
{
    // Input: desired end-effector position, desired end-effector orientation, initial guess for joint angles, threshold for the stopping-criterion
    // Output: joint angles which match desired end-effector position and orientation
    int num_it = 0;
    MatrixXd J_P(3,6), J_R(3,6), J(6,6), pinvJ(6,6);
    Matrix3d C_err, C_IE;
    VectorXd q(6),dq(6),dXe(6);
    Vector3d dr, dph;
    double lambda;
    
    //* Set maximum number of iterations
    double max_it = 200;
    
    //* Initialize the solution with the initial guess
    q = q0;
    C_IE = jointToRotMat(q);
    C_err = C_des*transposeMat(C_IE);
    //* Damping factor
    lambda = 0.001;
    
    //* Initialize error
    dr = r_des - jointToPosition(q);
    dph = rotMatToRotVec(C_err);
    dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
    
    //////////////////////////////////////////////
    //** Iterative inverse kinematics
    //////////////////////////////////////////////

    //* Iterate until terminating condition
    while (num_it<max_it && dXe.norm()>tol)
    {
        //Compute Inverse Jacobian
        J_P = jointToPosJac(q);
        J_R = jointToRotJac(q);

        J.block(0,0,3,6) = J_P;
        J.block(3,0,3,6) = J_R; // Geometric Jacobian
        
        // Convert to Geometric Jacobian to Analytic Jacobian
        dq = pseudoInverseMat(J,lambda)*dXe;
        
        // Update law
        q += 0.5*dq;
        
        // Update error
        C_IE = jointToRotMat(q);
        C_err = C_des*transposeMat(C_IE);
        
        dr = r_des - jointToPosition(q);
        dph = rotMatToRotVec(C_err);
        dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
        
        num_it++;
    }
    std::cout << "iteration: " << num_it << std::endl << ", value : " << q << std::endl;

    return q;
}

VectorXd Kinematics::inverseKinematics_R(Vector3d r_des, Matrix3d C_des, VectorXd q0, double tol)
{
    // Input: desired end-effector position, desired end-effector orientation, initial guess for joint angles, threshold for the stopping-criterion
    // Output: joint angles which match desired end-effector position and orientation
    double num_it = 0;
    MatrixXd J_P(3,6), J_R(3,6), J(6,6), pinvJ(6,6);
    Matrix3d C_err, C_IE;
    VectorXd q(6),dq(6),dXe(6);
    Vector3d dr, dph;
    double lambda;
    
    //* Set maximum number of iterations
    double max_it = 200;
    
    //* Initialize the solution with the initial guess
    q = q0;
    C_IE = jointToRotMat_R(q);
    C_err = C_des*transposeMat(C_IE);
    
    //* Damping factor
    lambda = 0.001;
    
    //* Initialize error
    dr = r_des - jointToPosition_R(q);
    dph = rotMatToRotVec(C_err);
    dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
    
    ////////////////////////////////////////////////
    //** Iterative inverse kinematics
    ////////////////////////////////////////////////
    
    //* Iterate until terminating condition
    while (num_it<max_it && dXe.norm()>tol)
    {
        //Compute Inverse Jacobian
        J_P = jointToPosJac_R(q);
        J_R = jointToRotJac_R(q);

        J.block(0,0,3,6) = J_P;
        J.block(3,0,3,6) = J_R; // Geometric Jacobian
        
        // Convert to Geometric Jacobian to Analytic Jacobian
        dq = pseudoInverseMat(J,lambda)*dXe;
        
        // Update law
        q += 0.5*dq;
        
        // Update error
        C_IE = jointToRotMat_R(q);
        C_err = C_des*C_IE.transpose();
        
        dr = r_des - jointToPosition_R(q);
        dph = rotMatToRotVec(C_err);
        dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
        
        num_it++;
    }
    // std::cout << "iteration: " << num_it << std::endl << ", value : " << q << std::endl;
    
    return q;
}

VectorXd Kinematics::Geometric_IK_L(VectorXd GB_cfg, VectorXd GF_cfg) {
    double A, B, C; // thigh, calf
    double alpha, c5;
    VectorXd PH(3) ;
    VectorXd p1(3), p2(3), p7(3), pf(3), fa(3), r(3);
    VectorXd q(6);
    MatrixXd R1(3,3), R7(3,3), RH(3,3);

    MatrixXd GB = MatrixXd::Identity(4,4);
    MatrixXd GF = MatrixXd::Identity(4,4);

    GB.block(0,0,3,3) = rotMatZ(GB_cfg(5))*rotMatY(GB_cfg(4))*rotMatX(GB_cfg(3));
    GB.block(0,3,3,1) << GB_cfg(0), GB_cfg(1), GB_cfg(2);


    GF.block(0,0,3,3) = rotMatZ(GF_cfg(5))*rotMatY(GF_cfg(4))*rotMatX(GF_cfg(3));
    GF.block(0,3,3,1) << GF_cfg(0), GF_cfg(1), GF_cfg(2);

    PH << 0, 0.05, 0;

    R1 = GB.block(0,0,3,3);
    R7 = GF.block(0,0,3,3);
    pf = GF.block(0,3,3,1);
    fa << 0, 0, 0.037 ;
    p7 = pf+fa ;
   
    A = 0.133;
    B = 0.138;
   
    p1 << GB(0,3), GB(1,3), GB(2,3);
    p2 = p1 + R1 * PH;
   
    r = R7.transpose() * (p2-p7);
    C = r.norm();
    c5 = cos((A*A + B*B - C*C) / (2 * A * B));
   
    if(c5 >= 1) q(3) = 0.0;
   
    else if(c5 <= -1) q(3) = PI;
   
    else q(3)=-acos((A*A + B*B - C*C) / (2 * A * B)) + PI;
   
    alpha = asin((A*sin(PI - q(3)))/C);
   
    q(5) = atan2(r(1), r(2));
    q(4) = -atan2(r(0), sign(r(2))*sqrt(r(1)*r(1)+r(2)*r(2))) - alpha;
   
    RH = R1.transpose() * R7 * rotMatX(-q(5)) * rotMatY(-q(3)-q(4));
   
    q(0) = atan2(-RH(0,1), RH(1,1));
    q(1) = atan2(RH(2,1), -sin(q(0)) * RH(0,1) + RH(1,1) * cos(q(0)));
    q(2) = atan2(-RH(2,0), RH(2,2));
   
    return q;
}

VectorXd Kinematics::Geometric_IK_R(VectorXd GB_cfg, VectorXd GF_cfg) {
    double A, B, C; // thigh, calf
    double alpha, c5;
    VectorXd PH (3) ;
    VectorXd p1(3), p2(3), p7(3), pf(3), fa(3), r(3);
    VectorXd q(6);
    MatrixXd R1(3,3), R7(3,3), RH(3,3);
   
    MatrixXd GB = MatrixXd::Identity(4,4);
    MatrixXd GF = MatrixXd::Identity(4,4);

    GB.block(0,0,3,3) = rotMatZ(GB_cfg(5))*rotMatY(GB_cfg(4))*rotMatX(GB_cfg(3));
    GB.block(0,3,3,1) << GB_cfg(0), GB_cfg(1), GB_cfg(2);


    GF.block(0,0,3,3) = rotMatZ(GF_cfg(5))*rotMatY(GF_cfg(4))*rotMatX(GF_cfg(3));
    GF.block(0,3,3,1) << GF_cfg(0), GF_cfg(1), GF_cfg(2);
    PH << 0, -0.05, 0;
   
    R1 = GB.block(0,0,3,3);
    R7 = GF.block(0,0,3,3);
    pf = GF.block(0,3,3,1);
    fa << 0, 0, 0.037 ;
    p7 = pf+fa ;
   
    A = 0.133;
    B = 0.138;
   
    p1 << GB(0,3), GB(1,3), GB(2,3);
    p2 = p1 + R1 * PH;
   
    r= R7.transpose() * (p2-p7);
    C= r.norm();
    c5 = cos((A*A + B*B - C*C) / (2 * A * B));
   
    if(c5 >= 1) q(3) = 0.0;
   
    else if(c5 <= -1) q(3) = PI;
   
    else q(3)=-acos((A*A + B*B - C*C) / (2 * A * B)) + PI;
   
    alpha = asin((A*sin(PI - q(3)))/C);
   
    q(5) = atan2(r(1), r(2));
    q(4) = -atan2(r(0), sign(r(2))*sqrt(r(1)*r(1)+r(2)*r(2))) - alpha;
   
    RH = R1.transpose() * R7 * rotMatX(-q(5)) * rotMatY(-q(3)-q(4));
   
    q(0) = atan2(-RH(0,1), RH(1,1));
    q(1) = atan2(RH(2,1), -sin(q(0)) * RH(0,1) + RH(1,1) * cos(q(0)));
    q(2) = atan2(-RH(2,0), RH(2,2));

    return q;
}