#ifndef CKUBOT_H_
#define CKUBOT_H_

#include "eigen3/Eigen/Dense"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::MatrixXf;
using Eigen::VectorXf;

#define PI      M_PI
#define R2D     180./PI
#define D2R     PI/180.

#define onesecSize  100
#define tasktime    0.01

#define PatternPlan 4
#define PatternElement 6

typedef struct LowerbodyKinematic //coordinate of Body
{
    //*Kubot Lower Body Kinematics
    double BASE_TO_LEG_Y;
    double BASE_TO_LEG_Z;
    double THIGH_LENGTH;
    double CALF_LENGTH;
    double ANKLE_LENGTH;
    double LEG_LENGTH;
    double LEG_SIDE_OFFSET;

    //*Kubot Lower Body Kinematics
    double foot_x_length;
    double foot_y_length;
} LBK;

typedef struct XYPreviewControl // PreviewControl for x-axis
{
    VectorXf ref;
    double m_ref;

    double old_zmp;
    Eigen::Vector3f state; //3X1 matrix
    Eigen::Vector3f new_state; //3X1 matrix
    double E;
    double sum_E;
    double sum_P;
    double U;
    double CoM, dCoM;

} XYPRE;

typedef struct PreviewControl {
    double RefTotalTrajSize;
    double RefTotalTime;
    double PreviewTimeSize;
    double PeriodTimeSize;
    double PreviewTime;
    double PreviewReadyTime;
    //double initTime;

    Eigen::Matrix3f A;
    Eigen::Vector3f B;
    Eigen::RowVector3f C;
    Eigen::Vector4f B_Tilde;
    Eigen::Vector4f I_Tilde;
    MatrixXf F_Tilde;
    Eigen::Matrix4f Q_Tilde;
    MatrixXf R;
    Eigen::Matrix4f A_Tilde;
    Eigen::Matrix4f K_Tilde;

    double G_I;

    Eigen::RowVector3f G_X;
    VectorXf G_P;
    Eigen::Matrix4f A_Tilde_c;
    Eigen::Vector4f X_Tilde;
    XYPRE X, Y;

    int count;
    int setcount;

} PREVIEWCONTROL;

typedef struct ZeroMomentPoint { //ZMP
    PREVIEWCONTROL previewControl;

    Eigen::Vector3f global_ref;
    Eigen::Vector3f global;
    Eigen::Vector3f local;

    double X;
    double Y;
} ZMP;

typedef struct WalkingTime //Step
{
    double sec; //*time while robot is walking

    double periodTime; //*walking periodTime
    double DSP_ratio; //*DSP ratio of walking periodTime
    double SSP_ratio; //*SSP ratio of walking periodTime
    double SSP_start_time;
    double SSP_end_time;
    double SSP_time;

    double XYDSP_ratio; //*DSP ratio of walking XY pattern periodTime
    double XYSSP_ratio; //*SSP ratio of walking XY pattern periodTime
    double XYSSP_start_time;
    double XYSSP_end_time;
    double XYSSP_time;
    double readytime;
} WALKINGTIME;

typedef struct Step //Step
{
    int current, current_footprints;
    double total, footprints;

    double FBstepSize, oldFBstepSize;
    double LRstepSize, oldLRstepSize;
    double TurnSize, oldTurnSize;
    double footHeight;
    double turningPoint;

    enum {
        S_L_F = 1, //start foot L
        S_R_F = -1, //start foot R
        L_L_F = 1, //last foot L
        L_R_F = -1 //last foot R
    };

    int start_foot; //start foot
    int last_foot;

     enum {
         forwardWalking = 1, //walk forward
         backwardWalking = -1, // back forward
         leftWalking = 1, //left
         rightWalking = -1, // right
         leftTurning = 1,
         rightTurning = -1,
         TurnModeOn = 1,
         TurnModeOff = 0,
         WalkinginPlace = 0 //
     };
     //    static int None = 0;
     int walking_f_or_b; //walkig f or b?
     int walking_l_or_r; //walkig l or r?
     int walking_lT_or_rT; //walkig l or r?
     int walking_Turn;

    MatrixXf Lfoot, Rfoot;
    VectorXf preLfoot, preRfoot, prefootCenter;
    VectorXf LastLfoot, LastRfoot, LastfootCenter;
} STEP;


typedef struct Walking {
    STEP step;
    WALKINGTIME time;
//     SWINGLEGVIBRATION SwingLegVibrationCon;
//     LANDINGCON Landing;

    enum \
        {
        LSSP = 1,
        RSSP = -1,
        DSP = 0,
        SSP = 2,
        UP = 3
    };

    // int ft_SP; //* SP by FTsensor
    // int old_ft_SP; //* SP by FTsensor

    int walk_SP; //* SP by WalkingPattern
    int walk_SSP;
    // int old_walk_SP; //* SP by WalkingPattern
    // int old_walk_SSP; //* SSP by WalkingPattern

    // int SP; //* Checking SP by FTsensor, WalkingPattern
    // int old_SP; //* Checking SP by FTsensor, WalkingPattern

    // int SP_check; //* SP for Checking
    // int SP_oldcheck; //* SP for Checking
    // int SP_checking; //* check number for Checking
    int Mode;

    // bool change_FB_walking_flag;
    // bool change_LR_walking_flag;
    // bool change_Turn_flag;
    // bool change_LTurn_flag;
    // bool change_RTurn_flag;

    bool Ready;
    bool walk;
    bool ssp_mode;

    bool NewfootPlan;

    bool initialize;
} WALK;

typedef struct EndPoint {
    //* Current information
    VectorXf current; //* EndPoint's Current Position
    // VectorXf vel; //* EndPoint's Current Velocity
    // VectorXf acc; //* EndPoint's Current Acceleration
    // VectorXf pre; //* EndPoint's Pre Position
    // VectorXf prevel; //* EndPoint's Pre Position
    // VectorXf preacc; //* EndPoint's Pre Position

    //* Reference information
    VectorXf refpos; //* EndPoint's ref Position
    // VectorXf refvel; //* EndPoint's ref Velocity
    // VectorXf refacc; //* EndPoint's ref acceleration
    // VectorXf refquat; //* EndPoint's ref quat
    // VectorXf prerefpos; //* EndPoint's ref Position
    // VectorXf prerefvel; //* EndPoint's ref Velocity
    // VectorXf prerefacc; //* EndPoint's ref Acceleration

    int ID;
} ENDPOINT;

class CKubot {
public:
    ZMP zmp;
    WALK walking;
    ENDPOINT RFoot, LFoot;
    LBK lowerbody;

    Eigen::EigenSolver<MatrixXd> eZMPSolver;
    void initializeCKubot();

    void setWalkingTime(double periodTime, double DSPratio);
    void setWalkingStep(double steps, int Startfoot, double FBsize, double LRsize, double Turnsize, double footHeight);
    void setZmpPreview(double previewTime);

    void initializeZmpTraj(void);
    void generateZmpTraj(MatrixXf LfootPrints, MatrixXf RfootPrints); //* generate ZMP Reference traj
    void FootPlanner(void);

    MatrixXd ZMP_DARE(Eigen::Matrix4d A, Eigen::Vector4d B, Eigen::Matrix4d Q, MatrixXd R);
    void generateFootTraj(); //* generate Foot Reference traj
    void generateFootTraj_ver2();
    void zmpPreviewControl();

    enum EndPointAxis {
        X = 0, Y, Z, Roll, Pitch, Yaw
    };

    double cosWave(double time, double half_period, double init_val, double double_amp);
    double sinWave(double time, double half_period, double init_val, double amp);

    int factorial(int n);
    Eigen::Vector3f genreate_8th_BezierCurve(Eigen::Vector3f point_0_1_2, Eigen::Vector3f point_3, Eigen::Vector3f point_4, Eigen::Vector3f point_5, Eigen::Vector3f point_6_7_8, double period, double time);
    double compensate(double time, double period, double ratio, double init_value, double amp);
};
#endif
