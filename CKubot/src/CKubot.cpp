#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sstream>
#include "CKubot.h"
//#include "e2box_imu_9dofv4.h"
//#include "t_serial.h"

using namespace std;
// using namespace Eigen;
class CKubot C;
//class e2box_imu_9dofv4 E2;

int main(int argc, char **argv) {
    ros::init(argc, argv, "CKubot");
    ros::NodeHandle n;
    ros::Publisher chatter1_pub = n.advertise<std_msgs::Float32>("ZMP_Ref_X", 1000);
    ros::Publisher chatter2_pub = n.advertise<std_msgs::Float32>("ZMP_Ref_Y", 1000);
    ros::Publisher chatter3_pub = n.advertise<std_msgs::Float32>("ZMP_X", 1000);
    ros::Publisher chatter4_pub = n.advertise<std_msgs::Float32>("ZMP_Y", 1000);
    ros::Publisher chatter5_pub = n.advertise<std_msgs::Float32>("CoM_X", 1000);
    ros::Publisher chatter6_pub = n.advertise<std_msgs::Float32>("CoM_Y", 1000);
    // ros::Publisher chatter7_pub = n.advertise<std_msgs::Float32>("time", 1000);
    // ros::Publisher chatter8_pub = n.advertise<std_msgs::Float32>("RFoot_ref_X", 1000);
     ros::Publisher chatter9_pub = n.advertise<std_msgs::Float32>("RFoot_ref_Y", 1000);
    // ros::Publisher chatter10_pub = n.advertise<std_msgs::Float32>("RFoot_ref_Z", 1000);
    // ros::Publisher chatter11_pub = n.advertise<std_msgs::Float32>("LFoot_ref_X", 1000);
     ros::Publisher chatter12_pub = n.advertise<std_msgs::Float32>("LFoot_ref_Y", 1000);
    // ros::Publisher chatter13_pub = n.advertise<std_msgs::Float32>("LFoot_ref_Z", 1000);
    // ros::Publisher chatter14_pub = n.advertise<std_msgs::Float32>("Foot_X", 1000);
    // ros::Publisher chatter15_pub = n.advertise<std_msgs::Float32>("Foot_Y", 1000);
    // ros::Publisher chatter16_pub = n.advertise<std_msgs::Float32>("Foot_Z", 1000);
    ros::Publisher chatter17_pub = n.advertise<std_msgs::Float32>("LFoot_Yaw", 1000);


    int cnt = 0;
    double t = 0;

    //std::cout<<"q:="<<E2.m_dQuaternion[0]<<std::endl;

    C.initializeCKubot();
    C.setWalkingStep(16, C.walking.step.S_R_F, 0, 0, 0, 0.05);
    C.setWalkingTime(0.4, 0.3);
    C.setZmpPreview(1.5);
    C.initializeZmpTraj();
    C.FootPlanner();
    C.generateZmpTraj(C.walking.step.Lfoot, C.walking.step.Rfoot);
    
    ros::Rate loop_rate(onesecSize);

    while (ros::ok()) {
        std_msgs::Float32 msg1;
        std_msgs::Float32 msg2;
        std_msgs::Float32 msg3;
        std_msgs::Float32 msg4;
        std_msgs::Float32 msg5;
        std_msgs::Float32 msg6;
        // std_msgs::Float32 msg7;
        // std_msgs::Float32 msg8;
         std_msgs::Float32 msg9;
        // std_msgs::Float32 msg10;
        // std_msgs::Float32 msg11;
         std_msgs::Float32 msg12;
        // std_msgs::Float32 msg13;
        // std_msgs::Float32 msg14;
        // std_msgs::Float32 msg15;
        // std_msgs::Float32 msg16;
        std_msgs::Float32 msg17;
        // std::stringstream ss;
        // ss <<  "hello world " << count;

        msg1.data = C.zmp.previewControl.X.ref(cnt);
        msg2.data = C.zmp.previewControl.Y.ref(cnt);
        msg3.data = C.zmp.previewControl.X.old_zmp;
        msg4.data = C.zmp.previewControl.Y.old_zmp;
        msg5.data = C.zmp.previewControl.X.CoM;
        msg6.data = C.zmp.previewControl.Y.CoM;        
        // msg7.data = 0.001*cnt;
        // msg8.data = C.RFoot.refpos(0);
         msg9.data = C.RFoot.refpos(1);
        // msg10.data = C.RFoot.refpos(2);
        // msg11.data = C.LFoot.refpos(0);
         msg12.data = C.LFoot.refpos(1);
        // msg13.data = C.LFoot.refpos(2);
        msg17.data = C.LFoot.refpos(5);


        chatter1_pub.publish(msg1);
        chatter2_pub.publish(msg2);
        chatter3_pub.publish(msg3);
        chatter4_pub.publish(msg4);
        chatter5_pub.publish(msg5);
        chatter6_pub.publish(msg6);
        // chatter7_pub.publish(msg7);
        // chatter8_pub.publish(msg8);
         chatter9_pub.publish(msg9);
        // chatter10_pub.publish(msg10);
        // chatter11_pub.publish(msg11);
         chatter12_pub.publish(msg12);
        // chatter13_pub.publish(msg13);
        chatter17_pub.publish(msg17);

        // ROS_INFO("ZMP_Ref_X : %f", msg1.data);
        // ROS_INFO("ZMP_Ref_Y : %f", msg2.data);
        // ROS_INFO("ZMP_X : %f", msg3.data);
        // ROS_INFO("ZMP_Y : %f", msg4.data);
        // ROS_INFO("CoM_X : %f", msg5.data);
        // ROS_INFO("CoM_Y : %f", msg6.data);
        // ROS_INFO("time : %f", msg7.data);
        // ROS_INFO("%s", msg.data.c_str());

        if (t > C.zmp.previewControl.PreviewReadyTime) {
            if(C.walking.time.sec > C.walking.time.periodTime) {
                C.walking.step.current++;
                C.walking.time.sec = 0.0;
                C.walking.step.LastLfoot = C.LFoot.refpos;
                C.walking.step.LastRfoot = C.RFoot.refpos;
            }
        }

        t = cnt / (double)onesecSize;
        C.zmp.previewControl.count = cnt;
        C.walking.time.sec = t - C.zmp.previewControl.PreviewReadyTime - C.walking.time.periodTime * C.walking.step.current;
        C.generateFootTraj_ver2();
        C.zmpPreviewControl();
            
        // msg14.data = Bezier_Traj(0);
        // msg15.data = Bezier_Traj(1);
        // msg16.data = Bezier_Traj(2);
        // chatter14_pub.publish(msg14);
        // chatter15_pub.publish(msg15);
        // chatter16_pub.publish(msg16);
        // cout << "X : " << Bezier_Traj(0) << ", Y : " << Bezier_Traj(1) << ", Z : " << Bezier_Traj(2) << endl;

        // cout << "cnt : " << cnt << endl;

        ros::spinOnce();
        loop_rate.sleep();
        cnt++;

        if(t>C.zmp.previewControl.RefTotalTrajSize - C.zmp.previewControl.PreviewTimeSize){
          cnt = 0;
          t = 0;

          C.initializeCKubot();
          C.setWalkingStep(16, C.walking.step.S_R_F, 0, -0.05, 0, 0.05);
          C.setWalkingTime(0.4, 0.3);
          C.setZmpPreview(1.5);
          C.initializeZmpTraj();
          C.FootPlanner();
          C.generateZmpTraj(C.walking.step.Lfoot, C.walking.step.Rfoot);
        }
    }

    return 0;
}

void CKubot::initializeCKubot() {
    lowerbody.LEG_SIDE_OFFSET = 0.05;   
    //  walking.step.Lfoot = MatrixXf::Zero(PatternElement, PatternPlan / 2);
    //  walking.step.Rfoot = MatrixXf::Zero(PatternElement, PatternPlan / 2);
    // walking.step.preLfoot = VectorXf::Zero(PatternElement);
    // walking.step.preRfoot = VectorXf::Zero(PatternElement);
    // walking.step.preLfoot = VectorXf::Zero(PatternElement);
    // walking.step.prefootCenter = VectorXf::Zero(PatternElement);
    walking.step.LastLfoot = VectorXf::Zero(PatternElement);
    walking.step.LastRfoot = VectorXf::Zero(PatternElement);
    walking.step.LastfootCenter = VectorXf::Zero(PatternElement);

    walking.step.LastLfoot(Y) = lowerbody.LEG_SIDE_OFFSET;
    walking.step.LastRfoot(Y) = lowerbody.LEG_SIDE_OFFSET * (-1);

    walking.step.current = 0;
}

void CKubot::setWalkingStep(double steps, int Startfoot, double FBsize, double LRsize, double Turnsize, double footHeight) {
    /* setWalkingStep
     * input : steps, starting foot (select walking.step.S_L_F or walking.step.S_R_F), step size, foot height for walking
     * output : Setting Steps, starting foot
     */

    walking.step.total = steps;
    walking.step.footprints = steps+1;
     walking.step.start_foot = Startfoot;
     if(LRsize < 0 || Turnsize < 0) {
         walking.step.start_foot = walking.step.S_R_F;
     }
     else if(LRsize > 0 || Turnsize > 0) {
         walking.step.start_foot = walking.step.S_L_F;
     }

    walking.step.footHeight = footHeight;

//     if(FBsize > 0) {
//         walking.step.walking_f_or_b = walking.step.forwardWalking;
//     }
//     else if(FBsize < 0) {
//         walking.step.walking_f_or_b = walking.step.backwardWalking;
//     }
//     else {
//         walking.step.walking_f_or_b = 0;
//     }

    walking.step.FBstepSize = FBsize;
//     walking.step.oldFBstepSize = walking.step.FBstepSize;

     if(LRsize > 0) {
         walking.step.walking_l_or_r= walking.step.leftWalking;
     }
     else if(LRsize < 0) {
         walking.step.walking_l_or_r = walking.step.rightWalking;
     }
     else {
         walking.step.walking_l_or_r = 0;
     }

    walking.step.LRstepSize = LRsize;
//     walking.step.oldLRstepSize = walking.step.LRstepSize;

     if(Turnsize != 0) {
         if(Turnsize > 0)
         {
             walking.step.walking_lT_or_rT= walking.step.rightTurning;
             walking.step.walking_Turn = walking.step.TurnModeOn;
         }
         else if(Turnsize < 0)
         {
             walking.step.walking_lT_or_rT = walking.step.leftTurning;
             walking.step.walking_Turn = walking.step.TurnModeOn;
         }
         walking.step.turningPoint = walking.step.FBstepSize/Turnsize;

         if(walking.step.total == 1)
         {
             walking.step.walking_Turn = walking.step.TurnModeOff;
         }
     }
     else {
         walking.step.walking_lT_or_rT = 0;
         walking.step.walking_Turn = walking.step.TurnModeOff;
     }

    walking.step.TurnSize = Turnsize;
//     walking.step.oldTurnSize = walking.step.TurnSize;

//     printf("FBstepSize =%lf mm\t LRstepSize =%lf mm\t TurnSize =%lf degree\n",walking.step.FBstepSize,walking.step.LRstepSize,walking.step.TurnSize*R2D);
//     printf("footHeight =%lf mm\n",walking.step.footHeight);
//     printf("turningPoint =%lf mm\n",walking.step.turningPoint);
}

void CKubot::setWalkingTime(double periodTime, double DSPratio) {//주기랑 dsp정해주는 함수 로봇에선 0.6초 dsp는 0.4초로 정함
    /* setWalkingTime,
     * input : period time, DSP ratio for walking,s
     * output : Setting walking period time, DSP ratio, SSP ratio
     */

    walking.time.periodTime = periodTime;
    walking.time.DSP_ratio = DSPratio;
    walking.time.SSP_ratio = 1-walking.time.DSP_ratio;

    printf("Walking periodTime = %lf sec\t DSP %lf\t SSP %lf\n", periodTime, walking.time.DSP_ratio, walking.time.SSP_ratio);

     walking.time.SSP_start_time = walking.time.periodTime*walking.time.DSP_ratio/2;
     walking.time.SSP_time = walking.time.periodTime*walking.time.SSP_ratio;
     walking.time.SSP_end_time = walking.time.SSP_start_time+walking.time.SSP_time;

    // walking.time.XYDSP_ratio = 1.05*DSPratio;
    // walking.time.XYSSP_ratio = 1-walking.time.XYDSP_ratio;

    // walking.time.XYSSP_start_time = walking.time.periodTime*walking.time.XYDSP_ratio/2;
    // walking.time.XYSSP_time = walking.time.periodTime*walking.time.XYSSP_ratio;
    // walking.time.XYSSP_end_time = walking.time.XYSSP_start_time+walking.time.XYSSP_time;
}

void CKubot::setZmpPreview(double previewTime) {
    /* setZmpPreview
     * input : preview time N_L for ZMP preview control
     * output : previewTime --> setZmpPreviewTime [Crobot class's Function]
     *          Setting variable for ZMP preview control
     *
     * Before run function
     * [First, Run once Crobot class's Function initializeSystemID when starting program]
     */

    std::cout<<"===== Setting ZMP preview control ====="<<std::endl;

    zmp.previewControl.PreviewTime = previewTime;
    zmp.previewControl.PreviewTimeSize = zmp.previewControl.PreviewTime*(double)onesecSize;

    double T = (double)tasktime;

    zmp.previewControl.A << 1, T, T*T/2.0, \
                            0, 1, T, \
                            0, 0, 1;

    zmp.previewControl.B << T*T*T/6.0, \
                            T*T/2.0, \
                            T;

    zmp.previewControl.C << 1, 0, -0.25/9.81;

    zmp.previewControl.B_Tilde.head(1) = zmp.previewControl.C*zmp.previewControl.B;
    zmp.previewControl.B_Tilde.tail(3) = zmp.previewControl.B;

    zmp.previewControl.I_Tilde << 1, 0, 0, 0;

    zmp.previewControl.F_Tilde = MatrixXf::Zero(4, 3);
    zmp.previewControl.F_Tilde.block(0, 0, 1, 3) = zmp.previewControl.C * zmp.previewControl.A;
    zmp.previewControl.F_Tilde.block(1, 0, 3, 3) = zmp.previewControl.A;

    zmp.previewControl.Q_Tilde.block(0, 0, 1, 1) = MatrixXf::Identity(1, 1);
    zmp.previewControl.Q_Tilde.block(1, 1, 3, 3) = MatrixXf::Zero(3, 3);

    zmp.previewControl.R = MatrixXf::Identity(1, 1)*0.000001;

    zmp.previewControl.A_Tilde.block(0, 0, 4, 1) = zmp.previewControl.I_Tilde;
    zmp.previewControl.A_Tilde.block(0, 1, 4, 3) = zmp.previewControl.F_Tilde;

   zmp.previewControl.K_Tilde = (ZMP_DARE(zmp.previewControl.A_Tilde.cast <double> (), zmp.previewControl.B_Tilde.cast <double> (), zmp.previewControl.Q_Tilde.cast <double> (), zmp.previewControl.R.cast <double> ())).cast <float> ();

    zmp.previewControl.G_I = ((zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.B_Tilde).inverse()*zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.I_Tilde)(0,0);    zmp.previewControl.G_P = VectorXf::Zero(zmp.previewControl.PreviewTimeSize);
    zmp.previewControl.G_X = (zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.B_Tilde).inverse()*zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.F_Tilde;
    zmp.previewControl.A_Tilde_c = zmp.previewControl.A_Tilde - zmp.previewControl.B_Tilde*(zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.B_Tilde).inverse()*zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.A_Tilde;

    for (int pre_cnt = 0; pre_cnt < zmp.previewControl.PreviewTimeSize; pre_cnt++) {
        if (pre_cnt == 0) {
            zmp.previewControl.X_Tilde = -zmp.previewControl.A_Tilde_c.transpose() * zmp.previewControl.K_Tilde * zmp.previewControl.I_Tilde;
            zmp.previewControl.G_P(pre_cnt) = -zmp.previewControl.G_I;
        }
        else {
            zmp.previewControl.X_Tilde = zmp.previewControl.A_Tilde_c.transpose() * zmp.previewControl.X_Tilde;
            zmp.previewControl.G_P(pre_cnt) = ((zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose() * zmp.previewControl.K_Tilde * zmp.previewControl.B_Tilde).inverse() * zmp.previewControl.B_Tilde.transpose() * zmp.previewControl.X_Tilde)(0,0);
        }
    }

    // cout << "G_I: " << zmp.previewControl.G_I << endl;
    // cout << "G_X: " << zmp.previewControl.G_X << endl;
    // cout << "G_P: " << zmp.previewControl.G_P << endl;
    std::cout<<"===== Setting End ZMP preview control ====="<<std::endl;
}

void CKubot::initializeZmpTraj() {
    //zmp.previewControl.initTime = 2.0;
    zmp.previewControl.PreviewReadyTime = 2.0;    
    zmp.previewControl.PeriodTimeSize = walking.time.periodTime*(double)onesecSize;//
    zmp.previewControl.RefTotalTrajSize = (zmp.previewControl.PreviewReadyTime \
                                           + walking.step.total * walking.time.periodTime \
                                           + 2*zmp.previewControl.PreviewTime)*(double)onesecSize;//(preview readytime + 주기*걸음수 + previe time)*100
    zmp.previewControl.RefTotalTime = zmp.previewControl.RefTotalTrajSize/(double)onesecSize;

    std::cout<<"RefTotalTrajSize :"<<zmp.previewControl.RefTotalTrajSize<<std::endl;
    std::cout<<"RefTotalTrajTime :"<<zmp.previewControl.RefTotalTime<<std::endl;
            
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////

    zmp.previewControl.Y.ref  = VectorXf::Zero((int)(zmp.previewControl.RefTotalTrajSize+0.001));
//    zmp.previewControl.Y.m_ref = walking.step.LastfootCenter(Y);
    zmp.previewControl.Y.state = VectorXf::Zero(3);	//3X1 matrix
    zmp.previewControl.Y.E = 0;
    // if(CommandFlag == ACT_PATTERN_TESTING)
    //     zmp.previewControl.Y.sum_E = zmp.previewControl.Y.sum_E;
    // else
    zmp.previewControl.Y.sum_E = 0;
    zmp.previewControl.Y.sum_P = 0;
    zmp.previewControl.Y.U = 0;

    zmp.previewControl.X.ref = VectorXf::Zero((int)(zmp.previewControl.RefTotalTrajSize+0.001));
    // zmp.previewControl.X.m_ref = walking.step.LastfootCenter(X);
    zmp.previewControl.X.state = VectorXf::Zero(3);	//3X1 matrix
    zmp.previewControl.X.E = 0;
    // if(CommandFlag == ACT_PATTERN_TESTING)
    //     zmp.previewControl.X.sum_E = zmp.previewControl.X.sum_E;
    // else
    zmp.previewControl.X.sum_E = 0;
    zmp.previewControl.X.sum_P = 0;
    zmp.previewControl.X.U = 0;

    zmp.previewControl.Y.new_state = VectorXf::Zero(3);
    zmp.previewControl.Y.old_zmp = walking.step.LastfootCenter(Y);
    zmp.previewControl.Y.CoM = walking.step.LastfootCenter(Y);
    zmp.previewControl.Y.new_state(0) = zmp.previewControl.Y.CoM;
    // zmp.previewControl.Y.dCoM = 0;

    zmp.previewControl.X.new_state = VectorXf::Zero(3);
    zmp.previewControl.X.old_zmp = walking.step.LastfootCenter(X);
    zmp.previewControl.X.CoM = walking.step.LastfootCenter(X);
    zmp.previewControl.X.new_state(0) = zmp.previewControl.X.CoM;
    // zmp.previewControl.X.dCoM = 0;

    zmp.previewControl.count = 0;
}

MatrixXd CKubot::ZMP_DARE(Eigen::Matrix4d A, Eigen::Vector4d B, Eigen::Matrix4d Q, MatrixXd R) { //Kookmin.Univ Preview
    unsigned int nSize = A.rows();
    MatrixXd Z(nSize* 2, nSize* 2);

    Z.block(0, 0, nSize, nSize) = A+ B* R.inverse()* B.transpose()* (A.inverse()).transpose() * Q;
    Z.block(0, nSize, nSize, nSize) = -B* R.inverse()* B.transpose()* (A.inverse()).transpose();
    Z.block(nSize, 0, nSize, nSize) = -(A.inverse()).transpose()* Q;
    Z.block(nSize, nSize, nSize, nSize) = (A.inverse()).transpose();

    eZMPSolver.compute(Z, true);

    Eigen::MatrixXcd U(nSize* 2, nSize);
    unsigned int j=0;
    for (unsigned int i=0; i<nSize* 2; i++)
    {
        std::complex<double> eigenvalue = eZMPSolver.eigenvalues()[i];
        double dReal = eigenvalue.real();
        double dImag = eigenvalue.imag();

        if( std::sqrt((dReal* dReal) + (dImag* dImag)) < 1.0)
        {
            U.block(0, j, nSize* 2, 1) = eZMPSolver.eigenvectors().col(i);
            j++;
        }
    }
    if(j != nSize)
    {
        printf("Warning! ******* Pelvis Planning *******\n");
    }

    Eigen::MatrixXcd U1 = U.block(0, 0, nSize, nSize);
    Eigen::MatrixXcd U2 = U.block(nSize, 0, nSize, nSize);

    Eigen::MatrixXcd X = U2 * U1.inverse();

    return X.real();
}

void CKubot::generateZmpTraj(Eigen::MatrixXf LfootPrints, Eigen::MatrixXf RfootPrints) {
    for (int pre_cnt = 0; \
            pre_cnt < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + 0 * walking.time.periodTime) / zmp.previewControl.RefTotalTime; \
            pre_cnt++) {
        zmp.previewControl.X.ref(pre_cnt) = walking.step.LastfootCenter(X);
        zmp.previewControl.Y.ref(pre_cnt) = walking.step.LastfootCenter(Y);
    }
    //    std::cout<<"Generate Zmp Traj"<<std::endl;
    for (int m_step = 0; m_step < walking.step.total; m_step++) {
        if (walking.step.start_foot == walking.step.S_L_F) {
           if ((m_step) % 2 == 0) {

               for (int pre_cnt = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step) * walking.time.periodTime) / zmp.previewControl.RefTotalTime;  \
                       pre_cnt < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step + 1) * walking.time.periodTime) / zmp.previewControl.RefTotalTime; \
                       pre_cnt++) {

                   zmp.previewControl.X.ref(pre_cnt) = RfootPrints(X, m_step / 2) ;
                   zmp.previewControl.Y.ref(pre_cnt) = RfootPrints(Y, m_step / 2) ;

               }
           }
           else if ((m_step) % 2 == 1) {

               for (int pre_cnt = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step) * walking.time.periodTime) / zmp.previewControl.RefTotalTime; \
                       pre_cnt < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step + 1) * walking.time.periodTime) / zmp.previewControl.RefTotalTime; \
                       pre_cnt++) {

                   zmp.previewControl.X.ref(pre_cnt) = LfootPrints(X, (int) ((double) (m_step) / 2.)) ;
                   zmp.previewControl.Y.ref(pre_cnt) = LfootPrints(Y, (int) ((double) (m_step) / 2.)) ;

               }
           }
        }

        else if (walking.step.start_foot == walking.step.S_R_F) {
           if ((m_step) % 2 == 0) {

               for (int pre_cnt = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step) * walking.time.periodTime) / zmp.previewControl.RefTotalTime;  \
                       pre_cnt < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step + 1) * walking.time.periodTime) / zmp.previewControl.RefTotalTime; \
                       pre_cnt++) {

                   zmp.previewControl.X.ref(pre_cnt) = LfootPrints(X, m_step / 2) ;
                   zmp.previewControl.Y.ref(pre_cnt) = LfootPrints(Y, m_step / 2) ;

               }
           }
           else if ((m_step) % 2 == 1) {

               for (int pre_cnt = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step) * walking.time.periodTime) / zmp.previewControl.RefTotalTime; \
                       pre_cnt < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step + 1) * walking.time.periodTime) / zmp.previewControl.RefTotalTime; \
                       pre_cnt++) {

                   zmp.previewControl.X.ref(pre_cnt) = RfootPrints(X, (int) ((double) (m_step) / 2.)) ;
                   zmp.previewControl.Y.ref(pre_cnt) = RfootPrints(Y, (int) ((double) (m_step) / 2.)) ;

               }
           }
        }
    }



        
    for (int pre_cnt = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (walking.step.total) * walking.time.periodTime) / zmp.previewControl.RefTotalTime; \
            pre_cnt < (int) (zmp.previewControl.RefTotalTrajSize + 0.001); \
            pre_cnt++) {

        if (walking.step.start_foot == walking.step.S_L_F) {
            if ((int) (walking.step.footprints) % 2 == 0) {
                zmp.previewControl.X.ref(pre_cnt) = (LfootPrints(X, walking.step.footprints / 2 - 1) + RfootPrints(X, walking.step.footprints / 2 - 1)) / 2.;
                zmp.previewControl.Y.ref(pre_cnt) = (LfootPrints(Y, walking.step.footprints / 2 - 1) + RfootPrints(Y, walking.step.footprints / 2 - 1)) / 2.;
            }
            else {
                zmp.previewControl.X.ref(pre_cnt) = (LfootPrints(X, (int) (walking.step.footprints / 2.) - 1) + RfootPrints(X, (int) (walking.step.footprints / 2.))) / 2.;
                zmp.previewControl.Y.ref(pre_cnt) = (LfootPrints(Y, (int) (walking.step.footprints / 2.) - 1) + RfootPrints(Y, (int) (walking.step.footprints / 2.))) / 2.;
            }
        }
        else if (walking.step.start_foot == walking.step.S_R_F) {
            if ((int) (walking.step.footprints) % 2 == 0) {
                zmp.previewControl.X.ref(pre_cnt) = (LfootPrints(X, walking.step.footprints / 2 - 1) + RfootPrints(X, walking.step.footprints / 2 - 1)) / 2.;
                zmp.previewControl.Y.ref(pre_cnt) = (LfootPrints(Y, walking.step.footprints / 2 - 1) + RfootPrints(Y, walking.step.footprints / 2 - 1)) / 2.;
            }
            else {
                zmp.previewControl.X.ref(pre_cnt) = (LfootPrints(X, (int) (walking.step.footprints / 2.)) + RfootPrints(X, (int) (walking.step.footprints / 2.) - 1)) / 2.;
                zmp.previewControl.Y.ref(pre_cnt) = (LfootPrints(Y, (int) (walking.step.footprints / 2.)) + RfootPrints(Y, (int) (walking.step.footprints / 2.) - 1)) / 2.;
            }
        }
    }
    std::cout<<"Planning ZmpTraj"<<std::endl;
}

void CKubot::FootPlanner() {
    LFoot.refpos = VectorXf::Zero(PatternElement);
    RFoot.refpos = VectorXf::Zero(PatternElement);

    if(walking.step.start_foot == walking.step.S_L_F)
    {
        if((int)(walking.step.footprints)%2 == 0)
        {
            walking.step.Lfoot = MatrixXf::Zero(PatternElement,walking.step.footprints/2);
            walking.step.Rfoot = MatrixXf::Zero(PatternElement,walking.step.footprints/2);
        }
        else
        {
            walking.step.Lfoot = MatrixXf::Zero(PatternElement,(int)(walking.step.footprints/2));
            walking.step.Rfoot = MatrixXf::Zero(PatternElement,(int)(walking.step.footprints/2)+1);
        }
    }
    else if(walking.step.start_foot == walking.step.S_R_F)
    {
        if((int)(walking.step.footprints)%2 == 0)
        {
            walking.step.Lfoot = MatrixXf::Zero(PatternElement,walking.step.footprints/2);
            walking.step.Rfoot = MatrixXf::Zero(PatternElement,walking.step.footprints/2);
        }
        else
        {
            walking.step.Lfoot = MatrixXf::Zero(PatternElement,(int)(walking.step.footprints/2)+1);
            walking.step.Rfoot = MatrixXf::Zero(PatternElement,(int)(walking.step.footprints/2));
        }
    }
    for(int m_step = 0; m_step < walking.step.footprints; m_step++)
    {
        if(walking.step.start_foot == walking.step.S_L_F)
        {
            if((m_step) == int(walking.step.footprints)-1)
            {
                if((int)(walking.step.footprints)%2 == 1)
                {
                    walking.step.Rfoot(X,m_step/2) = (1-walking.step.walking_Turn)*(walking.step.Lfoot(X,m_step/2-1))\
                                                   +   (walking.step.walking_Turn)*((-lowerbody.LEG_SIDE_OFFSET-walking.step.turningPoint)*sin((m_step-1)*-walking.step.TurnSize));
                    walking.step.Rfoot(Y,m_step/2) = (1-walking.step.walking_Turn)*(walking.step.Lfoot(Y,m_step/2-1)-2*lowerbody.LEG_SIDE_OFFSET)\
                                                   +   (walking.step.walking_Turn)*((-lowerbody.LEG_SIDE_OFFSET-walking.step.turningPoint)*cos((m_step-1)*-walking.step.TurnSize)+walking.step.turningPoint);
                    walking.step.Rfoot(Yaw,m_step/2) = (m_step-1)*walking.step.TurnSize;
                }
                else
                {
                    walking.step.Lfoot(X,(int)((double)(m_step)/2.)) = (1-walking.step.walking_Turn)*walking.step.Rfoot(X,(int)((double)(m_step)/2.))\
                                                                     +   (walking.step.walking_Turn)*((lowerbody.LEG_SIDE_OFFSET-walking.step.turningPoint)*sin((m_step-1)*-walking.step.TurnSize));
                    walking.step.Lfoot(Y,(int)((double)(m_step)/2.)) = (1-walking.step.walking_Turn)*(((int)((double)(m_step)/2.))*walking.step.LRstepSize + lowerbody.LEG_SIDE_OFFSET)\
                                                                     +   (walking.step.walking_Turn)*((lowerbody.LEG_SIDE_OFFSET-walking.step.turningPoint)*cos((m_step-1)*-walking.step.TurnSize)+walking.step.turningPoint);
                    walking.step.Lfoot(Yaw,(int)((double)(m_step)/2.)) = (m_step-1)*walking.step.TurnSize;
                }
            }
            else if ((m_step) % 2 == 0) {
                if (m_step == 0) {
                    walking.step.Rfoot(X, m_step / 2) = 0;
                    walking.step.Rfoot(Y, m_step / 2) = (1 - walking.step.walking_Turn)*((m_step / 2) * walking.step.LRstepSize - lowerbody.LEG_SIDE_OFFSET)\
                                                      + (walking.step.walking_Turn)*((-lowerbody.LEG_SIDE_OFFSET - walking.step.turningPoint) * cos((m_step)*-walking.step.TurnSize) + walking.step.turningPoint);
                    walking.step.Rfoot(Yaw, m_step / 2) = 0;
                }
                else {
                    walking.step.Rfoot(X, m_step / 2) = (1 - walking.step.walking_Turn)*((m_step) * walking.step.FBstepSize)\
                                                      + (walking.step.walking_Turn)*((-lowerbody.LEG_SIDE_OFFSET - walking.step.turningPoint) * sin((m_step)*-walking.step.TurnSize));
                    walking.step.Rfoot(Y, m_step / 2) = (1 - walking.step.walking_Turn)*((m_step / 2) * walking.step.LRstepSize - lowerbody.LEG_SIDE_OFFSET)\
                                                      + (walking.step.walking_Turn)*((-lowerbody.LEG_SIDE_OFFSET - walking.step.turningPoint) * cos((m_step)*-walking.step.TurnSize) + walking.step.turningPoint);
                    walking.step.Rfoot(Yaw, m_step / 2) = (m_step) * walking.step.TurnSize;
                }
            }
            else
            {
                walking.step.Lfoot(X,(int)((double)(m_step)/2.)) = (1-walking.step.walking_Turn)*((m_step)*walking.step.FBstepSize)\
                                                                 +   (walking.step.walking_Turn)*((lowerbody.LEG_SIDE_OFFSET-walking.step.turningPoint)*sin((m_step)*-walking.step.TurnSize));
                walking.step.Lfoot(Y,(int)((double)(m_step)/2.)) = (1-walking.step.walking_Turn)*(((int)((double)(m_step)/2.)+1)*walking.step.LRstepSize + lowerbody.LEG_SIDE_OFFSET)\
                                                                 +   (walking.step.walking_Turn)*((lowerbody.LEG_SIDE_OFFSET-walking.step.turningPoint)*cos((m_step)*-walking.step.TurnSize)+walking.step.turningPoint);
                walking.step.Lfoot(Yaw,(int)((double)(m_step)/2.)) = (m_step)*walking.step.TurnSize;
            }
        }
        else if(walking.step.start_foot == walking.step.S_R_F)
        {
            if((m_step) == int(walking.step.footprints)-1)
            {
                if((int)(walking.step.footprints)%2 == 1)
                {
                    walking.step.Lfoot(X,m_step/2)   = (1-walking.step.walking_Turn)*(walking.step.Rfoot(X,m_step/2-1))\
                                                     +   (walking.step.walking_Turn)*((lowerbody.LEG_SIDE_OFFSET-walking.step.turningPoint)*sin((m_step-1)*-walking.step.TurnSize));
                    walking.step.Lfoot(Y,m_step/2)   = (1-walking.step.walking_Turn)*(walking.step.Rfoot(Y,m_step/2-1)+2*lowerbody.LEG_SIDE_OFFSET)\
                                                     +   (walking.step.walking_Turn)*((lowerbody.LEG_SIDE_OFFSET-walking.step.turningPoint)*cos((m_step-1)*-walking.step.TurnSize)+walking.step.turningPoint);
                    walking.step.Lfoot(Yaw,m_step/2) = (m_step-1)*walking.step.TurnSize;
                }
                else
                {
                    walking.step.Rfoot(X,(int)((double)(m_step)/2.)) = (1-walking.step.walking_Turn)*(walking.step.Lfoot(X,(int)((double)(m_step)/2.)))\
                                                                     +   (walking.step.walking_Turn)*((-lowerbody.LEG_SIDE_OFFSET-walking.step.turningPoint)*sin((m_step-1)*-walking.step.TurnSize));
                    walking.step.Rfoot(Y,(int)((double)(m_step)/2.)) = (1-walking.step.walking_Turn)*(((int)((double)(m_step)/2.))*walking.step.LRstepSize - lowerbody.LEG_SIDE_OFFSET)\
                                                                     +   (walking.step.walking_Turn)*((-lowerbody.LEG_SIDE_OFFSET-walking.step.turningPoint)*cos((m_step-1)*-walking.step.TurnSize)+walking.step.turningPoint);
                    walking.step.Rfoot(Yaw,(int)((double)(m_step)/2.)) = (m_step-1)*walking.step.TurnSize;
                }
            }
            else if((m_step)%2 == 0)
{
                if (m_step == 0) {
                    walking.step.Lfoot(X, m_step / 2) = 0;
                    walking.step.Lfoot(Y, m_step / 2) = (1 - walking.step.walking_Turn)*((m_step / 2) * walking.step.LRstepSize + lowerbody.LEG_SIDE_OFFSET)\
                                                      + (walking.step.walking_Turn)*((lowerbody.LEG_SIDE_OFFSET - walking.step.turningPoint) * cos((m_step)*-walking.step.TurnSize) + walking.step.turningPoint);
                    walking.step.Lfoot(Yaw, m_step / 2) = 0;
                }
                else {
                    walking.step.Lfoot(X, m_step / 2) = (1 - walking.step.walking_Turn)*((m_step) * walking.step.FBstepSize)\
                                                      + (walking.step.walking_Turn)*((lowerbody.LEG_SIDE_OFFSET - walking.step.turningPoint) * sin((m_step)*-walking.step.TurnSize));
                    walking.step.Lfoot(Y, m_step / 2) = (1 - walking.step.walking_Turn)*((m_step / 2) * walking.step.LRstepSize + lowerbody.LEG_SIDE_OFFSET)\
                                                      + (walking.step.walking_Turn)*((lowerbody.LEG_SIDE_OFFSET - walking.step.turningPoint) * cos((m_step)*-walking.step.TurnSize) + walking.step.turningPoint);
                    walking.step.Lfoot(Yaw, m_step / 2) = (m_step) * walking.step.TurnSize;

                }
            }
            else
            {
                walking.step.Rfoot(X,(int)((double)(m_step)/2.))   = (1-walking.step.walking_Turn)*((m_step)*walking.step.FBstepSize)\
                                                                   +   (walking.step.walking_Turn)*((-lowerbody.LEG_SIDE_OFFSET-walking.step.turningPoint)*sin((m_step)*-walking.step.TurnSize));
                walking.step.Rfoot(Y,(int)((double)(m_step)/2.))   = (1-walking.step.walking_Turn)*(((int)((double)(m_step)/2.)+1)*walking.step.LRstepSize - lowerbody.LEG_SIDE_OFFSET)\
                                                                   +   (walking.step.walking_Turn)*((-lowerbody.LEG_SIDE_OFFSET-walking.step.turningPoint)*cos((m_step)*-walking.step.TurnSize)+walking.step.turningPoint);
                walking.step.Rfoot(Yaw,(int)((double)(m_step)/2.)) = (m_step)*walking.step.TurnSize;
            }
        }
    }


    std::cout<<"Planning Foot steps"<<std::endl;
}

void CKubot::generateFootTraj() {
    if(walking.time.sec <= walking.time.SSP_start_time) {
       if((walking.step.current) == 0) {
            LFoot.refpos(X) = walking.step.LastLfoot(X);
            LFoot.refpos(Y) = walking.step.LastLfoot(Y);
            LFoot.refpos(Z) = walking.step.LastLfoot(Z); // Z = 0

            RFoot.refpos(X) = walking.step.LastRfoot(X);
            RFoot.refpos(Y) = walking.step.LastRfoot(Y);
            RFoot.refpos(Z) = walking.step.LastRfoot(Z);
        }
    }

    else if(walking.time.sec <= walking.time.SSP_end_time) {
        if(walking.step.current < walking.step.total) {
            if ((walking.step.current) % 2 == 0) {
                if((walking.step.current) == 0){
                    LFoot.refpos(X) = (walking.step.Lfoot(X,(int)((double)(walking.step.current)/2.)) - walking.step.LastLfoot(X))*(walking.time.sec-walking.time.SSP_start_time)/walking.time.SSP_time;
                    LFoot.refpos(Y) = (walking.step.Lfoot(Y,(int)((double)(walking.step.current)/2.)) - walking.step.LastLfoot(Y))*(walking.time.sec-walking.time.SSP_start_time)/walking.time.SSP_time + walking.step.LastLfoot(Y);
                    LFoot.refpos(Z) = sinWave((walking.time.sec-walking.time.SSP_start_time), walking.time.SSP_time, walking.step.LastLfoot(Z), walking.step.footHeight); // Z = 0
                    
                    RFoot.refpos(X) = walking.step.LastRfoot(X);
                    RFoot.refpos(Y) = walking.step.LastRfoot(Y);
                    RFoot.refpos(Z) = walking.step.LastRfoot(Z);
                }
                else {
                    LFoot.refpos(X) = (walking.step.Lfoot(X,(int)((double)(walking.step.current)/2.)) - walking.step.Lfoot(X,(int)((double)(walking.step.current)/2.)-1))*(walking.time.sec-walking.time.SSP_start_time)/walking.time.SSP_time + walking.step.Lfoot(X,(int)((double)(walking.step.current)/2.)-1);
                    LFoot.refpos(Y) = (walking.step.Lfoot(Y,(int)((double)(walking.step.current)/2.)) - walking.step.Lfoot(Y,(int)((double)(walking.step.current)/2.)-1))*(walking.time.sec-walking.time.SSP_start_time)/walking.time.SSP_time + walking.step.Lfoot(Y,(int)((double)(walking.step.current)/2.)-1);
                    LFoot.refpos(Z) = sinWave((walking.time.sec-walking.time.SSP_start_time), walking.time.SSP_time, walking.step.LastLfoot(Z), walking.step.footHeight);
                    
                    RFoot.refpos(X) = walking.step.Rfoot(X, walking.step.current / 2);
                    RFoot.refpos(Y) = walking.step.Rfoot(Y, walking.step.current / 2);
                    RFoot.refpos(Z) = walking.step.Rfoot(Z, walking.step.current / 2);
                }
            }
        
        else if ((walking.step.current) % 2 == 1) {
                RFoot.refpos(X) = (walking.step.Rfoot(X,(int)((double)(walking.step.current + 1)/2.)) - walking.step.Rfoot(X,(int)((double)(walking.step.current + 1)/2.)-1))*(walking.time.sec-walking.time.SSP_start_time)/walking.time.SSP_time + walking.step.Rfoot(X,(int)((double)(walking.step.current + 1)/2.)-1);
                RFoot.refpos(Y) = (walking.step.Rfoot(Y,(int)((double)(walking.step.current + 1)/2.)) - walking.step.Rfoot(Y,(int)((double)(walking.step.current + 1)/2.)-1))*(walking.time.sec-walking.time.SSP_start_time)/walking.time.SSP_time + walking.step.Rfoot(Y,(int)((double)(walking.step.current + 1)/2.)-1);
                RFoot.refpos(Z) = sinWave((walking.time.sec-walking.time.SSP_start_time), walking.time.SSP_time, walking.step.LastLfoot(Z), walking.step.footHeight);
            
                LFoot.refpos(X) = walking.step.Lfoot(X, walking.step.current / 2);
                LFoot.refpos(Y) = walking.step.Lfoot(Y, walking.step.current / 2);
                LFoot.refpos(Z) = walking.step.Lfoot(Z, walking.step.current / 2);
            }
        }
    }

    else {
        LFoot.refpos(X) = walking.step.LastLfoot(X);
        LFoot.refpos(Y) = walking.step.LastLfoot(Y);
        LFoot.refpos(Z) = walking.step.LastLfoot(Z);

        RFoot.refpos(X) = walking.step.LastRfoot(X);
        RFoot.refpos(Y) = walking.step.LastRfoot(Y);
        RFoot.refpos(Z) = walking.step.LastRfoot(Z);
    }


}


void CKubot::generateFootTraj_ver2() {// 
    Eigen::Vector3f LFoot_Start_pos;
    Eigen::Vector3f LFoot_3;
    Eigen::Vector3f LFoot_4;
    Eigen::Vector3f LFoot_5;
    Eigen::Vector3f LFoot_End_pos;
    Eigen::Vector3f LFoot_pos;

    Eigen::Vector3f RFoot_Start_pos;
    Eigen::Vector3f RFoot_3;
    Eigen::Vector3f RFoot_4;
    Eigen::Vector3f RFoot_5;
    Eigen::Vector3f RFoot_End_pos;
    Eigen::Vector3f RFoot_pos;

    //* DSP
    if(walking.time.sec <= walking.time.SSP_start_time) {
       if((walking.step.current) == 0) {
            LFoot.refpos(X) = walking.step.LastLfoot(X);
            LFoot.refpos(Y) = walking.step.LastLfoot(Y);
            LFoot.refpos(Z) = walking.step.LastLfoot(Z); // Z = 0

            RFoot.refpos(X) = walking.step.LastRfoot(X);
            RFoot.refpos(Y) = walking.step.LastRfoot(Y);
            RFoot.refpos(Z) = walking.step.LastRfoot(Z);
        }
    }

    //* SSP
    else if(walking.time.sec <= walking.time.SSP_end_time) {
        if(walking.step.current < walking.step.total) {
          if(walking.step.start_foot == walking.step.S_L_F){
            if ((walking.step.current) % 2 == 0) {
                if((walking.step.current) == 0) {
                    LFoot_Start_pos << walking.step.LastLfoot(X), walking.step.LastLfoot(Y), walking.step.LastLfoot(Z);
                    LFoot_3 <<  walking.step.LastLfoot(X), \
                                walking.step.LastLfoot(Y), \
                                walking.step.footHeight * pow(2, 8) / 112;
                    LFoot_4 << (walking.step.Lfoot(X,(int)((double)(walking.step.current)/2.)) - walking.step.LastLfoot(X))/2. + walking.step.LastLfoot(X), \
                               (walking.step.Lfoot(Y,(int)((double)(walking.step.current)/2.)) - walking.step.LastLfoot(Y))/2. + walking.step.LastLfoot(Y), \
                                walking.step.LastLfoot(Z);
                    LFoot_5 <<  walking.step.Lfoot(X,(int)((double)(walking.step.current)/2.)), \
                                walking.step.Lfoot(Y,(int)((double)(walking.step.current)/2.)), \
                                walking.step.footHeight * pow(2, 8) / 112;
                    LFoot_End_pos << walking.step.Lfoot(X,(int)((double)(walking.step.current)/2.)), walking.step.Lfoot(Y,(int)((double)(walking.step.current)/2.)), walking.step.LastLfoot(Z);
                    LFoot_pos = genreate_8th_BezierCurve(LFoot_Start_pos, LFoot_3, LFoot_4, LFoot_5, LFoot_End_pos, walking.time.SSP_time, walking.time.sec-walking.time.SSP_start_time);

                    LFoot.refpos(X) = LFoot_pos(X);
                    LFoot.refpos(Y) = LFoot_pos(Y);
                    LFoot.refpos(Z) = LFoot_pos(Z);
                    LFoot.refpos(Yaw)  = cosWave(walking.time.sec-walking.time.SSP_start_time, walking.time.SSP_time, walking.step.LastLfoot(Yaw), walking.step.Lfoot(Yaw,(double)(walking.step.current)/2.));

                    RFoot.refpos(X) = walking.step.LastRfoot(X);
                    RFoot.refpos(Y) = walking.step.LastRfoot(Y);
                    RFoot.refpos(Z) = walking.step.LastRfoot(Z);
                    RFoot.refpos(Yaw) = walking.step.LastRfoot(Yaw);
                }
                else {
                    LFoot_Start_pos << walking.step.Lfoot(X,(int)((double)(walking.step.current)/2.)-1), walking.step.Lfoot(Y,(int)((double)(walking.step.current)/2.)-1),  walking.step.LastRfoot(Z);
                    LFoot_3 <<  walking.step.Lfoot(X,(int)((double)(walking.step.current)/2.)-1), \
                                walking.step.Lfoot(Y,(int)((double)(walking.step.current)/2.)-1), \
                                walking.step.footHeight * pow(2, 8) / 112;
                    LFoot_4 <<((walking.step.Lfoot(X,(int)((double)(walking.step.current)/2.)) - walking.step.Lfoot(X,(int)((double)(walking.step.current)/2.)-1))/2. + walking.step.Lfoot(X,(int)((double)(walking.step.current)/2.)-1)), \
                              ((walking.step.Lfoot(Y,(int)((double)(walking.step.current)/2.)) - walking.step.Lfoot(Y,(int)((double)(walking.step.current)/2.)-1))/2. + walking.step.Lfoot(Y,(int)((double)(walking.step.current)/2.)-1)), \
                                walking.step.LastLfoot(Z);
                    LFoot_5 <<  walking.step.Lfoot(X,(int)((double)(walking.step.current)/2.)), \
                                walking.step.Lfoot(Y,(int)((double)(walking.step.current)/2.)), \
                                walking.step.footHeight * pow(2, 8) / 112;
                    LFoot_End_pos << walking.step.Lfoot(X,(int)((double)(walking.step.current)/2.)), walking.step.Lfoot(Y,(int)((double)(walking.step.current)/2.)), walking.step.LastLfoot(Z);
                    LFoot_pos = genreate_8th_BezierCurve(LFoot_Start_pos, LFoot_3, LFoot_4, LFoot_5, LFoot_End_pos, walking.time.SSP_time, walking.time.sec-walking.time.SSP_start_time);

                    LFoot.refpos(X) = LFoot_pos(X);
                    LFoot.refpos(Y) = LFoot_pos(Y);
                    LFoot.refpos(Z) = LFoot_pos(Z);
                    LFoot.refpos(Yaw)  = cosWave(walking.time.sec-walking.time.SSP_start_time, walking.time.SSP_time, walking.step.LastLfoot(Yaw), walking.step.Lfoot(Yaw,(double)(walking.step.current)/2.));

                    RFoot.refpos(X) = walking.step.Rfoot(X, walking.step.current/2.);
                    RFoot.refpos(Y) = walking.step.Rfoot(Y, walking.step.current/2.);
                    RFoot.refpos(Z) = walking.step.Rfoot(Z, walking.step.current/2.);
                    RFoot.refpos(Yaw) = walking.step.Rfoot(Yaw, walking.step.current/2.);
                }
            }

        else if ((walking.step.current) % 2 == 1) {
                RFoot_Start_pos <<  walking.step.Rfoot(X,(int)((double)(walking.step.current+1)/2.)-1),  walking.step.Rfoot(Y,(int)((double)(walking.step.current+1)/2.)-1), walking.step.LastRfoot(Z);
                RFoot_3 <<  walking.step.Rfoot(X,(int)((double)(walking.step.current+1)/2.)-1), \
                            walking.step.Rfoot(Y,(int)((double)(walking.step.current+1)/2.)-1), \
                            walking.step.footHeight * pow(2, 8) / 112;
                RFoot_4 <<((walking.step.Rfoot(X,(int)((double)(walking.step.current+1)/2.)) - walking.step.Rfoot(X,(int)((double)(walking.step.current+1)/2.)-1))/2. + walking.step.Rfoot(X,(int)((double)(walking.step.current+1)/2.)-1)), \
                          ((walking.step.Rfoot(Y,(int)((double)(walking.step.current+1)/2.)) - walking.step.Rfoot(Y,(int)((double)(walking.step.current+1)/2.)-1))/2. + walking.step.Rfoot(Y,(int)((double)(walking.step.current+1)/2.)-1)), \
                            walking.step.LastRfoot(Z);
                RFoot_5 <<  walking.step.Rfoot(X,(int)((double)(walking.step.current+1)/2.)), \
                            walking.step.Rfoot(Y,(int)((double)(walking.step.current+1)/2.)), \
                            walking.step.footHeight * pow(2, 8) / 112;
                RFoot_End_pos << walking.step.Rfoot(X,(int)((double)(walking.step.current+1)/2.)), walking.step.Rfoot(Y,(int)((double)(walking.step.current+1)/2.)), walking.step.LastRfoot(Z);
                RFoot_pos = genreate_8th_BezierCurve(RFoot_Start_pos, RFoot_3, RFoot_4, RFoot_5, RFoot_End_pos, walking.time.SSP_time, walking.time.sec-walking.time.SSP_start_time);

                RFoot.refpos(X) = RFoot_pos(X);
                RFoot.refpos(Y) = RFoot_pos(Y);
                RFoot.refpos(Z) = RFoot_pos(Z);
                RFoot.refpos(Yaw)  = cosWave(walking.time.sec-walking.time.SSP_start_time, walking.time.SSP_time, walking.step.LastRfoot(Yaw), walking.step.Rfoot(Yaw,(double)(walking.step.current+1)/2.));

                LFoot.refpos(X) = walking.step.Lfoot(X, (int)((double)(walking.step.current/2.)));
                LFoot.refpos(Y) = walking.step.Lfoot(Y, (int)((double)(walking.step.current/2.)));
                LFoot.refpos(Z) = walking.step.Lfoot(Z, (int)((double)(walking.step.current/2.)));
                LFoot.refpos(Yaw) = walking.step.Lfoot(Yaw, (int)((double)(walking.step.current/2.)));
            }
          }
          if(walking.step.start_foot == walking.step.S_R_F){
            if ((walking.step.current) % 2 == 0) {
                if((walking.step.current) == 0) {
                    RFoot_Start_pos << walking.step.LastRfoot(X), walking.step.LastRfoot(Y), walking.step.LastRfoot(Z);
                    RFoot_3 <<  walking.step.LastRfoot(X), \
                                walking.step.LastRfoot(Y), \
                                walking.step.footHeight * pow(2, 8) / 112;
                    RFoot_4 << (walking.step.Rfoot(X,(int)((double)(walking.step.current)/2.)) - walking.step.LastRfoot(X))/2. + walking.step.LastRfoot(X), \
                                (walking.step.Rfoot(Y,(int)((double)(walking.step.current)/2.)) - walking.step.LastRfoot(Y))/2. + walking.step.LastRfoot(Y), \
                                 walking.step.LastRfoot(Z);
                    RFoot_5 <<  walking.step.Rfoot(X,(int)((double)(walking.step.current)/2.)), \
                                walking.step.Rfoot(Y,(int)((double)(walking.step.current)/2.)), \
                                walking.step.footHeight * pow(2, 8) / 112;
                    RFoot_End_pos << walking.step.Rfoot(X,(int)((double)(walking.step.current)/2.)), walking.step.Rfoot(Y,(int)((double)(walking.step.current)/2.)), walking.step.LastRfoot(Z);
                    RFoot_pos = genreate_8th_BezierCurve(RFoot_Start_pos, RFoot_3, RFoot_4, RFoot_5, RFoot_End_pos, walking.time.SSP_time, walking.time.sec-walking.time.SSP_start_time);

                    RFoot.refpos(X) = RFoot_pos(X);
                    RFoot.refpos(Y) = RFoot_pos(Y);
                    RFoot.refpos(Z) = RFoot_pos(Z);
                    RFoot.refpos(Yaw)  = cosWave(walking.time.sec-walking.time.SSP_start_time, walking.time.SSP_time, walking.step.LastRfoot(Yaw), walking.step.Rfoot(Yaw,(double)(walking.step.current)/2.));

                    LFoot.refpos(X) = walking.step.LastLfoot(X);
                    LFoot.refpos(Y) = walking.step.LastLfoot(Y);
                    LFoot.refpos(Z) = walking.step.LastLfoot(Z);
                    LFoot.refpos(Yaw) = walking.step.LastLfoot(Yaw);
                }
                else {
                    RFoot_Start_pos << walking.step.Rfoot(X,(int)((double)(walking.step.current)/2.)-1), walking.step.Rfoot(Y,(int)((double)(walking.step.current)/2.)-1),  walking.step.LastRfoot(Z);
                    RFoot_3 <<  walking.step.Rfoot(X,(int)((double)(walking.step.current)/2.)-1), \
                                walking.step.Rfoot(Y,(int)((double)(walking.step.current)/2.)-1), \
                                walking.step.footHeight * pow(2, 8) / 112;
                    RFoot_4 <<((walking.step.Rfoot(X,(int)((double)(walking.step.current)/2.)) - walking.step.Rfoot(X,(int)((double)(walking.step.current)/2.)-1))/2. + walking.step.Rfoot(X,(int)((double)(walking.step.current)/2.)-1)), \
                              ((walking.step.Rfoot(Y,(int)((double)(walking.step.current)/2.)) - walking.step.Rfoot(Y,(int)((double)(walking.step.current)/2.)-1))/2. + walking.step.Rfoot(Y,(int)((double)(walking.step.current)/2.)-1)), \
                                walking.step.LastRfoot(Z);
                    RFoot_5 <<  walking.step.Rfoot(X,(int)((double)(walking.step.current)/2.)), \
                                walking.step.Rfoot(Y,(int)((double)(walking.step.current)/2.)), \
                                walking.step.footHeight * pow(2, 8) / 112;
                    RFoot_End_pos << walking.step.Rfoot(X,(int)((double)(walking.step.current)/2.)), walking.step.Rfoot(Y,(int)((double)(walking.step.current)/2.)), walking.step.LastRfoot(Z);
                    RFoot_pos = genreate_8th_BezierCurve(RFoot_Start_pos, RFoot_3, RFoot_4, RFoot_5, RFoot_End_pos, walking.time.SSP_time, walking.time.sec-walking.time.SSP_start_time);

                    RFoot.refpos(X) = RFoot_pos(X);
                    RFoot.refpos(Y) = RFoot_pos(Y);
                    RFoot.refpos(Z) = RFoot_pos(Z);
                    RFoot.refpos(Yaw)  = cosWave(walking.time.sec-walking.time.SSP_start_time, walking.time.SSP_time, walking.step.LastRfoot(Yaw), walking.step.Rfoot(Yaw,(double)(walking.step.current)/2.));

                    LFoot.refpos(X) = walking.step.Lfoot(X, walking.step.current/2.);
                    LFoot.refpos(Y) = walking.step.Lfoot(Y, walking.step.current/2.);
                    LFoot.refpos(Z) = walking.step.Lfoot(Z, walking.step.current/2.);
                    LFoot.refpos(Yaw) = walking.step.Lfoot(Yaw, walking.step.current/2.);
                }
            }

        else if ((walking.step.current) % 2 == 1) {
                LFoot_Start_pos <<  walking.step.Lfoot(X,(int)((double)(walking.step.current+1)/2.)-1),  walking.step.Lfoot(Y,(int)((double)(walking.step.current+1)/2.)-1), walking.step.LastLfoot(Z);
                LFoot_3 <<  walking.step.Lfoot(X,(int)((double)(walking.step.current+1)/2.)-1), \
                            walking.step.Lfoot(Y,(int)((double)(walking.step.current+1)/2.)-1), \
                            walking.step.footHeight * pow(2, 8) / 112;
                LFoot_4 <<((walking.step.Lfoot(X,(int)((double)(walking.step.current+1)/2.)) - walking.step.Lfoot(X,(int)((double)(walking.step.current+1)/2.)-1))/2. + walking.step.Lfoot(X,(int)((double)(walking.step.current+1)/2.)-1)), \
                          ((walking.step.Lfoot(Y,(int)((double)(walking.step.current+1)/2.)) - walking.step.Lfoot(Y,(int)((double)(walking.step.current+1)/2.)-1))/2. + walking.step.Lfoot(Y,(int)((double)(walking.step.current+1)/2.)-1)), \
                            walking.step.LastRfoot(Z);
                LFoot_5 <<  walking.step.Lfoot(X,(int)((double)(walking.step.current+1)/2.)), \
                            walking.step.Lfoot(Y,(int)((double)(walking.step.current+1)/2.)), \
                            walking.step.footHeight * pow(2, 8) / 112;
                LFoot_End_pos << walking.step.Lfoot(X,(int)((double)(walking.step.current+1)/2.)), walking.step.Lfoot(Y,(int)((double)(walking.step.current+1)/2.)), walking.step.LastLfoot(Z);
                LFoot_pos = genreate_8th_BezierCurve(LFoot_Start_pos, LFoot_3, LFoot_4, LFoot_5, LFoot_End_pos, walking.time.SSP_time, walking.time.sec-walking.time.SSP_start_time);

                LFoot.refpos(X) = LFoot_pos(X);
                LFoot.refpos(Y) = LFoot_pos(Y);
                LFoot.refpos(Z) = LFoot_pos(Z);
                LFoot.refpos(Yaw)  = cosWave(walking.time.sec-walking.time.SSP_start_time, walking.time.SSP_time, walking.step.LastLfoot(Yaw), walking.step.Lfoot(Yaw,(double)(walking.step.current+1)/2.));

                RFoot.refpos(X) = walking.step.Rfoot(X, (int)((double)(walking.step.current/2.)));
                RFoot.refpos(Y) = walking.step.Rfoot(Y, (int)((double)(walking.step.current/2.)));
                RFoot.refpos(Z) = walking.step.Rfoot(Z, (int)((double)(walking.step.current/2.)));
                RFoot.refpos(Yaw) = walking.step.Rfoot(Yaw, (int)((double)(walking.step.current/2.)));
            }
          }
        }
    }

    else {
      if(walking.step.start_foot == walking.step.S_L_F){
        if((walking.step.current) % 2 == 0){
          LFoot.refpos(X) = walking.step.Lfoot(X, walking.step.current/2.);
          LFoot.refpos(Y) = walking.step.Lfoot(Y, walking.step.current/2.);
          LFoot.refpos(Z) = walking.step.Lfoot(Z, walking.step.current/2.);

          RFoot.refpos(X) = walking.step.Rfoot(X, (walking.step.current)/2.);
          RFoot.refpos(Y) = walking.step.Rfoot(Y, (walking.step.current)/2.);
          RFoot.refpos(Z) = walking.step.Rfoot(Z, (walking.step.current)/2.);
          }

        else{
          LFoot.refpos(X) = walking.step.Lfoot(X, walking.step.current/2.);
          LFoot.refpos(Y) = walking.step.Lfoot(Y, walking.step.current/2.);
          LFoot.refpos(Z) = walking.step.Lfoot(Z, walking.step.current/2.);

          RFoot.refpos(X) = walking.step.Rfoot(X, (walking.step.current+1)/2.);
          RFoot.refpos(Y) = walking.step.Rfoot(Y, (walking.step.current+1)/2.);
          RFoot.refpos(Z) = walking.step.Rfoot(Z, (walking.step.current+1)/2.);
        }
      }

      if(walking.step.start_foot == walking.step.S_R_F){
        if((walking.step.current) % 2 == 0){
          LFoot.refpos(X) = walking.step.Lfoot(X, walking.step.current/2.);
          LFoot.refpos(Y) = walking.step.Lfoot(Y, walking.step.current/2.);
          LFoot.refpos(Z) = walking.step.Lfoot(Z, walking.step.current/2.);

          RFoot.refpos(X) = walking.step.Rfoot(X, (walking.step.current)/2.);
          RFoot.refpos(Y) = walking.step.Rfoot(Y, (walking.step.current)/2.);
          RFoot.refpos(Z) = walking.step.Rfoot(Z, (walking.step.current)/2.);
          }

        else{
          LFoot.refpos(X) = walking.step.Lfoot(X, (walking.step.current+1)/2. );
          LFoot.refpos(Y) = walking.step.Lfoot(Y, (walking.step.current+1)/2. );
          LFoot.refpos(Z) = walking.step.Lfoot(Z, (walking.step.current+1)/2. );

          RFoot.refpos(X) = walking.step.Rfoot(X, walking.step.current/2.);
          RFoot.refpos(Y) = walking.step.Rfoot(Y, walking.step.current/2.);
          RFoot.refpos(Z) = walking.step.Rfoot(Z, walking.step.current/2.);
        }
      }

    }

}


void CKubot::zmpPreviewControl()
{
    /* zmpPreviewControl
     * output : trajectory by zmp preview control --> zmp.previewControl.X.CoM, zmp.previewControl.Y.CoM [Crobot class's Variable]
     *
     * Before run function
     * [First, Run once Crobot class's Function initializeSystemID when starting program]
     * [Second, Run Crobot class's Function setZmpPreview before running this function]
     * [Third, Run Crobot class's Function zmp plan(ex setZmpSwapPlan) before running this function]
     */

    if(zmp.previewControl.count < zmp.previewControl.RefTotalTrajSize - zmp.previewControl.PreviewTimeSize) {

        zmp.previewControl.Y.state = zmp.previewControl.Y.new_state;
        zmp.previewControl.X.state = zmp.previewControl.X.new_state;

        zmp.previewControl.Y.E = zmp.previewControl.Y.old_zmp - zmp.previewControl.Y.ref(zmp.previewControl.count);
        zmp.previewControl.X.E = zmp.previewControl.X.old_zmp - zmp.previewControl.X.ref(zmp.previewControl.count);

//        std::cout<<"zmp.previewControl.Y.E :"<<zmp.previewControl.Y.E<<std::endl;
//        std::cout<<"zmp.previewControl.X.E :"<<zmp.previewControl.X.E<<std::endl;

        zmp.previewControl.Y.m_ref = zmp.previewControl.Y.ref(zmp.previewControl.count);
        zmp.previewControl.X.m_ref = zmp.previewControl.X.ref(zmp.previewControl.count);

        zmp.previewControl.Y.sum_E = zmp.previewControl.Y.sum_E + zmp.previewControl.Y.E;
        zmp.previewControl.X.sum_E = zmp.previewControl.X.sum_E + zmp.previewControl.X.E;

        for (int pre_cnt = 0; pre_cnt < zmp.previewControl.PreviewTimeSize; pre_cnt++) {
            zmp.previewControl.Y.sum_P = zmp.previewControl.Y.sum_P + zmp.previewControl.G_P(pre_cnt) * zmp.previewControl.Y.ref(zmp.previewControl.count + pre_cnt);
            zmp.previewControl.X.sum_P = zmp.previewControl.X.sum_P + zmp.previewControl.G_P(pre_cnt) * zmp.previewControl.X.ref(zmp.previewControl.count + pre_cnt);
        }

        zmp.previewControl.X.U = -zmp.previewControl.G_I * zmp.previewControl.X.sum_E - (zmp.previewControl.G_X * zmp.previewControl.X.state)(0, 0) - zmp.previewControl.X.sum_P;
        zmp.previewControl.Y.U = -zmp.previewControl.G_I * zmp.previewControl.Y.sum_E - (zmp.previewControl.G_X * zmp.previewControl.Y.state)(0, 0) - zmp.previewControl.Y.sum_P;

        zmp.previewControl.Y.new_state = zmp.previewControl.A * zmp.previewControl.Y.state + zmp.previewControl.B * zmp.previewControl.Y.U;
        zmp.previewControl.X.new_state = zmp.previewControl.A * zmp.previewControl.X.state + zmp.previewControl.B * zmp.previewControl.X.U;

        zmp.previewControl.Y.old_zmp = (zmp.previewControl.C * zmp.previewControl.Y.state)(0, 0);
        zmp.previewControl.X.old_zmp = (zmp.previewControl.C * zmp.previewControl.X.state)(0, 0);

        zmp.previewControl.Y.CoM = zmp.previewControl.Y.new_state(0);
        zmp.previewControl.Y.dCoM = zmp.previewControl.Y.new_state(1);
        zmp.previewControl.X.CoM = zmp.previewControl.X.new_state(0);
        zmp.previewControl.X.dCoM = zmp.previewControl.X.new_state(1);

        zmp.previewControl.Y.sum_P = 0;
        zmp.previewControl.X.sum_P = 0;

    }
    else {

    }
}

double CKubot::cosWave(double time, double half_period, double init_val, double double_amp) {
    return ((double_amp - init_val) / 2)*(1 - cos(PI / half_period * time)) + init_val;
}

double CKubot::sinWave(double time, double half_period, double init_val, double amp) {
    return amp * sin(PI / half_period * time) + init_val;
}

int CKubot::factorial(int n) {
    if(n < 0) {
        std::cout << "n is a negative value" << std::endl;
    }
    else if (n == 0) {
        return n = 1;
    }
    else {
        return n = n*factorial(n-1);
    }
}

double CKubot::compensate(double time, double period, double ratio, double min_val, double max_val) {
    double constant_time = ratio * period;
    double slope_time = 0.5 * (1 - ratio) * period;

    if (time < slope_time) {
        return cosWave(time, slope_time, min_val, max_val);
    }
    else if (time >= slope_time and time < slope_time + constant_time) {
        return min_val + max_val;
    }
    else if (time >= slope_time + constant_time and time < period) {
        return cosWave(time - slope_time - constant_time, slope_time, max_val, min_val);
    }
    else {
        return min_val;
    }
}


Eigen::Vector3f CKubot::genreate_8th_BezierCurve(Eigen::Vector3f point_0_1_2, Eigen::Vector3f point_3, Eigen::Vector3f point_4, Eigen::Vector3f point_5, Eigen::Vector3f point_6_7_8, double period, double time) {
    double t = time/period;
    Eigen::Vector3f output_pos = VectorXf::Zero(3);
    if (time <= period) {
        t = time/period;
    }
    else {
        t = 1.0;
    }
//    std::cout << "t="<<t<< std::endl;

    Eigen::Vector3f B;
    VectorXf b(9);

    Eigen::Vector3f P0, P1, P2, P3, P4, P5, P6, P7, P8;

    P0 = point_0_1_2;
    P1 = P0;
    P2 = P1;
    // Set start vel & acc to zero.

    P3 = point_3;
    P4 = point_4;
    P5 = point_5;

    P6 = point_6_7_8;
    P7 = P6;
    P8 = P7;
    // Set end vel & acc to zero.

    for(int i = 0; i < 9; i++) {
        b(i) = (factorial(8)/(factorial(i)*factorial(8-i)))*pow(1-t,8-i)*pow(t,i);
    // std::cout << "b("<<i<<")"<<b(i)<<" (factorial(8)/(factorial(i)*factorial(8-i))) " <<(factorial(8)/(factorial(i)*factorial(8-i))) <<std::endl;
    }

    B = b(0)*P0 + b(1)*P1 + b(2)*P2 + b(3)*P3 + b(4)*P4 + b(5)*P5 + b(6)*P6 + b(7)*P7 + b(8)*P8;
    // std::cout << "B="<<B<< std::endl;

    output_pos = B;

    return output_pos;
}
