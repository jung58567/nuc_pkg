#include "kubot.h"
#include "CKubot.h"
#include "kinematics.h"

#include <iostream>
#include <fstream>
#include <string>

#include "e2box_imu_9dofv4.h"
#include "main.h"
#include "t_serial.h"

class e2box_imu_9dofv4 m_e2box_imu;
class IMU imu;
using namespace std;

DXL dxl("/dev/ttyDXL",2000000);
dxl_controller DXL_Controller_;
Kinematics KM;
CKubot C;

#define onesizeSec 100
#define NSEC_PER_SEC 1000000000     // 10^9
#define EC_TIMEOUTMON 500

enum EndPointAxis {
    X = 0, Y, Z, Roll, Pitch, Yaw
};

typedef struct {
    VectorXd BA, LF, RF;
} GLOBAL;

GLOBAL Global;

typedef struct {
    VectorXd init;
    VectorXd cal;
    VectorXd fin;
    VectorXd pre;
} JOINT;

typedef struct {
    JOINT R;
    JOINT L;
} JointVariables;

JointVariables q;

VectorXi jointdirection_L(6), jointdirection_R(6);
// double SL, RL, FH; 

unsigned int cycle_ns = 1000000;    // 10^6ns = 1ms

double present_pos[12];
int converted_pos[12];

int cnt = 0;
double t = 0.0;
double T = 1;
double thread_t=0.0;

double initialize_ready_time=2.0;
double tt = 0.0;
int cnt3=0;

int cnt2 =0;
double thread_t2[7][1000]={0,};//0==cnt, 1==Euler, 2==Ang_vel, 3==thread_t 4==

int flag_s = 0;

bool thread_is_running = true;
bool initialize2 =true;

//***************Xenomai time variables***************//

RT_TASK RT_task1,RT_task2;
RTIME now, previous;    // System Test time
RTIME now1, previous1;
RTIME now2, previous2;  // Thread 1 cycle time

double t_s,t_imu_s,t_pub_s,t_imu_d,t_pub_d,t_l,t_s_r;

void save_file();
void serial_task(void* arg);
//void serial_task2(void* arg);
void catch_signal(int sig);
void dxl_switch_Callback(const std_msgs::String::ConstPtr& msg) {
     ROS_INFO("data: [%s]", msg->data.c_str());
     if(msg->data == "tq_off") {
        for(int i=1; i<20; i++) {
            //dxl.Torque_Off(i);
        }
     }
     else if(msg->data == "Ready") {
        while(1) {
            // if (t <= C.zmp.previewControl.PreviewReadyTime) {
            //     // init_pose();
                break;
            // }
            // now1 = rt_timer_read();
            //  thread_t += (long)(now1-previous1)/cycle_ns;
            // // cout << "Thread_time : " << int(t/cycle_ns) << "s  dt : " << (long)(now1-previous1)/1000 << "ns" << endl;
            // previous1 = now1;
        }
    }
    return;
}
void callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received message: %s", msg->data.c_str());
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "kubot_node");
    ros::NodeHandle nh;
//imu때문에 추가한 코드@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    std::string port;
    int baudrate;

    nh.param<std::string>("port", port, "/dev/ttyUSB1");
    nh.param("baudrate", baudrate, 115200);

        if(!m_e2box_imu.serial.Open(const_cast<char*>(port.c_str()), baudrate)){
        cout << "device is not opened! " << endl;
    }
/////////////////////////////////////////////////////////
    ROS_INFO("run main.cpp");
    ros::Subscriber sub = nh.subscribe("dxl_switch", 1000, dxl_switch_Callback);
    ros::Subscriber sub2 = nh.subscribe("bounding_box_pub", 1000, callback);
    // ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    signal(SIGINT, catch_signal);
    signal(SIGTERM,catch_signal);

    //rt_task_create(&RT_task2, "serial_task2", 0, 99, 0);
    //rt_task_start(&RT_task2, &serial_task2, NULL);
    Load();

    //(PKW)
    rt_task_create(&RT_task1, "serial_task", 0, 99, 0);
    rt_task_start(&RT_task1, &serial_task, NULL);

    while (ros::ok())
    {
        // std_msgs::String msg;
        // msg.data = "I said hello";
        
        // chatter_pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    thread_is_running = false;
}

void save_file(){
    string line;
    ofstream file;
    file.open("time.txt",ios_base::app);
    if(file.is_open()) {
        // cout<<"save !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!="<<endl;
        // cout<<cnt2<<endl;
		file <<"count"<<","<<"Euler"<<","<<"ang_vel"<<","<<"task_thread"<<","<<"dt_imu"<<","<<"dt_pub"<<","<<"dt_thread"<<"\n";
        for(int k=1;k<=cnt2;k++)
        {
            for(int u=1;u<=7;u++){  
            file <<thread_t2[u-1][k-1]<<",";  
            }
        file<<"\n";
        }
        cout<<"save done"<<endl;
		file.close();
	}
    else {
		cout << "error" << endl;	
	}
}

void initializeKubot() {

    Global.BA = VectorXd::Zero(6);
    Global.LF = VectorXd::Zero(6);
    Global.RF = VectorXd::Zero(6);

    Global.BA(Z) = 0.308;// 0.308;
    Global.LF(Y) = 0.05;
    Global.RF(Y) = -0.05;

    q.R.init = VectorXd::Zero(6);
    q.L.init = VectorXd::Zero(6);

    q.R.cal = KM.Geometric_IK_R(Global.BA, Global.RF);
    q.L.cal = KM.Geometric_IK_L(Global.BA, Global.LF);

}

void Load()
{
  //  ROS_INFO("setting dynamixel ID...");
//    dxl.serial_port = dxl.Initialize();
//
//    dxl.motor_num = 12;
//
////    ROS_INFO("<dxl.motor_num : %d", dxl.motor_num);
//
//    for(int i = 0; i<dxl.motor_num; i++){
//        dxl.motor[i] = i+1;
////        ROS_INFO("dxl_motor_id = %d", dxl.motor[i]);
//        dxl.Torque_On(dxl.motor[i]);
//    }
//    usleep(10000);
//
//    //unsigned char read_buf[SIZE] = {0};
//    //int num_bytes = read(dxl.serial_port, read_buf, sizeof(read_buf));
//
//    for (int i = 0; i < 3; i++){
//        dxl.sync_read(dxl.serial_port, dxl.motor);
//        usleep(200000);
//    }
//
    
    int encoder_read[12];
    DXL_Controller_.Initialize();
    DXL_Controller_.Read_Dxl_Encoder_Once(encoder_read);//처음에 모터값을 읽어줌

    for(int i = 0; i < 12; i++) { // 12번 반복
    //    ROS_INFO("dxl.present_position[%d]=%d", i+1, dxl.present_position[i]);
        present_pos[i] = convertw4095ToRad(encoder_read[i]);//처음 읽어준 모터값을 라디안 값으로 바꿔서 출력함
     ROS_INFO("present_pos[%d]=%f", i+1, present_pos[i]);
    }

    jointdirection_R << -1,-1,-1,-1, 1, 1;
    jointdirection_L << -1,-1, 1, 1,-1, 1;

    initializeKubot();//쿠봇 다리 펴게 해주는 함수

    C.initializeCKubot();
    C.setWalkingStep(10, C.walking.step.S_R_F, 0.03, 0, 0, 0.04);
    C.setWalkingTime(0.6, 0.4);
    C.setZmpPreview(1.5);
    C.initializeZmpTraj();
    C.FootPlanner();
    C.generateZmpTraj(C.walking.step.Lfoot, C.walking.step.Rfoot); 
}



// void serial_task2(void* arg)
// {
//     rt_task_set_periodic(NULL, TM_NOW, cycle_ns * 10);

//     while (initialize2)
//     {
//         rt_task_wait_period(NULL);
//     if(tt <= initialize_ready_time) {
//             initializeKubot();
//         }

//         for(int i = 0; i < 6; i++) {
//             converted_pos[2*i]   = convertRadTow4095(jointdirection_R(i)*q.R.pre(i));
//         }
//         for(int i = 0; i < 6; i++) {
//             converted_pos[2*i+1] = convertRadTow4095(jointdirection_L(i)*q.L.pre(i));
//         }

//     DXL_Controller_.Sync_Position_command_TxOnly(converted_pos);
//     tt = (double)cnt3 / (double)onesecSize;
//     cnt3++;
//     }

//     initialize2=false;
// }







void serial_task(void* arg)
{
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns * 10);

    int j=0;
    now1 = rt_timer_read();
    previous1 = now1;
    t_s_r = now1;
    t_imu_d=now1;
    t_pub_d=now1;
    t_l=now1;

    while (thread_is_running)
    {
        //cout<<"pre_pos= "<<dxl.present_position[0]<<endl;
        rt_task_wait_period(NULL);
        //imu 데이터 받기 위해 추가로 작성한 코드@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        ////////////////////////////////////////////////////////////////////////////////////////
        now1 = rt_timer_read();
        //t_s = now1;        
        t_s = rt_timer_read();
        //t_s_r = t_s;
        //thread_t += (long)(now1-previous1)/cycle_ns;
        thread_t = (long)(now1-previous1)/1000000.0;
        previous1 = now1;
    

    t_imu_s = rt_timer_read();
    imu.OnReceiveImu();
    t_imu_d = rt_timer_read();
    // thread_t2 = (long)(t_imu_d-t_imu_s)/1000000.0;
    // previous2 = t_imu_s;
    t_pub_s = rt_timer_read();
    imu.publishImuData();
    t_pub_d = rt_timer_read();

    //std::cout<<"imu= "<<m_e2box_imu.Euler[0]<<std::endl;
    std::cout<<"imu= "<<m_e2box_imu.Euler[0]<<std::endl;

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    //여기까지@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        if(t <= C.zmp.previewControl.PreviewReadyTime) {
            init_pose();
        }
        else if(t < C.zmp.previewControl.RefTotalTime - 2*C.zmp.previewControl.PreviewTime) {
             process();
        }
        //else if()
	    else {
              q.L.pre = KM.Geometric_IK_L(Global.BA, Global.LF);
              q.R.pre = KM.Geometric_IK_R(Global.BA, Global.RF);
          }
         //kick_process();
        // now1 = rt_timer_read();
        // time += 0.01; // (double)(now1-previous1)/cycle_ns/1000;
        // cout << "Thread_time : " << t << "s  dt : " << (double)(now1-previous1)/1000 << "ns" << endl;




        for(int i = 0; i < 6; i++) {
            converted_pos[2*i]   = convertRadTow4095(jointdirection_R(i)*q.R.pre(i));
        }
        for(int i = 0; i < 6; i++) {
            converted_pos[2*i+1] = convertRadTow4095(jointdirection_L(i)*q.L.pre(i));
        }
        ////////////여기까지 주석처리 함

        t = (double)cnt / (double)onesecSize;//단위가 s이다(그래서 100으로 나눠줬네)
        C.walking.time.sec = t - C.zmp.previewControl.PreviewReadyTime - C.walking.step.current * C.walking.time.periodTime;
        //C.generateFootTraj();-------------> 이건 예전버전이라 주석 달아놓음
        C.generateFootTraj_ver2();
        C.zmp.previewControl.count = cnt;
        C.zmpPreviewControl();

       //if (t < C.zmp.previewControl.RefTotalTime - 2*C.zmp.previewControl.PreviewTime)
        if (t < C.zmp.previewControl.RefTotalTime - 2*C.zmp.previewControl.PreviewTime) {
            if(C.walking.time.sec >= C.walking.time.SSP_start_time and C.walking.time.sec <= C.walking.time.SSP_end_time) {
                if((C.walking.step.current % 2) == 0) {
                    converted_pos[2] = converted_pos[2] + C.compensate(C.walking.time.sec - C.walking.time.SSP_start_time, C.walking.time.SSP_time, 0.9, 0, 100);
                    //converted_pos[3] = converted_pos[3] - C.trapezoidal(C.walking.time.sec - C.walking.time.SSP_start_time, C.walking.time.SSP_time, 0.4, 0, 16);
                }
                else if((C.walking.step.current % 2) == 1) {
                    converted_pos[3] = converted_pos[3] - C.compensate(C.walking.time.sec - C.walking.time.SSP_start_time, C.walking.time.SSP_time, 0.9, 0, 100);
                    //converted_pos[2] = converted_pos[2] + C.trapezoidal(C.walking.time.sec - C.walking.time.SSP_start_time, C.walking.time.SSP_time, 0.4, 0, 16);
                }
            }
        }

//        if (t > 3*T and 7*T >= t) {
//            converted_pos[3] = converted_pos[3] - C.compensate( t-3*T , 4*T, 0.8, 0, 190);
//        }

        if(C.walking.time.sec >= C.walking.time.periodTime) {
            C.walking.time.sec = 0;
            C.walking.step.current++;
        }

        DXL_Controller_.Sync_Position_command_TxOnly(converted_pos);

    cnt++;
   
        t_l = rt_timer_read();

        thread_t2[0][cnt2]=cnt2;
        thread_t2[1][cnt2]=m_e2box_imu.Euler[0];
        thread_t2[2][cnt2]=m_e2box_imu.m_dAngRate[0];
        thread_t2[3][cnt2]=(long)(t_l - t_s)/1000000.0; //dt_task_thread_time
        thread_t2[4][cnt2]=(long)(t_imu_d - t_imu_s)/1000000.0; // dt_imu_recive
        thread_t2[5][cnt2]=(long)(t_pub_d - t_pub_s)/1000000.0; // dt_imu_pub
        thread_t2[6][cnt2]=(thread_t); //dt_thread


         if(cnt2==800)
        {
             cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
             thread_is_running=false;
             save_file();
        }
        cnt2++;       
    }
    return;
}

void init_pose() {
    Global.BA(X) = C.zmp.previewControl.X.CoM;
    Global.BA(Y) = C.zmp.previewControl.Y.CoM;
    Global.BA(Z) = C.cosWave(t, C.zmp.previewControl.PreviewReadyTime, 0.308, 0.25);
    //coswave를 넣어준 이유 아마도 z좌표는 preview control을 해주지 않기 때문에 연속적인 궤적이 나오지 않는다 따라서 coswave를 통해 궤적을 만들어 준다.

    q.R.pre = KM.Geometric_IK_R(Global.BA, Global.RF);
    q.L.pre = KM.Geometric_IK_L(Global.BA, Global.LF);
}

void process() {
              Global.BA(X) = C.zmp.previewControl.X.CoM;
              Global.BA(Y) = C.zmp.previewControl.Y.CoM;
              Global.BA(Z) = 0.25;
              if (C.walking.step.start_foot == C.walking.step.S_L_F){
                  Global.BA(Yaw) = C.LFoot.refpos(Yaw);
              }
              else if (C.walking.step.start_foot == C.walking.step.S_R_F){
                  Global.BA(Yaw) = C.RFoot.refpos(Yaw);
              }

              Global.LF(X) = C.LFoot.refpos(X);
              Global.LF(Y) = C.LFoot.refpos(Y);
              Global.LF(Z) = C.LFoot.refpos(Z);
              Global.LF(Yaw) = C.LFoot.refpos(Yaw);

              Global.RF(X) = C.RFoot.refpos(X);
              Global.RF(Y) = C.RFoot.refpos(Y);
              Global.RF(Z) = C.RFoot.refpos(Z);
              Global.RF(Yaw) = C.RFoot.refpos(Yaw);

              q.L.pre = KM.Geometric_IK_L(Global.BA, Global.LF);
              q.R.pre = KM.Geometric_IK_R(Global.BA, Global.RF);

              if (C.walking.time.sec >= C.walking.time.periodTime) {
                  C.walking.step.current++;
                  C.walking.time.sec = 0.0;
                  C.walking.step.LastLfoot = C.LFoot.refpos;
                  C.walking.step.LastRfoot = C.RFoot.refpos;
              }
    return;
}

void kick_process() {

    double base_Y = 0.09;

    if (t <= 2*T) {

            Global.BA(X) = 0;
            Global.BA(Y) = 0;
            Global.BA(Z) = C.cosWave(t, C.zmp.previewControl.PreviewReadyTime, 0.308, 0.25);

            q.L.pre = KM.Geometric_IK_L(Global.BA, Global.LF);
            q.R.pre = KM.Geometric_IK_R(Global.BA, Global.RF);
        }


        else if (t > 2*T and 3*T >= t) {
            Global.BA(X) = 0;
            Global.BA(Y) = C.cosWave(t-2*T, T, 0, base_Y);
            Global.BA(Z) = 0.25;


            q.L.pre = KM.Geometric_IK_L(Global.BA, Global.LF);
            q.R.pre = KM.Geometric_IK_R(Global.BA, Global.RF);


        }

        else if (t > 3*T and 4*T >= t) {
            Global.BA(X) = 0;
            Global.BA(Y) = base_Y;
            Global.BA(Z) = 0.25;

            Global.LF(X) = 0;
            Global.LF(Y) = 0.05;
            Global.LF(Z) = 0;
            Global.RF(X) = C.cosWave(t-3*T, T, 0, -0.05);
            Global.RF(Y) = -0.05;
            Global.RF(Z) = C.cosWave(t-3*T, T, 0, base_Y);


            q.L.pre = KM.Geometric_IK_L(Global.BA, Global.LF);
            q.R.pre = KM.Geometric_IK_R(Global.BA, Global.RF);


        }

        else if (t > 4*T and 5*T >= t) {
            Global.BA(X) = 0;
            Global.BA(Y) = base_Y;
            Global.BA(Z) = 0.25;

            Global.LF(X) = 0;
            Global.LF(Y) = 0.05;
            Global.LF(Z) = 0;
            Global.RF(X) = C.cosWave(t-4*T, T, -0.05, 0.1);
            Global.RF(Y) = -0.05;
            Global.RF(Z) = C.cosWave(t-4*T, T, base_Y, 0.02);


            q.L.pre = KM.Geometric_IK_L(Global.BA, Global.LF);
            q.R.pre = KM.Geometric_IK_R(Global.BA, Global.RF);
        }

        else if (t > 5*T and 6*T >= t) {
            Global.BA(X) = 0;
            Global.BA(Y) = base_Y;
            Global.BA(Z) = 0.25;

            Global.LF(X) = 0;
            Global.LF(Y) = 0.05;
            Global.LF(Z) = 0;
            Global.RF(X) = C.cosWave(t-5*T, T, 0.1, 0);
            Global.RF(Y) = -0.05;
            Global.RF(Z) = 0.02;


            q.L.pre = KM.Geometric_IK_L(Global.BA, Global.LF);
            q.R.pre = KM.Geometric_IK_R(Global.BA, Global.RF);


        }

        else if (t > 6*T and 7*T >= t) {
            Global.BA(X) = 0;
            Global.BA(Y) = base_Y;
            Global.BA(Z) = 0.25;

            Global.LF(X) = 0;
            Global.LF(Y) = 0.05;
            Global.LF(Z) = 0;
            Global.RF(X) = 0;
            Global.RF(Y) = -0.05;
            Global.RF(Z) = C.cosWave(t-6*T, T, 0.02, 0);


            q.L.pre = KM.Geometric_IK_L(Global.BA, Global.LF);
            q.R.pre = KM.Geometric_IK_R(Global.BA, Global.RF);
        }

        else if (t > 7*T and 8*T >= t) {
            Global.BA(X) = 0;
            Global.BA(Y) = C.cosWave(t-7*T, T, base_Y, 0);
            Global.BA(Z) = 0.25;

            Global.LF(X) = 0;
            Global.LF(Y) = 0.05;
            Global.LF(Z) = 0;
            Global.RF(X) = 0;
            Global.RF(Y) = -0.05;
            Global.RF(Z) = 0;


            q.L.pre = KM.Geometric_IK_L(Global.BA, Global.LF);
            q.R.pre = KM.Geometric_IK_R(Global.BA, Global.RF);


        }



}

int convertRadTow4095(double rad) {
    return (int) ((rad + M_PI) * 2048.0 / M_PI);
}

double convertw4095ToRad(int w4095) {
    return (w4095 - 2048) * M_PI / 2048.0;
}

void catch_signal(int sig) {
    //signal(sig, SIG_IGN);
    printf("Program END...\n");
    cout<<"program end"<<endl;

    //close(dxl.serial_port);
    printf("Program END...\n");
    ros::shutdown();
    exit(0);
}
