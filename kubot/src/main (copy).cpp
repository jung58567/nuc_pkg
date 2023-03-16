#include "kubot.h"
#include "Ckubot/Ckubot.h"
#include "kinematics/kinematics.h"

//DXL dxl("/dev/ttyUSB0",2000000);
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

bool thread_is_running = true;

//***************Xenomai time variables***************//

RT_TASK RT_task1;
RTIME now, previous;    // System Test time
RTIME now1, previous1;
RTIME now2, previous2;  // Thread 1 cycle time

void serial_task(void* arg);
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
            now1 = rt_timer_read();
            // t += (long)(now1-previous1)/cycle_ns;
            // cout << "Thread_time : " << int(t/cycle_ns) << "s  dt : " << (long)(now1-previous1)/1000 << "ns" << endl;
            previous1 = now1;
        }
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kubot_node");
    ros::NodeHandle nh;

    ROS_INFO("run main.cpp");
    ros::Subscriber sub = nh.subscribe("dxl_switch", 1000, dxl_switch_Callback);
    // ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    signal(SIGINT, catch_signal);
    signal(SIGTERM,catch_signal);

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
    DXL_Controller_.Read_Dxl_Encoder_Once(encoder_read);
    for(int i = 0; i < 12; i++) { // 12번 반복
    //    ROS_INFO("dxl.present_position[%d]=%d", i+1, dxl.present_position[i]);
        present_pos[i] = convertw4095ToRad(encoder_read[i]);
     ROS_INFO("present_pos[%d]=%f", i+1, present_pos[i]);
    }

    jointdirection_R << -1,-1,-1,-1, 1, 1;
    jointdirection_L << -1,-1, 1, 1,-1, 1;

    initializeKubot();
    C.initializeCKubot();
    C.setWalkingStep(11, C.walking.step.S_L_F, 0.05, 0, 0, 0.05);
    C.setWalkingTime(0.6, 0.4);
    C.setZmpPreview(1.5);
    C.initializeZmpTraj();
    C.FootPlanner();
    C.generateZmpTraj(C.walking.step.Lfoot, C.walking.step.Rfoot); 
}

void serial_task(void* arg)
{
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns * 10);

    //previous1 = rt_timer_read();

    while (thread_is_running)
    {
        rt_task_wait_period(NULL);

        if(t <= C.zmp.previewControl.PreviewReadyTime) {
            init_pose();
        }
        else {
             process();
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

        // previous1 = now1;

        t = (double)cnt / (double)onesecSize;
        C.walking.time.sec = t - C.zmp.previewControl.PreviewReadyTime - C.walking.step.current * C.walking.time.periodTime;
        C.generateFootTraj();
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

        // cout << "time:" << t << endl;

        cnt++;
    }
    return;
}

void init_pose() {
    Global.BA(X) = C.zmp.previewControl.X.CoM;
    Global.BA(Y) = C.zmp.previewControl.Y.CoM;
    Global.BA(Z) = C.cosWave(t, C.zmp.previewControl.PreviewReadyTime, 0.308, 0.25);

    q.R.pre = KM.Geometric_IK_R(Global.BA, Global.RF);
    q.L.pre = KM.Geometric_IK_L(Global.BA, Global.LF);
}

void process() {
    Global.BA(X) = C.zmp.previewControl.X.CoM;
    Global.BA(Y) = C.zmp.previewControl.Y.CoM;
    Global.BA(Z) = 0.25;

//    Global.LF(X) = 0;
//    Global.LF(Y) = 0.05;
//    Global.LF(Z) = 0;
//    Global.RF(X) = 0;
//    Global.RF(Y) = -0.05;
//    Global.RF(Z) = 0;

    Global.LF(X) = C.LFoot.refpos(X);
    Global.LF(Y) = C.LFoot.refpos(Y);
    Global.LF(Z) = C.LFoot.refpos(Z);
    Global.RF(X) = C.RFoot.refpos(X);
    Global.RF(Y) = C.RFoot.refpos(Y);
    Global.RF(Z) = C.RFoot.refpos(Z);

    q.R.pre = KM.Geometric_IK_R(Global.BA, Global.RF);
    q.L.pre = KM.Geometric_IK_L(Global.BA, Global.LF);

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
