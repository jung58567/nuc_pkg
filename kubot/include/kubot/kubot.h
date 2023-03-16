#ifndef KUBOT_H_
#define KUBOT_H_

// ROS library headers

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

// C library headers
#include <stdio.h>
#include <string.h>

// Eigen/RBDL library headers
// #include <eigen3/Eigen/Dense> -- kinematics.h 안에 있음
// #include <rbdl/rbdl.h>

// Basic libraries
#include <sys/mman.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>
#include <pthread.h>
#include <inttypes.h>

// Xenomai libraries
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <boilerplate/trace.h>
#include <xenomai/init.h>

// Kudos libraries
#include "dxl_serial/dxl_serial.h"
// #include "Ckubot/kinematics.h"
// #include "Ckubot/Ckubot.h"

#include "kubot_dxl_controller/dxl_controller.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::MatrixXf;
using Eigen::VectorXf;

#define SIZE 512

using namespace std;

// using namespace RigidBodyDynamics;
// using namespace RigidBodyDynamics::Math;

int position(double time);
int convertRadTow4095(double rad);
double convertw4095ToRad(int w4095);

void dxl_switch_Callback(const std_msgs::String::ConstPtr& msg);
void Load();
void init_pose();
void init_pose_2();
void process();
void kick_process();
void set_motor_id();

double func_1_cos(double t, double init, double final, double T);

#endif // kubot
