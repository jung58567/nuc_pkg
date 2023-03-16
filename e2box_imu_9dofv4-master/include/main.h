#ifndef MAIN_H
#define MAIN_H

#include <ros/ros.h>
#include "t_serial.h"
#include "e2box_imu_9dofv4.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>

class IMU
{
public:
    void OnReceiveImu(void);
    void publishImuData(void);
};
#endif 