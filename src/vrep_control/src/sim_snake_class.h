#ifndef SIM_SNAKE_CLASS_H_
#define SIM_SNAKE_CLASS_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>

#include "snake_msgs/SnakeJointData.h"
#include "snake_msgs/SnakeJointData2.h"
#include "snake_msgs/SnakeIMUData.h"
#include "snake_msgs/SnakeJointCommand.h"

#include <iostream>

#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/Imu.h>

#include "snake_msgs/SnakeJointData.h"
#include "snake_msgs/SnakeJointData2.h"
#include "snake_msgs/SnakeIMUData.h"
#include "snake_msgs/SnakeJointCommand.h"

#include "snake_msgs_abe/FsensorData.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

//--------------set snake robot parameter----------------//
//middle power type spec
 //#define FORCE_SENSOR_NUM 15
 //#define JOINT_NUM 30

//high power type spec
 #define FORCE_SENSOR_NUM 22 //19
 #define JOINT_NUM 23

#define MIDDLE_POWER 0
//if used middle power type robot :: 1
//if used high power type robot   :: 0
//--------------------------------------------------------//

class Sim_snake_IO {
  public:
    double sim_time;

    int f_sensor_index;
    float target_motor_angle[JOINT_NUM];
    float current_motor_angle[JOINT_NUM];
    double force_sensor_data[FORCE_SENSOR_NUM][4];
    double Force_z, Force_x;
    double theta[FORCE_SENSOR_NUM];
    double scale[FORCE_SENSOR_NUM];

    ros::Publisher pub_target_position;
    ros::Publisher pub_current_position;
    ros::Publisher pub_collision_position;
    ros::Publisher pub_IMU_data_raw;
    ros::Publisher pub_IMU_data;

    std_msgs::Float32MultiArray msg;

    void Set_publisher(ros::NodeHandle nh);
    void Pub_motor_angle();
    void Sub_time(const std_msgs::Float32ConstPtr& msg);
    void Sub_force_data(sensor_msgs::JointState force_data);
    void Sub_motor_angle(const snake_msgs::SnakeJointData joint_target_pos);
    void Sub_current_motor_angle(std_msgs::Float32MultiArray motor_data);
    void Sub_imu_data(std_msgs::Float32MultiArray imu_data_array);
    void Sub_imu_roll_pitch_yaw(sensor_msgs::Imu rpy_data);

    sensor_msgs::Imu imu_data;
    snake_msgs::SnakeJointCommand joint_command;
    snake_msgs::SnakeJointData joint_data;
//    snake_msgs::SnakeJointData2 collision_data;
    // snake_msgs_abe::FsensorData collision_data;
    snake_msgs::SnakeIMUData imu_data_snake;

    geometry_msgs::Quaternion geom_imu_quaternion;

};

#endif
