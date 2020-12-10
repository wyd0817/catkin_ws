#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "sim_snake_class.h"

using namespace std;


ros::Publisher pub_target_position;
ros::Publisher pub_current_position;
ros::Publisher pub_collision_position;

void Sim_snake_IO::Set_publisher(ros::NodeHandle nh){

  pub_target_position = nh.advertise<std_msgs::Float32MultiArray>("target_motor_angle", 100);
  pub_current_position = nh.advertise<snake_msgs::SnakeJointData>("joint_position",100);
  pub_collision_position = nh.advertise<snake_msgs_abe::FsensorData>("collision_position",1);//変更
  pub_IMU_data_raw = nh.advertise<sensor_msgs::Imu>("sim_imu_data_raw",1);
  pub_IMU_data = nh.advertise<snake_msgs::SnakeIMUData>("imu_data",1);
}

void Sim_snake_IO::Sub_time(const std_msgs::Float32ConstPtr& msg){
  int result;
  sim_time = msg->data;
}
void Sim_snake_IO::Pub_motor_angle(void){

    msg.data.clear();

    for(int i=0; i<JOINT_NUM; i++)
    {
      msg.data.push_back(target_motor_angle[i]);
      //cout << i << " : "<< target_motor_angle[i] << endl;
    }
    //cout << endl;
    pub_target_position.publish(msg);
}

void Sim_snake_IO::Sub_motor_angle(const snake_msgs::SnakeJointData joint_target_pos){

  unsigned int joint_ID = 0;

  joint_ID = joint_target_pos.joint_index;
    //cout << joint_ID << endl;
  target_motor_angle[joint_ID] = joint_target_pos.value * 2.0 * M_PI/360.0;
}
void Sim_snake_IO::Sub_current_motor_angle(std_msgs::Float32MultiArray motor_data){

  ros::Time time_stamp(motor_data.data[0]);

  for(int i=0; i<JOINT_NUM; i++){
    current_motor_angle[i] = motor_data.data[i+1];
    joint_data.timestamp = time_stamp;
    joint_data.joint_index = i;
    joint_data.value = current_motor_angle[i]*180/M_PI;

    pub_current_position.publish(joint_data);

  }
}

void Sim_snake_IO::Sub_imu_data(std_msgs::Float32MultiArray imu_data_array){

  imu_data_snake.imu_index = 0;

  imu_data_snake.accel_x = imu_data_array.data[0];
  imu_data_snake.accel_y = imu_data_array.data[1];
  imu_data_snake.accel_z = imu_data_array.data[2];

  imu_data_snake.gyro_x = imu_data_array.data[3];
  imu_data_snake.gyro_y = imu_data_array.data[4];
  imu_data_snake.gyro_z = imu_data_array.data[5];

  imu_data.linear_acceleration.x = -imu_data_array.data[0];
  imu_data.linear_acceleration.y = -imu_data_array.data[1];
  imu_data.linear_acceleration.z = -imu_data_array.data[2];

  imu_data.angular_velocity.x = imu_data_array.data[3];///M_PI * 180;
  imu_data.angular_velocity.y = imu_data_array.data[4];///M_PI * 180;
  imu_data.angular_velocity.z = imu_data_array.data[5];///M_PI * 180;
  imu_data.header.frame_id ="imu_tf_raw";
  pub_IMU_data_raw.publish(imu_data);


}

void Sim_snake_IO::Sub_imu_roll_pitch_yaw(sensor_msgs::Imu rpy_data){

  imu_data_snake.imu_index = 0;
  tf::Vector3 imu_rpy;
  geom_imu_quaternion = rpy_data.orientation;
  tf::Quaternion tf_imu_quaternion( geom_imu_quaternion.x,
                                geom_imu_quaternion.y,
                                geom_imu_quaternion.z,
                                geom_imu_quaternion.w);
  tf::Matrix3x3 m(tf_imu_quaternion);

  m.getRPY(imu_data_snake.roll, imu_data_snake.pitch, imu_data_snake.yaw);

  pub_IMU_data.publish(imu_data_snake);

}

#if MIDDLE_POWER
void Sim_snake_IO::Sub_force_data(sensor_msgs::JointState force_data){

    static snake_msgs_abe::FsensorData collision_data;
    ros::Time time_stamp(force_data.header.stamp);
    collision_data.joint_index.clear();
    collision_data.angle.clear();
    collision_data.scale.clear();
    //collision_data.clear();
    collision_data.timestamp = time_stamp;

    for(int i=0 ; i<FORCE_SENSOR_NUM; i++){
      f_sensor_index = (uint)force_data.position[i];
      collision_data.joint_index.push_back(f_sensor_index);
      collision_data.angle.push_back(force_data.velocity[i]);
      collision_data.scale.push_back(force_data.effort[i]);
    }
    pub_collision_position.publish(collision_data);

}
#else

void Sim_snake_IO::Sub_force_data(sensor_msgs::JointState force_data){

    static snake_msgs_abe::FsensorData collision_data;
    ros::Time time_stamp(force_data.header.stamp);
    collision_data.joint_index.clear();
    collision_data.angle.clear();
    collision_data.scale.clear();
    //collision_data.clear();
    collision_data.timestamp = time_stamp;

    for(int i=0 ; i<FORCE_SENSOR_NUM; i++){  
      f_sensor_index = (uint)force_data.position[i];
      collision_data.joint_index.push_back(f_sensor_index);
      collision_data.angle.push_back(force_data.velocity[i]);
      collision_data.scale.push_back(force_data.effort[i]);
    }
    pub_collision_position.publish(collision_data);

}

#endif
