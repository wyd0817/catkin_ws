
#include <stdint.h>
#include <ros/ros.h>

#include "control_robot_for_vrep.h"
#include "sim_snake_class.h"
#include "vrep.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "snake_robot_master_vrep_node");
  Sim_snake_IO sim_io;

  for(int i=0; i<JOINT_NUM; i++){
    sim_io.target_motor_angle[i] = 0.0;
    sim_io.current_motor_angle[i] = 0.0;
  }
  ros::NodeHandle node;
  sim_io.Set_publisher(node);

  ros::Subscriber sub_current_position = node.subscribe("current_motor_angle", 100, &Sim_snake_IO::Sub_current_motor_angle, &sim_io);
  ros::Subscriber sub_time = node.subscribe("simulationTime", 1, &Sim_snake_IO::Sub_time, &sim_io);
  ros::Subscriber sub_motor_angle = node.subscribe("joint_target_position", 100,  &Sim_snake_IO::Sub_motor_angle, &sim_io);
  ros::Subscriber sub_force_sensor = node.subscribe("force_data", 1, &Sim_snake_IO::Sub_force_data, &sim_io);
  ros::Subscriber sub_imu_data = node.subscribe("imu_data_vrep", 1, &Sim_snake_IO::Sub_imu_data, &sim_io);
  ros::Subscriber sub_imu_rpy = node.subscribe("sim_imu_data", 1, &Sim_snake_IO::Sub_imu_roll_pitch_yaw, &sim_io);
  
  RobotControlForVrep::Initialize();
  ROS_INFO("Initialized : snake_robot_master_vrep_node");
  while(ros::ok()) {
    sim_io.Pub_motor_angle();
    ros::spin();
  }
}

