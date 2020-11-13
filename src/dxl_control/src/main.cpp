/*
 *
 *
 *
 *
 *
 * */


#include <stdint.h>
#include <ros/ros.h>
//#include "dxl_xl320_control.h"
//#include "dxl_xh430_control.h"
#include "dxl_rx28_control.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "dxl_control_node");

  DynamixelControl::Initialize();
  ROS_INFO("Initialized : dxl_control node");
  while(ros::ok()) {

    ros::spin();
  }
}
