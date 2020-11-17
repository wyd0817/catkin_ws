/*
 * main.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: ubuntu-ti
 */

#include <ros/ros.h>
#include <joy_handler_hori/JoySelectedData.h>

#include "robot_spec.h"
#include "snake_control.h"

joy_handler_hori::JoySelectedData joystick;

// NO USED, just for test
void callback(joy_handler_hori::JoySelectedData joy_selected_data)
{
	if(joy_selected_data.button_circle){
		ROS_INFO("joy_selected_data.button_circle");
	}

	if(joy_selected_data.button_cross){
		ROS_INFO("joy_selected_data.button_cross");
	}

	if(joy_selected_data.button_triangle){
		ROS_INFO("joy_selected_data.button_triangle");
	}

	if(joy_selected_data.button_square){
		ROS_INFO("joy_selected_data.button_square");
	}

	if(joy_selected_data.button_ps){
		ROS_INFO("joy_selected_data.button_ps");
	}

	if(joy_selected_data.button_select){
		ROS_INFO("joy_selected_data.button_select");
	}

	if(joy_selected_data.button_start){
		ROS_INFO("joy_selected_data.button_start");
	}
}

// 通常   20ミリ秒に1回でtimerCallback関数を呼び出す
void timerCallback(const ros::TimerEvent& event)
{
	static int mode = 0;

	/* if (joystick.button_ps) {
		mode = 0;    //ストップモード
		ROS_INFO("Stop mode -->");
	}*/

    /* S Shaped Longitudnal Wave */
	if (joystick.button_select and joystick.button_r1) {
	// if (1) {
		mode=1;    // s字縦波モーション
		ROS_INFO("***    S Shaped Longitudnal Wave   -->  ***");
	}
    if(mode==1){

    	SnakeControl::OperateMoveShapedLongitudnalWave(joystick);
    }

    /* sinus liftinhg */
	if (joystick.button_select and joystick.button_r2) {
		mode=2;    //sinus liftinhg
		ROS_INFO("***  sinus liftinhg -->  ***");
	}
    if(mode==2){
    	SnakeControl::OperateMoveSinusLifting(joystick);
    }

    /* pedal wave motion */
	if (joystick.button_select and joystick.button_l1) {
		mode=3;    //pedal wave motion
		ROS_INFO("***  pedal wave motion -->  ***");
	}
    if(mode==3){
    	SnakeControl::OperateMovePedalWaveMotion(joystick);
    }

    /* sidewinding */
	if (joystick.button_select and joystick.button_l2) {
		mode=4;    // sidewinding
		ROS_INFO("***  sidewinding  -->  ***");
	}
    if(mode==4){
    	SnakeControl::OperateMoveSideWinding(joystick);
    }

	/* warp gait */
	if (joystick.button_select and joystick.button_square) {
			mode=5;    //warp gait
			ROS_INFO("***  Warp Gait -->  ***");
	}

	if(mode==5){
		SnakeControl::OperateMoveWarpGait(joystick);
	}

    /* Helical Wave Propagation Motion */
	if (joystick.button_select and joystick.button_circle) {
		mode=7;    //helical wave propagate motion
		ROS_INFO("***  Helical Wave Propagation Motion -->  ***");
	}
    if(mode==7){
    	SnakeControl::OperateMoveHelicalWavePropagateMotion(joystick);
    }

	/* Inchworm Gait */
	if (joystick.button_select and joystick.button_triangle) {
		mode=8;    //Inchworm Gait
		ROS_INFO("Inchworm Gait -->");
	}
    if(mode==8){	//Inchworm Gait
    	//SnakeControl::OperateMoveWindingShift(joystick);
    	SnakeControl::OperateMoveInchwormGait(joystick);
    }




    //All motor TORQUE ON
    if(joystick.button_start){
    	SnakeControlRequest::RequestJointActivateAll();  //
		ros::Duration(0.2).sleep();
    }

    // All motor TORQUE OFF
    if(joystick.button_select and joystick.button_start){
    	SnakeControlRequest::RequestJointFreeAll();
    	ros::Duration(0.2).sleep();
    }

    if(joystick.button_ps){
    	uint8_t id=0;
    	SnakeControlRequest::RequestJointPing(id);
    }

    if (joystick.button_r3 ){	//ノード再起動
    	pid_t pid;
    	char path_name[64];
    	char process_name[64];
    	char arg_name[64];
    	strcpy( path_name, "/opt/ros/indigo/bin/" );
    	strcpy( process_name, "rosrun" );
    	strcpy( arg_name, "snake_control snake_control_node" );
    	pid = fork();

    	switch( pid ){
    	case -1:
    		perror( "fork failed" );
    		exit(1);
    	case 0: // 子プロセス
    		chdir( path_name );
    		execl( process_name, process_name, "snake_control", "snake_control_node", (char *)NULL);
    		//execl( process_name, process_name, (char *)NULL );
    		break;
    	default: // 親プロセス
    		break;
    	}
    	ros::Duration(0.5).sleep();
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "snake_control");//初始化节点名称
	ROS_INFO("snake_control node initialized !");

	ros::NodeHandle nh;//声明一个节点句柄来与ROS系统进行通信
	// nh.param("loop_rate", SnakeControl::loop_rate_, 50.0);//50Hz = 0.02 秒
	nh.param("loop_rate", SnakeControl::loop_rate_, 100.0);//50Hz = 0.02 秒


	SnakeControl::Initialize();

	// 声明订阅者,创建一个订阅者 joy_selected_data ,
	// 它利用 joy_handler_hori 功能包的的 JoySelectedData 消息文件。
	// 话题名称是 "joy_selected_data" ,订阅者队列( queue )的大小设为 1 。
	ros::Subscriber sub_joy_selected_data =
			nh.subscribe("joy_selected_data", 1, &SnakeControl::CallBackOfJoySelectedData);

	ros::NodeHandle nh_timer;
	SnakeControl::sampling_time_ = 1.0/SnakeControl::loop_rate_;  // サンプリング時間を設定

	// タイマーを作る Durationの単位は[sec]
	//指定された間隔で呼び出すコールバック関数です．
	//createTimer関数の第一引数であるDurationは，呼び出す間隔を表します．
	//ros::Duration(0.02)とした場合には，0.02分の1秒(20ミリ秒に1回)の間隔でtimerCallback関数を呼び出します．
	ros::Timer timer = nh_timer.createTimer(ros::Duration(SnakeControl::sampling_time_), timerCallback);
	// 用于调用后台函数,等待接收消息。在接收到消息时执行后台函数。
	ros::spin();
}
