/*
 * warp_gait.h
 *
 *  Created on: Apr 19, 2017
 *      Author: ubuntu-ti
 */

#ifndef SNAKE_CONTROL_SRC_WARP_GAIT_H_
#define SNAKE_CONTROL_SRC_WARP_GAIT_H_

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "robot_spec.h"
#include "shift_control_method.h"

class WarpGait: public ShiftControlMethod{

public:
	virtual ~WarpGait(){}

	WarpGait(RobotSpec spec, double ds){

		ds_ = ds;
		pre_s_ = 0;
		step_s_= ds/28;
		sl_ = 0;
		sw_ = 0;
		S_T = 0;
		t = 0;

		target_angle_ = 0;

		num_link_ = spec.num_joint();
		link_length_ = 0.06;
		link_wide_ = 0.06;

		pipe_r_ = 0.0891/2;
		pipe_d_ = 0.1325;
		offset_ = 0.01;

		pai_multiple_ = 1.0;
		R1_multiple_ = pai_multiple_;
		R2_multiple_ = pai_multiple_;

		pai_ = 3.14159;

		//可動域の関係からRは0.076以上を指定
		R1_ = pipe_r_ + link_wide_/2 + offset_-0.00; //曲率の大きいU字曲線の半径
		R2_ = (pipe_r_ - link_wide_/2 + pipe_d_ - offset_)*R2_multiple_-0.00; //曲率の小さいU字曲線の半径*(2-pai_multiple_)
		R3_ = pipe_r_ + pipe_d_/2-0.015; //配管同士の中間地点を通っていける大きさ

		warp_mode_ = 0;
		warp_shape_ = 0;
		warp_shift_ = 0;

		Init(spec);
	}

	void set_sl(double sl);
	void add_sl(double add_sl){set_sl(sl_ + add_sl); }

	void set_sw(double sw);
	void add_sw(double add_sw){set_sw(sw_ + add_sw); }

	//--- 形状パラメータ変更
	void set_R1(double R1);
	void set_R2(double R2);
	void add_R1(double add_R1){ set_R1(R1_ + add_R1);}
	void add_R2(double add_R2){ set_R2(R2_ + add_R2);}

	//設置モード，推進モードの変更
	void WarpModeChange(int warp_mode);
	void WarpShapeChange(int warp_shape);
	void WarpShiftChange(int warp_shift);

	void Warpinit();

	//Installation
	void WarpGaitInstallationByShift(RobotSpec spec);
	void CalculateTargetAngleInstallation(RobotSpec spec);

	//Promotion
	void WarpGaitPromotion(RobotSpec spec);
	void CalculateTargetAnglePromotion(RobotSpec spec);

	double sl_, sw_, S_T, t;
	double pre_s_, step_s_;
	double target_angle_;

	int num_link_;
	double link_length_, link_wide_;

	double pipe_r_, pipe_d_;
	double offset_;
	double R1_,R2_,R3_;
	double kr_, kr1_, kr2_, rkr1_, rkr2_;
	double warp_s_;

	double pai_multiple_, R1_multiple_, R2_multiple_;
	double pai_;

	int warp_mode_, warp_shape_, warp_shift_;
};





#endif /* SNAKE_CONTROL_SRC_WARP_GAIT_H_ */
