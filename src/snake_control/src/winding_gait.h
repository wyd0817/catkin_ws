/*
 * winding_gait.h
 *
 *  Created on: Sep 22, 2016
 *      Author: TI
 */

#ifndef SNAKE_CONTROL_SRC_WINDING_GAIT_H_
#define SNAKE_CONTROL_SRC_WINDING_GAIT_H_

#include <ros/ros.h>
#include <vector>
#include <stdint.h>

#include "robot_spec.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "shift_control_method.h"

class WindingGait: public ShiftControlMethod {
 public:
	virtual ~WindingGait(){}
	WindingGait(RobotSpec spec,
			double min_alpha,
			double max_alpha,
			double min_l,
			double max_l,
			double ds
			){

		ds_ = ds;

		num_link_ 	  = spec.num_joint();
		link_length_  = spec.link_length_body();
		target_angle_ = 0;

		serpenoid_curve.alpha = M_PI/4;         // くねり角[rad]
		serpenoid_curve.l     = (num_link_*link_length_)/8;         // 曲線の1/4周期の長さ[m]
		serpenoid_curve.v     = 0.00;
    serpenoid_curve.t     = 0.00;

		s_ 	= 0;
		S_T 	= 0;
		dt_ 	= 0.010;  // サンプリングタイム10[msec]

		pre_s_  = 0;
		step_s_ = ds/28;

    pre_t_  = 0;

		psi_ 	= 0;
		psi_hyper_ = 0;

		tau_   = 0;
		kappa_ = 0;
    kappa_0_ = 0;
		bias_  = 0;
    flag_ 	= false;
		Init(spec);
	}

	//--- 動作
	void Winding(RobotSpec spec);
	void WindingShift(RobotSpec spec);
	void CalculateTargetAngleToWinding(RobotSpec spec);
	void CalculateCurvature();
	virtual void InitializeShape() {}

	//--- 形状パラメータ変更
	void set_alpha(double alpha);
	void add_alpha(double alpha_add){ set_alpha(serpenoid_curve.alpha + alpha_add); }
	void set_l(double l);
	void add_l(double l_add){ set_l(serpenoid_curve.l + l_add); }
	void set_bias(double bias);
	void add_bias(double bias){ set_bias(bias); }
	void set_v(double v);
  void set_t(double t);
	void add_v(double v_add){ set_v(v_add); }
  void add_t(double t_add){ set_t(t_add); }

	void set_flag_on();
	void set_flag_off();

	void print_parameters();

	double s_, 	dt_ ;
	double pre_s_;
  double pre_t_;
	double step_s_;
	double target_angle_;
	   int num_link_;
	double link_length_;
	double S_T;
  bool flag_;


};

#endif /* SNAKE_CONTROL_SRC_WINDING_GAIT_H_ */
