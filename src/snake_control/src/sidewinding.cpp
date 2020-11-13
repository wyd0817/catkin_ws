/*
 * winding_gati.cpp
 *
 *  Created on: Sep 22, 2016
 *      Author: TI
 */

#include <ros/ros.h>
#include <cmath>
#include "sidewinding.h"

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SideWinding::set_alpha(double alpha)
{
	serpenoid_curve.alpha = alpha;
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SideWinding::set_l(double l)
{
	serpenoid_curve.l = l;
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SideWinding::set_bias(double bias)
{
	double kp_bias = 0.22;      // 操舵バイアス比例ゲイン
	double biasmax = M_PI/8;	// 最大バイアス 単位は[rad]
	double b = 0;

	b = bias* kp_bias * serpenoid_curve.v; // 操舵バイアス（速度にも比例してインクリメント）
	if(b > biasmax) b = biasmax;
	if(b < -biasmax) b = -biasmax;

	bias_ = b;
}


/*
 * @fn
 * @brief
 * @param
 * @paran
 * @return なし
 * @detail
*/
void SideWinding::set_v(double v)
{
	serpenoid_curve.v = v;
	s_ += v * dt_;
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SideWinding::print_parameters()
{
	ROS_INFO("*");
	ROS_INFO("* -->  serpenoid_curve.alpha = [%4.3f rad] *", serpenoid_curve.alpha);
	ROS_INFO("* -->  serpenoid_curve.l     = [%4.3f m  ] *", serpenoid_curve.l);
	ROS_INFO("* -->                      s = [%4.3f m  ] *", s_);
	ROS_INFO("* -->                   bias = [%4.3f m  ] *", bias_);
	ROS_INFO("------------     SideWinding      ----------");
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SideWinding::SideWindingShift(RobotSpec spec)
{
	while(s_ > (pre_s_ + step_s_)){  //

		SideWinding::CalculateCurvature();

		ShiftControlMethod::Shift_Param_Forward(spec);

		pre_s_ = pre_s_ + step_s_;
		SideWinding::CalculateTargetAngleToSideWinding(spec);
		
	}
	print_parameters();
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SideWinding::CalculateCurvature(){

	double a  = (M_PI*serpenoid_curve.alpha) / (2*serpenoid_curve.l);
	double ss = (M_PI/2) * (pre_s_/serpenoid_curve.l);
	kappa_    = a*sin(ss);
	psi_     += 0.01;
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SideWinding::CalculateTargetAngleToSideWinding(RobotSpec spec)
{
	snake_model_param.angle.clear();

	for(int i=0; i<num_link_; i++){
		if(i%2){  /*  (奇数番目) pitch joints?   */
			target_angle_ =
					-2*link_length_*snake_model_param.kappa[i]*sin(snake_model_param.psi[i]);
										//横うねり推進ではroll 軸の回転はしないため，ψ(s)＝０になる．//そのため，計算上はpis_を消しでもいい．
										//また，平面運動ので   PITCH軸は動かないのでPICHT軸の目標角度を０にした．
		}else{    /* (偶数番目)   */
			target_angle_ =
					2*link_length_*snake_model_param.kappa[i]*cos(snake_model_param.psi[i] + M_PI/4); // + snake_model_param.bias[i];;
		}
		snake_model_param.angle.push_back(target_angle_);
	}
}


