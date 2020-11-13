/*
 * winding_gati.cpp
 *
 *  Created on: Sep 22, 2016
 *      Author: TI
 */

#include <ros/ros.h>
#include <cmath>
#include "winding_gait.h"

/*
 * @fn
 * @brief
 * @param
 * @paran
 * @return なし
 * @detail
*/
void WindingGait::set_alpha(double alpha)
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
void WindingGait::set_l(double l)
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
void WindingGait::set_bias(double bias)
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
void WindingGait::set_v(double v)
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
void WindingGait::set_t(double t)
{
	if(serpenoid_curve.t < 3.14)
					serpenoid_curve.t += (0.01*t);
	else
					serpenoid_curve.t = -3.14;

}

void WindingGait::set_flag_on()
{
	flag_ = true;
}

void WindingGait::set_flag_off()
{
	flag_ = false;
}

/*
 * @fn
 * @brief
 * @param
 * @paran
 * @return なし
 * @detail
*/
void WindingGait::print_parameters()
{
	// ROS_INFO("*");
	// ROS_INFO("* -->  serpenoid_curve.alpha = [%4.3f rad] *", serpenoid_curve.alpha);
	// ROS_INFO("* -->  serpenoid_curve.l     = [%4.3f m  ] *", serpenoid_curve.l);
	// ROS_INFO("* -->                      s = [%4.3f m  ] *", s_);
	// ROS_INFO("* -->                   bias = [%4.3f m  ] *", bias_);
	// ROS_INFO("* -->  serpenoid_curve.t     = [%4.3f m  ] *", serpenoid_curve.t);
	// ROS_INFO("* -->  cosh(1*t)             = [%4.3f m  ] *", cosh(1*serpenoid_curve.t));
	// ROS_INFO("* -->  1/cosh(1*t)           = [%4.3f m  ] *", 1/cosh(1*serpenoid_curve.t));
	//
	// ROS_INFO("------------     Winding Gait     ----------");
}

/*
 * @fn
 * @brief
 * @param
 * @paran
 * @return なし
 * @detail
*/
void WindingGait::WindingShift(RobotSpec spec)
{
	while(s_ > (pre_s_ + step_s_)){  //

		WindingGait::CalculateCurvature();

		ShiftControlMethod::Shift_Param_Forward(spec);

		pre_s_ = pre_s_ + step_s_;
		WindingGait::CalculateTargetAngleToWinding(spec);

	}
	while(serpenoid_curve.t != pre_t_){  //

		WindingGait::CalculateCurvature();

		ShiftControlMethod::Shift_Param_Forward(spec);

		pre_t_ = serpenoid_curve.t;
		WindingGait::CalculateTargetAngleToWinding(spec);

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
		rho = a_*sech(omega_*t_ - phi_);
		a_t = rho + radius_;
		b_t = t_;

		x=a_t*sin(t_);
		y = 0;
		z= t_;

*/
void WindingGait::CalculateCurvature(){

	double pipe_r_ = 0.0891/2;//配管外径：８９.１mm　ｒｐ
  double pipe_d_ = 0.1325;//隣接配管との距離：１３２．５mm　　ｄｐ
  double radius_ = 1/(pipe_r_ + pipe_d_/2); //配管同士の中間地点を通っていける大きさ
	double a_     = radius_/2;
	//double a_     = 0;
	double num   = 0, denom = 0;

	double x[3], y[3], z[3];
	double omega_ = 1,phi_ = 0;
	double done;

	if(flag_){
		//1 回微分
		x[0] = cos(serpenoid_curve.t)*(a_*pow(cosh(omega_*serpenoid_curve.t-phi_),-1)+radius_)-a_*omega_*sin(serpenoid_curve.t)*
	    pow(cosh(omega_*serpenoid_curve.t-phi_),-2)*sinh(omega_*serpenoid_curve.t-phi_);
		y[0] = 0;
		z[0] = 1;
		// 2 回微分
		x[1] = 2*a_*pow(omega_,2)*sin(serpenoid_curve.t)*pow(cosh(omega_*serpenoid_curve.t-phi_),-3)*pow(sinh(omega_*
	    serpenoid_curve.t-phi_),2)-2*a_*omega_*cos(serpenoid_curve.t)*pow(cosh(omega_*serpenoid_curve.t-phi_),-2)*sinh(omega_*
	    serpenoid_curve.t-phi_)-sin(serpenoid_curve.t)*(a_*pow(cosh(omega_*serpenoid_curve.t-phi_),-1)+radius_)-a_*pow(omega_,2)*
	    sin(serpenoid_curve.t)*pow(cosh(omega_*serpenoid_curve.t-phi_),-1);
		y[1] = 0;
		z[1] = 0;
		// 3 回微分
		x[2] =  (-6*a_*pow(omega_,3)*sin(serpenoid_curve.t)*pow(cosh(omega_*serpenoid_curve.t-phi_),-4)*pow(sinh(omega_*
	    serpenoid_curve.t-phi_),3))+6*a_*pow(omega_,2)*cos(serpenoid_curve.t)*pow(cosh(omega_*serpenoid_curve.t-phi_),-3)*
	    pow(sinh(omega_*serpenoid_curve.t-phi_),2)+5*a_*pow(omega_,3)*sin(serpenoid_curve.t)*pow(cosh(omega_*
	    serpenoid_curve.t-phi_),-2)*sinh(omega_*serpenoid_curve.t-phi_)+3*a_*omega_*sin(serpenoid_curve.t)*pow(cosh(omega_*
	    serpenoid_curve.t-phi_),-2)*sinh(omega_*serpenoid_curve.t-phi_)-cos(serpenoid_curve.t)*(a_*pow(cosh(omega_*
	    serpenoid_curve.t-phi_),-1)+radius_)-3*a_*pow(omega_,2)*cos(serpenoid_curve.t)*pow(cosh(omega_*
	    serpenoid_curve.t-phi_),-1);
		y[2] = 0;
		z[2] = 0;

	// // 1 回微分
	// done = -1491*omega_*sinh(omega_*serpenoid_curve.t-phi_);
	// x[0] = (done*pow(cosh(omega_*serpenoid_curve.t-phi_),-2))/20000.0;
	// y[0] = 0;
	// z[0] = 1;
	// // 2 回微分
	// done = 1491*pow(omega_,2)*pow(sinh(omega_*serpenoid_curve.t-phi_),2);
	// x[1] = ((-1491.0)*pow(omega_,2)*pow(cosh(omega_*serpenoid_curve.t-phi_),-1))/
  //  20000.0+(done*pow(cosh(omega_*serpenoid_curve.t-phi_),-3))/10000.0;
	// y[1] = 0;
	// z[1] = 0;
	// // 3 回微分
	// done = -4473*pow(omega_,3)*pow(sinh(omega_*serpenoid_curve.t-phi_),3);
	// x[2] =  (1491.0*pow(omega_,3)*pow(cosh(omega_*serpenoid_curve.t-phi_),-2)*sinh(omega_*serpenoid_curve.t-phi_))/
  //    4000.0+(done*pow(cosh(omega_*serpenoid_curve.t-phi_),-4))/10000.0;
	// y[2] = 0;
	// z[2] = 0;

	num =
			sqrt(pow(y[0]*z[1] - z[0]*y[1], 2)
				 + pow(z[0]*x[1] - x[0]*z[1], 2)
				 + pow(x[0]*y[1] - y[0]*x[1], 2));

	denom =	pow(x[0]*x[0] + y[0]*y[0] + z[0]*z[0], 1.5);
	kappa_ = num/denom;
	//ROS_INFO("* -->  serpenoid_curve.t     = [%4.3f m  ] *", serpenoid_curve.t);
	//ROS_INFO("* -->  num/denom             = [%4.3f m  ] *", num/denom);
	ROS_INFO("%4.3f", num/denom);
}else{
	double a = (M_PI*serpenoid_curve.alpha) / (2*serpenoid_curve.l);
	double ss = (M_PI/2) * (pre_s_/serpenoid_curve.l);
	kappa_ = a*sin(ss);
	ROS_INFO("* -->  pre_s_/serpenoid_curve.l    = [%4.3f m  ] *", pre_s_/serpenoid_curve.l);
	ROS_INFO("* -->  ss											     = [%4.3f m  ] *", ss);
	}
}

/*
 * @fn
 * @brief
 * @param
 * @paran
 * @return なし
 * @detail
*/
void WindingGait::CalculateTargetAngleToWinding(RobotSpec spec)
{
	snake_model_param.angle.clear();

	for(int i=0; i<num_link_; i++){
		if(i%2){  /*  (奇数番目) pitch joints?   */
			target_angle_ = 0;          //-2*link_length_*snake_model_param.kappa[i]*sin(snake_model_param.psi[i]);
										//横うねり推進ではroll 軸の回転はしないため，ψ(s)＝０になる．//そのため，計算上はpis_を消しでもいい．
										//また，平面運動ので   PITCH軸は動かないのでPICHT軸の目標角度を０にした．
		}else{    /* (偶数番目)   */
			target_angle_ =
					2*link_length_*snake_model_param.kappa[i] + snake_model_param.bias[i]; //*cos(snake_model_param.psi[i]);
		}
		snake_model_param.angle.push_back(target_angle_);
	}
}
