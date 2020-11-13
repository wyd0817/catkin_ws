


#include "sinus_lifting.h"


void SinusLifting::set_alpha(double alpha)
{
	serpenoid_curve.alpha = alpha;
}

void SinusLifting::set_l(double l) 
{
	serpenoid_curve.l = l;
}

void SinusLifting::set_v(double v)
{
	s_ = serpenoid_curve.v += v;
}

/*
 * @fn
 * @brief 
 * @param 
 * @paran 
 * @return なし
 * @detail
*/
void SinusLifting::print_parameters()
{
	ROS_INFO("* -->  serpenoid_curve.alpha = [%4.3f rad] *", serpenoid_curve.alpha);
	ROS_INFO("* -->  serpenoid_curve.l     = [%4.3f m  ] *", serpenoid_curve.l);
	ROS_INFO("* -->                      s = [%4.3f m  ] *", s_);
	ROS_INFO("-------------   Sinus Lifting  -------------");
}

void SinusLifting::SinusLiftingByShift(RobotSpec spec)
{
	while(s_ > (pre_s_ + step_s_)){  //

		SinusLifting::CalculateCurvature();

		ShiftControlMethod::Shift_Param_Forward(spec);

		SinusLifting::CalculateTargetAngle(spec);
		pre_s_ = pre_s_ + step_s_;
		
	}
	print_parameters();
}

void SinusLifting::CalculateCurvature()
{
	double k_0 = (M_PI*serpenoid_curve.alpha) / (2*serpenoid_curve.l);
	double k   = (M_PI/2) * (pre_s_/serpenoid_curve.l);

	kappa_zero_ = k_0;
	kappa_      = k;
}

void SinusLifting::CalculateTargetAngle(RobotSpec spec)
{
	double kappa = 0,
			kappa_0 = 0;

	snake_model_param.angle.clear();

	for(int i=0; i<num_link_; i++){
		kappa   = snake_model_param.kappa[i];
		kappa_0 = snake_model_param.kappa_zero[i];

		if(i%2){   /* 奇数番目 pitch joints ? ロボットの構造による*/
			target_angle_ =
					-2*link_length_*kappa_0*sin(4*M_PI*kappa);

		}else{ /* 偶数番目 yaw joints ? ロボットの構造による*/
			target_angle_ =
					 2*link_length_*kappa_0*sin(2*M_PI*kappa);
		}
		snake_model_param.angle.push_back(target_angle_);
	}
}
