


#include "pedal_wave_motion.h"

void PedalWaveMotion::set_alpha(double alpha)
{
	serpenoid_curve.alpha = alpha;
}

void PedalWaveMotion::set_l(double l) 
{
	serpenoid_curve.l = l;
}

void PedalWaveMotion::set_v(double v)
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
void PedalWaveMotion::print_parameters()
{
	ROS_INFO("* -->  serpenoid_curve.alpha = [%4.3f rad] *", serpenoid_curve.alpha);
	ROS_INFO("* -->  serpenoid_curve.l     = [%4.3f m  ] *", serpenoid_curve.l);
	ROS_INFO("* -->                      s = [%4.3f m  ] *", s_);
	ROS_INFO("--------------------------------------------");
}


void PedalWaveMotion::PedalWaveMotionByShift(RobotSpec spec)
{
	while(s_ > (pre_s_ + step_s_)){  //

		PedalWaveMotion::CalculateCurvature();

		ShiftControlMethod::Shift_Param_Forward(spec);

		PedalWaveMotion::CalculateTargetAngle(spec);
		pre_s_ = pre_s_ + step_s_;
		
	}
	print_parameters();
}

void PedalWaveMotion::CalculateCurvature()
{
	double a = (M_PI*serpenoid_curve.alpha) / (2*serpenoid_curve.l);
	double ss = (M_PI/2) * (pre_s_/serpenoid_curve.l);
	kappa_ = ss;//a*sin(ss);
}

void PedalWaveMotion::CalculateTargetAngle(RobotSpec spec)
{
	snake_model_param.angle.clear();

	double a = (M_PI*serpenoid_curve.alpha) / (2*serpenoid_curve.l);

	for(int i=0; i<num_link_; i++){

		//(奇数番目)
		if(i%2){
			target_angle_ =
					-2*link_length_*sin(snake_model_param.kappa[i]*4*M_PI)*a;

		//(偶数番目)
		}else{
			target_angle_ =
					2*link_length_*sin(snake_model_param.kappa[i]*2*M_PI)*a;  //*sin(2*M_PI*(M_PI/2) * (pre_s_/serpenoid_curve.l));
		}

		snake_model_param.angle.push_back(target_angle_);
	}
}
