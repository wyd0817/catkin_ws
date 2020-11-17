/*
 * warp_gait.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: ubuntu-ti
 */


#include "warp_gait.h"

void WarpGait::set_sl(double sl)
{
	sl_ = sl;
}

void WarpGait::set_sw(double sw)
{
	sw_ = sw;
}

void WarpGait::set_R1(double R1)
{
	R1_ = R1;
}

void WarpGait::set_R2(double R2)
{
	R2_ = R2;
}

void WarpGait::WarpModeChange(int warp_mode)
{
	if(warp_mode == 1){
		warp_mode_ = warp_mode;
	}if(warp_mode == 0){
		warp_mode_ = warp_mode;
	}
}

void WarpGait::WarpShapeChange(int warp_shape)
{
	if(warp_shape == 1){
		warp_shape_ = warp_shape;
	}if(warp_shape == 0){
		warp_shape_ = warp_shape;
	}
}

void WarpGait::WarpShiftChange(int warp_shift)
{
	if(warp_shift == 0){
		warp_shift_ = warp_shift;
	}if(warp_shift == 1){
		warp_shift_ = warp_shift;
	}
}

void WarpGait::Warpinit(){
	sl_ = 0;
	sw_ = 0;

}

void WarpGait::WarpGaitInstallationByShift(RobotSpec spec){
		ROS_INFO("Installation mode");
		ROS_INFO("> sl_ = %4.3f\n", sl_);

		if(warp_shift_ == 0){
			while(sl_>(pre_s_+ step_s_)){
				kappa_ = (1/R3_)*pai_multiple_;
				double tau = (1/R3_)/pai_multiple_;

				tau_ = tau_ + (tau*step_s_);

				ShiftControlMethod::Shift_Param_Reverse(spec);
				CalculateTargetAngleInstallation(spec);
				pre_s_ = pre_s_ + step_s_;
			}
		}
		if(warp_shift_ == 0){
	    while(sl_<(pre_s_- step_s_)){
				kappa_ = (1/R3_)*pai_multiple_;
				double tau = (1/R3_)/pai_multiple_;

				tau_ = tau_ + (tau*step_s_);

				ShiftControlMethod::Shift_Param_Reverse(spec);
				CalculateTargetAngleInstallation(spec);
				pre_s_ = pre_s_ - step_s_;
	    }
		}
}


void WarpGait::CalculateTargetAngleInstallation(RobotSpec spec)
{
	for(int i=0; i<num_link_; i++){
		if(i%2){ //(奇数番目)
			target_angle_ =
					2*link_length_*snake_model_param.kappa[i]*sin(snake_model_param.tau[i] + snake_model_param.psi[i]);
			snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle_);

		}else{  //(偶数番目)
			target_angle_ = 0;
					//2*link_length_*snake_model_param.kappa[i]*cos(snake_model_param.tau[i]+ snake_model_param.psi[i]);
			snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle_);
		}
    snake_model_param.angle.pop_back();
		//snake_model_param.kappa.pop_back();
		//snake_model_param.tau.pop_back();
		//snake_model_param.psi.pop_back();
	}
	usleep(1000*10);        // 制御に時間がかかるので1秒寝て待つ
}

void WarpGait::WarpGaitPromotion(RobotSpec spec){
	kr1_ = 1/R1_ ;
	kr2_ = 1/R2_ ;

	ROS_INFO("> R1_ = %4.3f R2_ = %4.3f\n", R1_, R2_);

	if(warp_shape_ == 0){
		ROS_INFO("Promotion mode A");
		rkr1_ = kr1_ ;
		rkr2_ = -kr2_ ;
	}else if(warp_shape_ == 1){
		ROS_INFO("Promotion mode B");
		rkr1_ = kr2_ ;
		rkr2_ = -kr1_ ;
	}
	CalculateTargetAnglePromotion(spec);
}

void WarpGait::CalculateTargetAnglePromotion(RobotSpec spec)
{
	for(int i=0; i<num_link_; i++){
		warp_s_ = link_length_ * i;

		if(warp_shape_ == 0){
			if(0<=warp_s_ and warp_s_<(R1_multiple_*pai_*0*R1_ + R2_multiple_*pai_*1*R2_)){
				kr_ = rkr2_;
			}else if((R1_multiple_*pai_*0*R1_ + R2_multiple_*pai_*1*R2_)<=warp_s_ and warp_s_<(R1_multiple_*pai_*1*R1_ + R2_multiple_*pai_*1*R2_)){
				kr_ = rkr1_;
			}else if((R1_multiple_*pai_*1*R1_ + R2_multiple_*pai_*1*R2_)<=warp_s_ and warp_s_<(R1_multiple_*pai_*1*R1_ + R2_multiple_*pai_*2*R2_)){
				kr_ = rkr2_;
			}else if((R1_multiple_*pai_*1*R1_ + R2_multiple_*pai_*2*R2_)<=warp_s_ and warp_s_<(R1_multiple_*pai_*2*R1_ + R2_multiple_*pai_*2*R2_)){
				kr_ = rkr1_;
			}else if((R1_multiple_*pai_*2*R1_ + R2_multiple_*pai_*2*R2_)<=warp_s_ and warp_s_<(R1_multiple_*pai_*2*R1_ + R2_multiple_*pai_*3*R2_)){
				kr_ = rkr2_;
			}else if((R1_multiple_*pai_*2*R1_ + R2_multiple_*pai_*3*R2_)<=warp_s_ and warp_s_<(R1_multiple_*pai_*3*R1_ + R2_multiple_*pai_*3*R2_)){
				kr_ = rkr1_;
			}
		}else if(warp_shape_ == 1){
			if(0<=warp_s_ and warp_s_<(R1_multiple_*pai_*1*R1_ + R2_multiple_*pai_*0*R2_)){
				kr_ = rkr2_;
			}else if((R1_multiple_*pai_*1*R1_ + R2_multiple_*pai_*0*R2_)<=warp_s_ and warp_s_<(R1_multiple_*pai_*1*R1_ + R2_multiple_*pai_*1*R2_)){
				kr_ = rkr1_;
			}else if((R1_multiple_*pai_*1*R1_ + R2_multiple_*pai_*1*R2_)<=warp_s_ and warp_s_<(R1_multiple_*pai_*2*R1_ + R2_multiple_*pai_*1*R2_)){
				kr_ = rkr2_;
			}else if((R1_multiple_*pai_*2*R1_ + R2_multiple_*pai_*1*R2_)<=warp_s_ and warp_s_<(R1_multiple_*pai_*2*R1_ + R2_multiple_*pai_*2*R2_)){
				kr_ = rkr1_;
			}else if((R1_multiple_*pai_*2*R1_ + R2_multiple_*pai_*2*R2_)<=warp_s_ and warp_s_<(R1_multiple_*pai_*3*R1_ + R2_multiple_*pai_*2*R2_)){
				kr_ = rkr2_;
			}else if((R1_multiple_*pai_*3*R1_ + R2_multiple_*pai_*2*R2_)<=warp_s_ and warp_s_<(R1_multiple_*pai_*3*R1_ + R2_multiple_*pai_*3*R2_)){
				kr_ = rkr1_;
			}
		}

		if(i%2){ //(奇数番目)
			target_angle_ = 2*link_length_*sin(sw_)*kr_;
			snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle_);

		}else{  //(偶数番目)
			target_angle_ = 2*link_length_*cos(sw_)*kr_;
			snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle_);
		}

    snake_model_param.angle.pop_back();
		//snake_model_param.kappa.pop_back();
		//snake_model_param.tau.pop_back();
		//snake_model_param.psi.pop_back();
	}
	usleep(1000*20);        // 制御に時間がかかるので1秒寝て待つ
}
