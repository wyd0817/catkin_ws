/*
 * s_shaped_longitudnal_wave.cpp
 *
 *  Created on: Dec 30, 2018
 *      Author: Wang Yongdong
 */

#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include "s_shaped_longitudnal_wave.h"



//--- Publisher ---//
ros::Publisher SShapedLongitudnalWave::ros_parameter_pub_t;
ros::Publisher SShapedLongitudnalWave::ros_parameter_pub_phi;
ros::Publisher SShapedLongitudnalWave::ros_parameter_pub_s;
ros::Publisher SShapedLongitudnalWave::ros_parameter_pub_moving_distance;

ros::Publisher SShapedLongitudnalWave::ros_parameter_pub_angle_1;
ros::Publisher SShapedLongitudnalWave::ros_parameter_pub_angle_2;
ros::Publisher SShapedLongitudnalWave::ros_parameter_pub_angle_3;
ros::Publisher SShapedLongitudnalWave::ros_parameter_pub_angle_4;
ros::Publisher SShapedLongitudnalWave::ros_parameter_pub_angle_5;
ros::Publisher SShapedLongitudnalWave::ros_parameter_pub_angle_6;

/*
 * @fn
 * @brief
 * @param
 * @paran
 * @return なし
 * @detail
*/
void SShapedLongitudnalWave::set_alpha(double alpha)
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
void SShapedLongitudnalWave::set_l(double l)
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
void SShapedLongitudnalWave::set_wavelength(double wavelength)
{
	wavelength_ = wavelength;
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
void SShapedLongitudnalWave::set_radius(double radius)
{
	radius_ = radius;
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
void SShapedLongitudnalWave::set_bias(double bias)
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
 * @brief;//
 * @param
 * @paran
 * @return なし
 * @detail
*/
void SShapedLongitudnalWave::set_v(double v)
{
	serpenoid_curve.v = v;
	s_ += v * dt_;
}


bool  Flag_Cycle_Plus = false ,Flag_Cycle_Reduce = false ,Flag_Now_length_t_ = true ;
//20ミリ秒に1回でこの関数を呼び出す  adyon
void SShapedLongitudnalWave::set_phi(RobotSpec spec,double phi)
{
	double curve_period_ = wavelength_/moving_speed_;//[s]
	double omega_ = 2*pi/curve_period_;//Angular frequency
	double omega_hyperboli_ = 4*pi/curve_period_;//Angular frequency
	if(phi > 0)
	{
		 if(serpenoid_curve.phi < (Now_length_t_ + 0.05)){///////0.25
						serpenoid_curve.phi += (0.04*phi);
					}
	  else{
	  				// serpenoid_curve.phi = -0.25;
						serpenoid_curve.phi = 0;
						Flag_Cycle_Plus = true  ;
					}
	 }else if(phi < 0){
		 if(serpenoid_curve.phi > Now_length_t_-First_Now_length_t_-0.05){
						serpenoid_curve.phi += (0.04*phi);
					}
		else{
						// serpenoid_curve.phi = Now_length_t_ + 0.25;
						serpenoid_curve.phi = Now_length_t_;
						Flag_Cycle_Reduce = true  ;
					}
	 }
	 //serpenoid_curve.phi = -0.25;
}
void SShapedLongitudnalWave::set_flag_on()
{
	flag_ = true;
}

void SShapedLongitudnalWave::set_flag_off()
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
double Moving_distance = 0.0;
void SShapedLongitudnalWave::print_parameters()
{
	ROS_INFO("*  		  		A = [%4.3f] *", radius_);
	// ROS_INFO("*   				 		L = [%4.3f] *", wavelength_);
	 ROS_INFO("*   				  PHI = [%4.3f] *", pre_phi_);
	 ROS_INFO("*   		                  Moving_distance = [%4.3f] *", Moving_distance);
	// ROS_INFO("*   				 						 	kappa = [%4.3f] *", kappa_);
	// ROS_INFO("*   				 						          length_t_ = [%4.3f] *", Now_length_t_);
	//ROS_INFO("------------     S Shaped Longitudnal Wave     ----------");
}

void SShapedLongitudnalWave::Write_file_to_hard_disk(double data,const std::string &name,bool *flag)
{
	std::string fileName  = "/home/ubuntu/Dropbox/catkin_ws_dense_pipes/";
	fileName += name;
 	std::ofstream ofs;
 	if(*flag == true) {
 		ofs.open(fileName.c_str(), std::ofstream::out | std::ofstream::trunc  );*flag = false;}
 	else
 	 ofs.open(fileName.c_str(), std::ofstream::out | std::ofstream::app );
 	ofs << data ;
 	ofs << "\n";
  ofs.close();
}


/*
 * @fn
 * @brief
 * @param
 * @paran
 * @return なし
 * @detail
*/
int num0 = 0,num1 = 0;
void SShapedLongitudnalWave::SShapedLongitudnalWaveByShift(RobotSpec spec)
{
	while(s_ > (pre_s_ + step_s_)){  //

		SShapedLongitudnalWave::CalculateCurvature();

		ShiftControlMethod::Shift_Param_Forward(spec);

		pre_s_ = pre_s_ + step_s_;
		SShapedLongitudnalWave::CalculateTargetAngleForHyperbola(spec);

	}

	while(serpenoid_curve.phi != pre_phi_){  //

		SShapedLongitudnalWave::CalculateCurvature();

		ShiftControlMethod::Shift_Param_Forward(spec);

		pre_phi_ = serpenoid_curve.phi;
		SShapedLongitudnalWave::CalculateTargetAngleForHyperbola(spec);

	}
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
double full_length;
bool Kappa_First_file = true,Kappa_First_file_0 = true,
file_0 = true,file_1 = true,file_2 = true,file_3 = true,file_4 = true;
void SShapedLongitudnalWave::CalculateCurvature(){
}
double SShapedLongitudnalWave::CalcKappa(double t)
{
  //double kappa_ = 0;
  double x[2], y[2];

	double curve_period_ = wavelength_/moving_speed_;//[s]
	//bbbbbbbbbbb
	double a_     = 0.3*radius_;//Hyperbolic amplitude
	double phi_   = pre_phi_;//Hyperbolic phase
	double omega_ = 2*pi/curve_period_;//Angular frequency
	double omega_hyperboli_ = 4*pi/curve_period_;//Angular frequency
  omega_ = 2*pi/wavelength_;
		ROS_INFO("*a_ = [%4.3f] *", a_);
//phi_ = 0.5*Now_length_t_;
	//omega_ = 1;//Angular frequency
	double num   = 0, denom = 0;
  phi_  = -1*phi_;
	//pre_phi_ = 0;
	//1 回微分
	x[0] = 1;
	y[0] = omega_*cos(omega_*t)*(a_*pow(cosh(omega_*t+omega_hyperboli_*
	    phi_),-1)+radius_)-a_*omega_*sin(omega_*t)*pow(cosh(omega_*
	    t+omega_hyperboli_*phi_),-2)*sinh(omega_*t+omega_hyperboli_*phi_);
	// 2 回微分
	x[1] = 0;
	y[1] = 2*a_*pow(omega_,2)*sin(omega_*t)*pow(cosh(omega_*t+omega_hyperboli_*phi_),-3)*
	    pow(sinh(omega_*t+omega_hyperboli_*phi_),2)-2*a_*pow(omega_,2)*cos(omega_*
	    t)*pow(cosh(omega_*t+omega_hyperboli_*phi_),-2)*sinh(omega_*
	    t+omega_hyperboli_*phi_)-pow(omega_,2)*sin(omega_*t)*(a_*pow(cosh(omega_*
	    t+omega_hyperboli_*phi_),-1)+radius_)-a_*pow(omega_,2)*sin(omega_*t)*
	    pow(cosh(omega_*t+omega_hyperboli_*phi_),-1);

	num =x[0]*y[1] - x[1]*y[0];
	denom =	pow(x[0]*x[0] + y[0]*y[0], 1.5);
	kappa_ = num/denom;


	print_parameters();
	Write_file_to_hard_disk(kappa_,"kappa.txt",&file_2);
	if(kappa_ > 13.08)	 {kappa_ = 13.08; ROS_WARN("* kappa is too big*");}
  return kappa_;
}
//Yvon
double Num,Integral_joint_angle;
double s = 0.0,s_cal = 0.0;
double Past_x = 0.0;
bool  Flag_Execution_Initialization = true;
std_msgs::Float64 topic_t_;
std_msgs::Float64 topic_phi_;
std_msgs::Float64 topic_s_;
std_msgs::Float64 topic_moving_distance_;

std_msgs::Float64 topic_angle_1_;
std_msgs::Float64 topic_angle_2_;
std_msgs::Float64 topic_angle_3_;
std_msgs::Float64 topic_angle_4_;
std_msgs::Float64 topic_angle_5_;
std_msgs::Float64 topic_angle_6_;


static std::vector<double> joint_angle_backup(50, 0.0);//此处的50需要更改为规范的代码
std::vector<double> SShapedLongitudnalWave::CalculateTargetAngle(RobotSpec spec)
{
	double curve_period_ = wavelength_/moving_speed_;//[s]
	double omega_ = 2*pi/curve_period_;//Angular frequency
	double omega_hyperboli_ = 4*pi/curve_period_;//Angular frequency
  std::vector<double> joint_angle(spec.num_joint(), 0.0);  // 値0.0で初期化
	s = 0.0;
	//Moving_distance = 0;
  full_length = spec.full_length();
	// ROS_INFO("* serpenoid_curve.phi = [%4.3f] *", serpenoid_curve.phi);
	// ROS_INFO("* pre_phi_ = [%4.3f] *", pre_phi_);
	// Write_file_to_hard_disk(serpenoid_curve.phi,"serpenoidphi.txt",&file_0);
	// Write_file_to_hard_disk(pre_phi_,"prephi.txt",&file_1);
	if(serpenoid_curve.phi != pre_phi_){
		pre_phi_ = serpenoid_curve.phi;
  }
	// if(Flag_Cycle_Plus == true) {Moving_distance += 0.0332;Flag_Cycle_Plus = false;}
	// else if(Flag_Cycle_Reduce == true) {Moving_distance -= 0.0332;Flag_Cycle_Reduce = false;}
	// if(Flag_Cycle_Plus == true) {Moving_distance += 0.0532;Flag_Cycle_Plus = false;}
	// else if(Flag_Cycle_Reduce == true) {Moving_distance -= 0.0532;Flag_Cycle_Reduce = false;}

	//aaaaaaaaaaaaaaaa
	if(Flag_Cycle_Plus == true) {Moving_distance += 0.0432;Flag_Cycle_Plus = false;}
	else if(Flag_Cycle_Reduce == true) {Moving_distance -= 0.0432;Flag_Cycle_Reduce = false;}

	t_ = 0;
	S_T = 0;

  for(uint32_t i_joint=1; i_joint<spec.num_joint(); i_joint+=2) {
		joint_angle[i_joint] = joint_angle_backup[i_joint];//Restore the previous Angle
		s = (i_joint+1)*spec.link_length_body();
		s_cal = s + Moving_distance;
		Integral_joint_angle = 0;
		Num = 0;
		while(s_cal > S_T){
			t_ = t_ + 0.01;
			S_T = RungeKutta(S_T, t_, t_+0.01, 1000);	// ルンゲクッタ法(初期条件x0, 区間[t_, t_+0.1], 分割数100-> 0.1/100 ->0.001
		}
			topic_t_.data = t_;
			topic_phi_.data = pre_phi_;
			topic_s_.data = s_cal;
			topic_moving_distance_.data = Moving_distance;
			ros_parameter_pub_t.publish(topic_t_);
			ros_parameter_pub_phi.publish(topic_phi_);
			ros_parameter_pub_s.publish(topic_s_);
			ros_parameter_pub_moving_distance.publish(topic_moving_distance_);
    	// Write_file_to_hard_disk(t_,"t.txt",&file_3);
			joint_angle[i_joint-1] = 0;
			 if((t_+0.25)<(omega_hyperboli_*pre_phi_/omega_) || Flag_Execution_Initialization)
			 {
				joint_angle[i_joint] = Integral_joint_angle;

			 }
			joint_angle_backup[i_joint] = joint_angle[i_joint];//Backup this Angle

  }
	Flag_Execution_Initialization = false;
	Now_length_t_ = t_;
	if(Flag_Now_length_t_ == true){First_Now_length_t_ = Now_length_t_;Flag_Now_length_t_ = false;}

	Write_file_to_hard_disk(s,"s.txt",&file_4);
	//ROS_INFO("* s = [%4.3f] *",s);
	for(uint32_t i_joint=1; i_joint<spec.num_joint(); i_joint++) {
		if(i_joint%2)
		ROS_INFO("* AAAAAjoint_angle[%d] = [%4.3f] *",i_joint,joint_angle[i_joint]);
		else
		ROS_INFO("*                                AAAAAjoint_angle[%d] = [%4.3f] *",i_joint,joint_angle[i_joint]);
	}

	topic_angle_1_.data = joint_angle[0];
	topic_angle_2_.data = joint_angle[1];
	topic_angle_3_.data = joint_angle[2];
	topic_angle_4_.data = joint_angle[3];
	topic_angle_5_.data = joint_angle[4];
	topic_angle_6_.data = joint_angle[5];
	ros_parameter_pub_angle_1.publish(topic_angle_1_);
	ros_parameter_pub_angle_2.publish(topic_angle_2_);
	ros_parameter_pub_angle_3.publish(topic_angle_3_);
	ros_parameter_pub_angle_4.publish(topic_angle_4_);
	ros_parameter_pub_angle_5.publish(topic_angle_5_);
	ros_parameter_pub_angle_6.publish(topic_angle_6_);
  return joint_angle;
}

//y'=sqrt((x')^2+(y')^2)を求める
double SShapedLongitudnalWave::dtds(double t, double s)
{
	double x1=0, y1=0;

	double curve_period_ = wavelength_/moving_speed_;//[s]
	//bbbbbbbbbbbbb
	double a_     = 0.3*radius_;//Hyperbolic amplitude
	double phi_   = pre_phi_;//Hyperbolic phase
	double omega_ = 2*pi/curve_period_;//Angular frequency
	double omega_hyperboli_ = 4*pi/curve_period_;//Angular frequency
	omega_ = 2*pi/wavelength_;//Angular frequency
	// phi_ = 0.5*Now_length_t_;
	double num   = 0, denom = 0;
	phi_  = -1*phi_;

	//1 回微分
	x1 = 1;
	y1 = omega_*cos(omega_*t)*(a_*pow(cosh(omega_*t+omega_hyperboli_*
	    phi_),-1)+radius_)-a_*omega_*sin(omega_*t)*pow(cosh(omega_*
	    t+omega_hyperboli_*phi_),-2)*sinh(omega_*t+omega_hyperboli_*phi_);
	double j=pow(x1,2) + pow(y1,2);
	return sqrt(j);
}
double SShapedLongitudnalWave::RungeKutta(double s0, double t0, double tn, int n)
{
    int i;
    double s, t, h, k1, k2, k3, k4,Temp_kappa;
    s = s0;
    t = t0;
    h = (tn - t0) /n;
		ROS_INFO("* Integral_joint_angle = [%4.3f] *", Integral_joint_angle);

    Temp_kappa = CalcKappa((tn + t0)/2);

    // 漸化式を計算
    for ( i=1; i <= n ; i++){
        t = t0 + i*h;
        k1 = dtds(t,s);
        k2 = dtds(t,s + k1*h*0.5);
        k3 = dtds(t,s + k2*h*0.5);
        k4 = dtds(t,s + k3*h);
        s += (k1 + 2 * k2 + 2 * k3 + k4)*(h/6.0);
				// Num ++;
				//if(i == 1) Integral_joint_angle = 	1*s*Temp_kappa;
			 	//else
				//Temp_kappa = CalcKappa(t);
				Integral_joint_angle += 	1*((k1 + 2 * k2 + 2 * k3 + k4)*(h/6.0))*Temp_kappa;

    }
	return s;
}
/*
 * @fn
 * @brief
 * @param
 * @paran
 * @return なし
 * @detail
*/
void SShapedLongitudnalWave::CalculateTargetAngleForHyperbola(RobotSpec spec)
{
	snake_model_param.angle.clear();

	 for(int i=0; i<num_link_; i++){
	//for(int i=0; i<1; i++){
		if(i%2){  /*  (奇数番目) pitch joints?   */
			target_angle_ = 0;          //-2*link_length_*snake_model_param.kappa[i]*sin(snake_model_param.psi[i]);
										//横うねり推進ではroll 軸の回転はしないため，ψ(s)＝０になる．//そのため，計算上はpis_を消しでもいい．
										//また，平面運動ので   PITCH軸は動かないのでPICHT軸の目標角度を０にした．
		}else{    /* (偶数番目)   */
			target_angle_ =
					// 2*link_length_*snake_model_param.kappa[i] + snake_model_param.bias[i]; //*cos(snake_model_param.psi[i]);
					2*link_length_*snake_model_param.kappa[i]; //*cos(snake_model_param.psi[i]);
		}
		snake_model_param.angle.push_back(target_angle_);
	}
}
