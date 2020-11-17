/*
 * s_shaped_longitudnal_wave.h
 *
 *  Created on: Dec 30, 2018
 *      Author: Wang Yongdong
 */

#ifndef SNAKE_CONTROL_SRC_S_SHAPED_LONGITUDNAL_WAVE_H_
#define SNAKE_CONTROL_SRC_S_SHAPED_LONGITUDNAL_WAVE_H_

#include <ros/ros.h>
#include <vector>
#include <stdint.h>

#include "robot_spec.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "shift_control_method.h"

class SShapedLongitudnalWave: public ShiftControlMethod {
 public:
	virtual ~SShapedLongitudnalWave(){}
	SShapedLongitudnalWave(RobotSpec spec,
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
    serpenoid_curve.phi   = 0.00;

		s_ 	= 0;
    s_0_ = 0;//双曲線関数使う変数
    t_ = 0;
		S_T 	= 0;
		dt_ 	= 0.010;  // サンプリングタイム10[msec]

		pre_s_  = 0;
    pre_s_0_ = 0;
		step_s_ = ds/28;

		psi_ 	= 0;
		psi_hyper_ = 0;

		tau_   = 0;
		kappa_ = 0;
    kappa_0_ = 0;
		bias_  = 0;
    flag_ 	= true;

    pi = 3.14159;
    moving_speed_ = 0.5;//[m/s]The moving speed of S
    link_wide_ = 0.06;//[m]
    pipe_d_ = 0.0891;//[m] Pipe diameter
    pipe_s_ = 0.1325; //[m] Piping spacing
    wavelength_ = pipe_d_*2 + pipe_s_*2 - 0.05;  //[m]
    //radius_ = pipe_d_/2 + link_wide_/2 + 0.035;  //[m] 0.075
    radius_ = pipe_d_/2 + link_wide_/2 + 0.015;  //[m] 0.075
		Init(spec);
	}
  static void Initialize() {

    ros::NodeHandle node_param;// 声明节点句柄
    ros_parameter_pub_t = node_param.advertise<std_msgs::Float64 >("ros_param_t", 100);
    ros_parameter_pub_phi = node_param.advertise<std_msgs::Float64 >("ros_param_phi", 100);
    ros_parameter_pub_s = node_param.advertise<std_msgs::Float64 >("ros_param_s", 100);
    ros_parameter_pub_moving_distance = node_param.advertise<std_msgs::Float64 >("ros_parame_moving_distance", 100);

    ros_parameter_pub_angle_1 = node_param.advertise<std_msgs::Float64 >("ros_param_angle_1", 100);
    ros_parameter_pub_angle_2 = node_param.advertise<std_msgs::Float64 >("ros_param_angle_2", 100);
    ros_parameter_pub_angle_3 = node_param.advertise<std_msgs::Float64 >("ros_param_angle_3", 100);
    ros_parameter_pub_angle_4 = node_param.advertise<std_msgs::Float64 >("ros_param_angle_4", 100);
    ros_parameter_pub_angle_5 = node_param.advertise<std_msgs::Float64 >("ros_param_angle_5", 100);
    ros_parameter_pub_angle_6 = node_param.advertise<std_msgs::Float64 >("ros_param_angle_6", 100);

}
	//--- 動作
	void SShapedLongitudnalWaveByShift(RobotSpec spec);
	void CalculateTargetAngleForHyperbola(RobotSpec spec);
	void CalculateCurvature();
  double CalcKappa(double t);
  std::vector<double> CalculateTargetAngle(RobotSpec spec);
  double dtds(double t, double s);
  double RungeKutta(double s0, double t0, double tn, int n);
	virtual void InitializeShape() {}

	//--- 形状パラメータ変更
	void set_alpha(double alpha);
	void add_alpha(double alpha_add){ set_alpha(serpenoid_curve.alpha + alpha_add); }
	void set_l(double l);
	void add_l(double l_add){ set_l(serpenoid_curve.l + l_add); }

  void set_wavelength(double wavelength);
	void add_wavelength(double wavelength_add){ set_wavelength(wavelength_ + wavelength_add); }
	void set_radius(double radius);
	void add_radius(double radius_add){ set_radius(radius_ + radius_add); }

	void set_bias(double bias);
	void add_bias(double bias){ set_bias(bias); }
	void set_v(double v);
  void set_phi(RobotSpec spec,double phi);
	void add_v(double v_add){ set_v(v_add); }
  void add_phi(RobotSpec spec,double phi_add){ set_phi(spec,phi_add); }

	void set_flag_on();
	void set_flag_off();

	void print_parameters();
  void Write_file_to_hard_disk(double data,const std::string &name,bool *flag);

	double s_,s_0_, dt_,t_,Now_length_t_,First_Now_length_t_;
	double pre_s_,pre_s_0_,pre_phi_;
	double step_s_;
	double target_angle_;
	int num_link_;
	double link_length_;
	double S_T;
  bool flag_;

  double pi;
  double moving_speed_;//[m/s]The moving speed of S
  double link_wide_;//[m]
  double pipe_d_;//[m] Pipe diameter
  double pipe_s_; //[m] Piping spacing
  double wavelength_;  //[m]
  double radius_;  //[m]

  private:
  //--- Publisher ---//
  static ros::Publisher ros_parameter_pub_t;
  static ros::Publisher ros_parameter_pub_phi;
  static ros::Publisher ros_parameter_pub_s;
  static ros::Publisher ros_parameter_pub_moving_distance;

  static ros::Publisher ros_parameter_pub_angle_1;
  static ros::Publisher ros_parameter_pub_angle_2;
  static ros::Publisher ros_parameter_pub_angle_3;
  static ros::Publisher ros_parameter_pub_angle_4;
  static ros::Publisher ros_parameter_pub_angle_5;
  static ros::Publisher ros_parameter_pub_angle_6;


};

#endif /* SNAKE_CONTROL_SRC_S_SHAPED_LONGITUDNAL_WAVE_H_ */
