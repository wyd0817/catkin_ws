

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "robot_spec.h"
#include "shift_control_method.h"

class SinusLifting: public ShiftControlMethod {
 public:
	virtual ~SinusLifting(){}
	SinusLifting(RobotSpec spec,
			double ds
			){

		ds_ = ds;

		num_link_ 	  = spec.num_joint();
		link_length_  = spec.link_length_body();
		target_angle_ = 0;


		// serpenoid_curve.alpha = [2.4 rad] *
		// serpenoid_curve.l     = [2.5 m  ] *

		serpenoid_curve.alpha =2.40;         // くねり角[rad]
		serpenoid_curve.l     =2.50;         // 曲線の1/4周期の長さ[m]
		serpenoid_curve.v     =0.00;

		s_ 		= 0;
		S_T 	= 0;
		t_ 		= 0;

		pre_s_  = 0;
		step_s_ = ds/28;

		psi_ 	= 0;
		psi_hyper_ = 0;

		flag_ 	= false;

		tau_   = 0;
		kappa_ = 0;
		bias_  = 0;
		Init(spec);

		Init(spec);
	}

    void init();
    void SinusLiftingByShift(RobotSpec spec);
    void CalculateCurvature();
    void CalculateTargetAngle(RobotSpec spec);

	//--- 形状パラメータ変更
	void set_alpha(double alpha);
	void add_alpha(double alpha_add){ set_alpha(serpenoid_curve.alpha + alpha_add); }
	void set_l(double l);
	void add_l(double l_add){ set_l(serpenoid_curve.l + l_add); }
	void set_v(double v);
	void add_v(double v_add){ set_v(v_add); }

	void print_parameters();

	double target_angle_;
	int num_link_;
	double link_length_;

	double s_, 	t_ ;
	double pre_s_;
	double step_s_;
	double S_T;
	bool flag_;

};

