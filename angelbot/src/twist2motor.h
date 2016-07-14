#ifndef twist2motor_h
#define twist2motor_h
#endif

class TwistToMotors
{

public:
	TwistToMotors();
	void spin();

private:
	ros::NodeHandle n;
	ros::Publisher cmd_wheel_angularVel_pub;
	ros::Subscriber cmd_vel_sub;

	double rate;

	double wheelRadius;
	double wheelSeparation;

	double leftwheel_angularVel;
	double rightwheel_angularVel;
	double vel_x;
	double vel_th;

	float ticks_since_target;
	double timeout_ticks;

	void init_variables();
	void get_parameters();

	void spinOnce();
	void twistCallback(const geometry_msgs::Twist &twist_aux);
};
