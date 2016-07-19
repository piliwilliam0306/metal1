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

	double left_omega;
	double right_omega;
	double vel;
	double omega;

	float ticks_since_target;
	double timeout_ticks;

	void init_variables();
	void get_node_params();

	void spinOnce();
	void twistCallback(const geometry_msgs::Twist &twist_aux);
};
