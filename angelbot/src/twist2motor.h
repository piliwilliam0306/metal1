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

	ros::Publisher pub_left_motor;
	ros::Publisher pub_right_motor;
	ros::Subscriber cmd_vel_sub;

	float left;
	float right;

	float ticks_since_target;
	double timeout_ticks;

	double w;
	double rate;

	float dx,dy,dr;

	void init_variables();
	void get_parameters();

	void spinOnce();
	void twistCallback(const geometry_msgs::Twist &twist_aux);
};
