#ifndef basectrl_angelbot_h
#define basectrl_angelbot_h
#endif

class basectrl_angelbot
{

public:
	basectrl_angelbot();
	void spin();

private:
	ros::NodeHandle n1,n2;

	ros::Publisher cmd_wheel_angularVel_pub;
	ros::Subscriber cmd_vel_sub;

	double rate;

	double wheelRadius;
	double wheelSeparation;

	double leftwheel_angularVel;
	double rightwheel_angularVel;

	double vel_x;
	double vel_th;

	void init_variables();
	void get_parameters();

	void spinOnce();
	void cmd_velCallback(const geometry_msgs::Twist &twist_aux);
};
