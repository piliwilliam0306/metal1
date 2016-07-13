#ifndef basectrl_angelbot_h
#define basectrl_angelbot_h
#endif

class basectrl_angelbot
{

public:
	basectrl_angelbot();
	void spin();

private:
	ros::NodeHandle n;

	ros::Publisher cmd_wheel_angularVel_pub;
	ros::Subscriber cmd_vel_sub;

	float left;
	float right;

	float ticks_since_target;

	double wheelRadius;
	double wheelSeparation;

	float dx,dy,dr;

	void init_variables();
	void get_parameters();

	void spinOnce();
	void cmd_velCallback(const geometry_msgs::Twist &twist_aux);
};
