#ifndef Odom_calc_h
#define Odom_calc_h
#endif

class Odometry_calc{

public:
	Odometry_calc();
	void spin();
private:
	ros::NodeHandle n;
	ros::Subscriber feedback_wheel_angularVel_sub;
	ros::Publisher odom_pub;

	tf::TransformBroadcaster odom_broadcaster;

	double rate; //

	ros::Duration t_delta;
	ros::Time current_time, last_time, t_next;

	double left_omega;
	double right_omega;

	double wheelSeparation;
	double wheelRadius;

	double vel;
	double omega;

	double disXdir,disYdir,disTheta;

	void feedback_wheel_angularVelCallback(const angelbot::WheelFb &msg);
	void init_variables();
	void get_node_params();
	void update();
};
