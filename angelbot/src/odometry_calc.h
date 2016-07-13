#ifndef Odom_calc_h
#define Odom_calc_h
#endif

class Odometry_calc{

public:
	Odometry_calc();
	void spin();
private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Subscriber left_wheel_sub;
	ros::Subscriber right_wheel_sub;
	ros::Publisher odom_pub;

	tf::TransformBroadcaster odom_broadcaster;
	//Encoder related variables
	double encoder_min;
	double encoder_max;

	double encoder_low_wrap;
	double encoder_high_wrap;

	double prev_left_encoder;
	double prev_right_encoder;

	double leftmult;
	double rightmult;

	double left;
	double right; //

	double rate; //

	ros::Duration t_delta;

	ros::Time t_next;

	ros::Time then;


	double enc_left ;

	double enc_right;

	double ticks_meter; //ticks per meter?

	double base_width;

	double dx;

	double dr;

	double x_final,y_final, theta_final;

	ros::Time current_time, last_time;


	void leftencoderCb(const std_msgs::Int64::ConstPtr& left_ticks);

	void rightencoderCb(const std_msgs::Int64::ConstPtr& right_ticks);

	void init_variables();

	void get_node_params();


	void update();
};
