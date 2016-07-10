/**
 *
 * @param argc
 * @param argv
 * @return
 */
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double inc= 0.005, base2shaft_inc= 0.005, shaft2leftwheel_inc= 0.005, shaft2rightwheel_inc= 0.005;
    double angle= 0 ,base2shaft = 0, shaft2leftwheel = 0, shaft2rightwheel = 0;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    tf::Transform world_trans;
    tf::Quaternion world_q;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "axis";

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(3);
        joint_state.position.resize(3);
        joint_state.name[0] ="base2shaft";
        joint_state.position[0] = base2shaft;
        joint_state.name[1] ="shaft2leftwheel";
        joint_state.position[1] = shaft2leftwheel;
        joint_state.name[2] ="shaft2rightwheel";
        joint_state.position[2] = shaft2rightwheel;

        // update transform
        // (moving in a circle with radius=2)
//        odom_trans.header.stamp = ros::Time::now();
//        odom_trans.transform.translation.x = cos(angle)*2;
//        odom_trans.transform.translation.y = sin(angle)*2;
//        odom_trans.transform.translation.z = .7;
//        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

//        world_trans.setOrigin(tf::Vector3(0,0,0.0));
//        world_q.setRPY(0,0,0);
//        world_trans.setRotation(world_q);

        //send the joint state and transform
        joint_pub.publish(joint_state);
//        broadcaster.sendTransform(odom_trans);
        //broadcaster.sendTransform(tf::StampedTransform(world_trans,ros::Time::now(),"world","baselink"));

        // Create new robot state
        shaft2leftwheel += shaft2leftwheel_inc;
        if (shaft2leftwheel<-.5 || shaft2leftwheel>0) shaft2leftwheel_inc *= -1;
        shaft2rightwheel += shaft2rightwheel_inc;
        if (shaft2rightwheel>.2 || shaft2rightwheel<0) shaft2rightwheel_inc *= -1;
        base2shaft += degree;
//        angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
