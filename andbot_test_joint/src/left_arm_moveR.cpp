#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <sensor_msgs/Joy.h>
#include <stdlib.h>
#define PI 3.14159
#define LEFT_ARM 1
#define RIGHT_ARM 0

struct point{
  std_msgs::Float64 x;
  std_msgs::Float64 y;
  std_msgs::Float64 z;
};

struct Arm_Angle{
  geometry_msgs::Vector3 th_0;
  geometry_msgs::Vector3 th_1;
  geometry_msgs::Vector3 th_2;
  geometry_msgs::Vector3 th_3;
};

void v_scalar_multip(double s,double a[],double c[],int n)
{
  for (int i=0;i<n;i++)
  {
    c[i]=s*a[i];
  }

  return ;
}
double v_dot(double a[],double b[],int n)
{
  double sum=0;
  for(int i=0;i<n;i++)
  {
    sum=sum+a[i]*b[i];

   //ROS_INFO("a[i]= %f\n",a[i]);
   //ROS_INFO("b[i]= %f\n",b[i]);

  }
  return sum;
}
 
void v_sub(double a[],double b[],double c[],int n)
{
  for(int i=0;i<n;i++)
  {
    c[i]=a[i]-b[i];
  }
  return;
}
void v_add(double a[],double b[],double c[],int n)
{
  for(int i=0;i<n;i++)
  {
    c[i]=a[i]+b[i];
  }
  return;
}
void v_cross(double a[],double b[],double c[])
{
 
  c[0]=a[1]*b[2]-a[2]*b[1];
  c[1]=a[2]*b[0]-a[0]*b[2];
  c[2]=a[0]*b[1]-a[1]*b[0];

  return;
}
void v_print(double a[],int n)
{
  for(int i=0;i<n;i++)
  {
    ROS_INFO("p = %f\n",a[i]);
  }
  return;
}

class MoveR
{
public:
  double TH0_MAX,TH0_MIN,TH1_MAX,TH1_MIN,TH3_MAX,TH3_MIN;
  point Target;
  Arm_Angle arm_angle;
  double lup,ldn;
  void goalCallback(const geometry_msgs::Pose& msg);
  MoveR();
  void IK_loop();
  void IK4(const point& P);
  double move_time;
  double fi;
  ros::NodeHandle nh_;
  ros::Publisher axis_0_pub,axis_1_pub,axis_2_pub,axis_3_pub;
  ros::Subscriber goal_sub; 
};

 void MoveR::goalCallback(const geometry_msgs::Pose& pose){
  //ROS_INFO("subscribe\n");
  Target.x.data=pose.position.x;
  Target.y.data=pose.position.y;
  Target.z.data=pose.position.z;
  fi=pose.orientation.x;
  move_time=pose.orientation.y;

}


  MoveR::MoveR()
{
  TH0_MAX=PI;
  TH0_MIN=-1.57;
  if(LEFT_ARM){
    TH1_MAX=1.57;
    TH1_MIN=-0.01;
  }else if(RIGHT_ARM)
  {
     TH1_MAX=0.01;
      TH1_MIN=-1.57;
  }
  TH3_MAX=1.57;
  TH3_MIN=-0.01;

  move_time=10;
  fi=0;
  lup=235,ldn=250;    

  arm_angle.th_0.x=0.0;
  arm_angle.th_0.y=0.0;

  arm_angle.th_1.x=0.0;
  arm_angle.th_1.y=0.0;

  arm_angle.th_2.x=0.0;
  arm_angle.th_2.y=0.0;

  arm_angle.th_3.x=0.0;
  arm_angle.th_3.y=0.0;

  Target.x.data=0.0,Target.y.data=0.0,Target.z.data=0.0;
  if(LEFT_ARM){
    axis_0_pub = nh_.advertise<geometry_msgs::Vector3>("andbot/joint/L0/cmd/position", 1);
    axis_1_pub = nh_.advertise<geometry_msgs::Vector3>("andbot/joint/L1/cmd/position", 1);
    axis_2_pub = nh_.advertise<geometry_msgs::Vector3>("andbot/joint/L2/cmd/position", 1);
    axis_3_pub = nh_.advertise<geometry_msgs::Vector3>("andbot/joint/L3/cmd/position", 1);
    goal_sub = nh_.subscribe("/andbot/left_arm/goal", 1000, &MoveR::goalCallback,this);
  }
  if(RIGHT_ARM){
    axis_0_pub = nh_.advertise<geometry_msgs::Vector3>("andbot/joint/R0/cmd/position", 1);
    axis_1_pub = nh_.advertise<geometry_msgs::Vector3>("andbot/joint/R1/cmd/position", 1);
    axis_2_pub = nh_.advertise<geometry_msgs::Vector3>("andbot/joint/R2/cmd/position", 1);
    axis_3_pub = nh_.advertise<geometry_msgs::Vector3>("andbot/joint/R3/cmd/position", 1);
    goal_sub = nh_.subscribe("/andbot/right_arm/goal", 1000, &MoveR::goalCallback,this);
  }

}

void MoveR::IK4(const point& P)
{
  //ROS_INFO("IK\n");
  double th0=0,th1=0,th2=0,th3=0;
  double x=P.x.data;
  double y=P.y.data;
  double z=P.z.data;
  double p[3]={x,y,z};
  double temp1=pow(x,2)+pow(y,2)+pow(z,2);
  double norm_p=pow(temp1,0.5);
  double b=acos((pow(lup,2)+pow(ldn,2)-pow(norm_p,2))/(2*lup*ldn));
  th3=PI-b;  
  double a=asin((ldn/norm_p)*sin(b));
  double s[3]={0,0,0};
  double u_z[3]={0,0,1};
  double p_s[3];
  v_sub(p,s,p_s,3);
  //ROS_INFO("p\n");
  //v_print(p,3);	
  //ROS_INFO("s\n");
  //v_print(s,3);	
  //ROS_INFO("p-s\n");
  //v_print(p_s,3);	

  double norm_p_s=pow(pow(p_s[0],2)+pow(p_s[1],2)+pow(p_s[2],2),0.5);
  double u_n[3]={p_s[0]/norm_p_s,p_s[1]/norm_p_s,p_s[2]/norm_p_s};
  
  //ROS_INFO("u_n\n");
  //v_print(u_n,3);	
  double tmp[3];
  v_scalar_multip(v_dot(u_z,u_n,3),u_n,tmp,3);
  double u[3],u_u[3];
  //v_add(u_z,tmp,u,3);

  //u correct on 2/17
  v_cross(u_n,u_z,u);

  double norm_u=pow(pow(u[0],2)+pow(u[1],2)+pow(u[2],2),0.5);
  v_scalar_multip(1/norm_u,u,u_u,3);

  //ROS_INFO("u_u \n");
  //v_print(u_u,3);	
  double v[3],u_v[3];
  v_cross(u_n,u_u,v);
  double norm_v=pow(pow(v[0],2)+pow(v[1],2)+pow(v[2],2),0.5);
  v_scalar_multip(1/norm_v,v,u_v,3);

  //ROS_INFO("u_v \n");
  //v_print(u_v,3);	

  double c[3];
  v_scalar_multip(cos(a)*lup,u_n,tmp,3);
  v_add(s,tmp,c,3);
  double r=lup*sin(a);
  //double fi=PI/2;
  
  //ROS_INFO("a %f\n",a);
  double tmp1[3],tmp2[3];
  v_scalar_multip(cos(fi),u_u,tmp1,3);
  v_scalar_multip(sin(fi),u_v,tmp2,3);
  v_add(tmp1,tmp2,tmp,3);
  v_scalar_multip(r,tmp,tmp,3);
  double e[3];
  v_add(c,tmp,e,3);
  //ROS_INFO("e \n");
  //v_print(e,3);	

  double ex=e[0],ey=e[1],ez=e[2];
  //ROS_INFO("ex %f\n",ex);
  //ROS_INFO("ey %f\n",ey);
  //ROS_INFO("ez %f\n",ez);

  if(ex>=0&&ey<0)
    th0=atan((double)abs(ex)/abs(ey));
  else if(ex>=0&&ey>=0)
      th0=PI-atan((double)abs(ex)/abs(ey));
    else if(ex<0&&ey>=0)
        th0=PI+atan((double)abs(ex)/abs(ey));
      else if(ex<0&&ey<0)
          th0=-atan((double)abs(ex)/abs(ey));
  double temp2=pow(pow(ex,2)+pow(ey,2),0.5);
  if(ez>=0)
    th1=-atan((double)abs(ez)/temp2);
  else if(ez<0)
    th1=atan((double)abs(ez)/temp2);

   //ROS_INFO("IK:th0 %f\n",th0);
   //ROS_INFO("IK:th1 %f\n",th1);
   //ROS_INFO("th3 %f\n",th3);


  th2=atan2(-x*sin(th0)*sin(th1)+y*cos(th0)*sin(th1)-z*cos(th1),x*cos(th0)+y*sin(th0));
  ROS_INFO("x %f  y %f  z %f\n",x,y,z);

  ROS_INFO("th0 %f  th1 %f  th2 %f  th3 %f  fi% f\n",th0,th1,th2,th3,fi);

  if(isnan(th0)||isnan(th1)||isnan(th2)||isnan(th3)||th0>TH0_MAX||th0<TH0_MIN||th1>TH1_MAX||th1<TH1_MIN||th3>TH3_MAX||th3<TH3_MIN){
    ROS_INFO("Fail and dont move\n");
   }
  else{
    ROS_INFO("go move move time=%f\n",move_time);
    arm_angle.th_0.x=th0;
    arm_angle.th_0.y=move_time;
    arm_angle.th_1.x=th1;
    arm_angle.th_1.y=move_time;
    arm_angle.th_2.x=th2;
    arm_angle.th_2.y=move_time;
    arm_angle.th_3.x=th3;
    arm_angle.th_3.y=move_time;

  }
}


int main(int argc, char** argv)
{
  if(LEFT_ARM)
    ros::init(argc, argv, "moveR_left_arm");
  else if(RIGHT_ARM)
      ros::init(argc, argv, "moveR_right_arm");

  MoveR move_R;
  move_R.IK_loop();
  return(0);
}


void MoveR::IK_loop()
{


  ros::Rate r(2);

  while(nh_.ok())
  {
    ros::spinOnce();
    IK4(Target);
    axis_0_pub.publish(arm_angle.th_0);
    axis_1_pub.publish(arm_angle.th_1);
    axis_2_pub.publish(arm_angle.th_2);
    axis_3_pub.publish(arm_angle.th_3);

    r.sleep();
  }


  return;
}


