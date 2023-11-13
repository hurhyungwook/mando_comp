#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"

#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60) 
#define RPS2RPM(x) ((x)*60) 

float robot_wheel_base =  1.1;  // 단위 m
float steering_cmd_angular;
float steering_cmd_linear;
int robot_steering_angle = 0;

void cmd_velCallback(const geometry_msgs::Twist& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  
  float radius = 0.0;
  steering_cmd_angular = msg.angular.z;
  steering_cmd_linear  = msg.linear.x ;
  
  radius = msg.linear.x/msg.angular.z;
  robot_steering_angle = (int)(RAD2DEG(atan2(radius,robot_wheel_base)) + 0.5); 
     
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "cmd_vel_steer_angle");

  ros::NodeHandle n;
  /* robot param set */
  ros::param::get("~wheel_base", robot_wheel_base);
  
  /* Subscriber define */
  ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel",100,&cmd_velCallback);
  
  /* Publisher define */
  ros::Publisher steering_angle_pub = n.advertise<std_msgs::Int16>("/Steering_angle_Int16",1);
  
  
  ros::Rate loop_rate(10);
  
  std_msgs::Int16 steering_angle; 
  
  while(ros::ok())
  {
	 steering_angle.data =  robot_steering_angle;
	 steering_angle_pub.publish(steering_angle);
	  
	 ros::spinOnce();
     loop_rate.sleep(); 
  }
  
  
  
  return 0;
}  

