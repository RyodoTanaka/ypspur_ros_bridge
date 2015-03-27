#include <ros/ros.h>                    // for ros
#include <tf/transform_broadcaster.h>	// for tf
#include <nav_msgs/Odometry.h>	        //for odometry
#include <ypspur.h>		        // for yp-spur
#include <cmath>	 	        // for math PI

class NishidalabYpspurOdomPublisher
{
public:
  NishidalabYpspurOdomPublisher();

  // odometry pose infomation
  double x;
  double y;
  double th;

  // velocity twist information
  double vx;
  double vy;
  double vth;

  // set node handler
  ros::NodeHandle n;
  
  // set odometory publisher
  ros::Publisher odom_pub; 
  // set tf broad caster
  tf::TransformBroadcaster odom_broadcaster;
  // ros timer
  ros::Time current_time;
  
private:
};

NishidalabYpspurOdomPublisher::NishidalabYpspurOdomPublisher() :
  x(0.0),
  y(0.0),
  th(0.0),
  vx(0.0),
  vy(0.0),
  vth(0.0)
{
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  current_time = ros::Time::now();
  
  if(Spur_init() < 0)
    ROS_ERROR("Can't open spur");
}
