#include <ros/ros.h>                    // for ros
#include <geometry_msgs/Twist.h>        // for cmd_vel
#include <ypspur.h>		        // for yp-spur
#include <cmath>	 	        // for math PI

class NishidalabYpspurDriver
{
public:
  NishidalabYpspurDriver();
  ~NishidalabYpspurDriver();
  
private:
  // Call back
  void Callback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
  // subscriber
  ros::Subscriber cmd_vel_sub;

  // set node handler
  ros::NodeHandle n;
  // linear & angular limits
  double linear_vel_max;
  double angular_vel_max;
  double linear_acc_max;
  double angular_acc_max;

};

NishidalabYpspurDriver::NishidalabYpspurDriver() : 
  linear_vel_max(1.1),
  angular_vel_max(M_PI),
  linear_acc_max(1.0),
  angular_acc_max(M_PI)
{
  // init yp-spur---------------------------------------------------------------------
  if( Spur_init() < 0)
    ROS_ERROR("can't open ypspur");

  // using parameter server
  n.param("nishidalab_ypspur/linear_vel_max", linear_vel_max, linear_vel_max);
  n.param("nishidalab_ypspur/angular_vel_max", angular_vel_max, angular_vel_max);
  n.param("nishidalab_ypspur/linear_acc_max", linear_acc_max, linear_acc_max);
  n.param("nishidalab_ypspur/angular_acc_max", angular_acc_max, angular_acc_max);
  // init velocity & accelaration limits (Unit is m/s & rad/s)
  Spur_set_vel(linear_vel_max);
  Spur_set_accel(linear_acc_max);
  Spur_set_angvel(angular_vel_max);
  Spur_set_angaccel(angular_acc_max);
  // end init yp-spur------------------------------------------------------------------

  cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &NishidalabYpspurDriver::Callback, this);

}

NishidalabYpspurDriver::~NishidalabYpspurDriver()
{
  Spur_stop();
  Spur_free();
  ROS_INFO("Stop the robot");

}

void NishidalabYpspurDriver::Callback(const geometry_msgs::Twist::ConstPtr& cmd_vel){
  Spur_vel(cmd_vel->linear.x, cmd_vel->angular.z);
}
