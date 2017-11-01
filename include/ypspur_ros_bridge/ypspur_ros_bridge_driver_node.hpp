#include <ros/ros.h>                    // for ros
#include <geometry_msgs/Twist.h>        // for cmd_vel
#include <ypspur.h>		        // for yp-spur
#include <cmath>	 	        // for math PI

class YpspurROSBridgeDriver
{
public:
  YpspurROSBridgeDriver();
  ~YpspurROSBridgeDriver();
  
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

YpspurROSBridgeDriver::YpspurROSBridgeDriver() : 
  linear_vel_max(1.1),
  angular_vel_max(M_PI),
  linear_acc_max(1.0),
  angular_acc_max(M_PI)
{
  ros::NodeHandle nh("~");
  
  // init yp-spur---------------------------------------------------------------------
  if( Spur_init() < 0)
    ROS_ERROR("can't open ypspur");

  // using parameter server
  nh.param("linear_vel_max", linear_vel_max, linear_vel_max);
  nh.param("angular_vel_max", angular_vel_max, angular_vel_max);
  nh.param("linear_acc_max", linear_acc_max, linear_acc_max);
  nh.param("angular_acc_max", angular_acc_max, angular_acc_max);
  // init velocity & accelaration limits (Unit is m/s & rad/s)
  Spur_set_vel(linear_vel_max);
  Spur_set_accel(linear_acc_max);
  Spur_set_angvel(angular_vel_max);
  Spur_set_angaccel(angular_acc_max);
  // end init yp-spur------------------------------------------------------------------

  cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &YpspurROSBridgeDriver::Callback, this);

}

YpspurROSBridgeDriver::~YpspurROSBridgeDriver()
{
  Spur_stop();
  Spur_free();
  ROS_INFO("Stop the robot");

}

void YpspurROSBridgeDriver::Callback(const geometry_msgs::Twist::ConstPtr& cmd_vel){
  Spur_vel(cmd_vel->linear.x, cmd_vel->angular.z);
}
