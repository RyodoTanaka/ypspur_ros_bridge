#include <ros/ros.h>                    // for ros
#include <geometry_msgs/Twist.h>	//for cmd_vel
#include <ypspur.h>		        // for yp-spur
#include <cmath>	 	        // for math PI
#include "nishidalab_ypspur_driver/nishidalab_ypspur_driver.hpp"


int main(int argc, char** argv){

  // init ros
  ros::init(argc, argv, "nishidalab_ypspur_driver");

  //original class
  NishidalabYpspurDriver NlabYpspurDriver;
  
  ros::spin();
}

