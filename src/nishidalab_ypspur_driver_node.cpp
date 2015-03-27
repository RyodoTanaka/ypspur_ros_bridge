#include <ros/ros.h>                    // for ros
#include "nishidalab_ypspur_driver/nishidalab_ypspur_driver_node.hpp"


int main(int argc, char** argv){

  // init ros
  ros::init(argc, argv, "nishidalab_ypspur_driver");

  //original class
  NishidalabYpspurDriver NlabYpspurDriver;
  
  ros::spin();
}

