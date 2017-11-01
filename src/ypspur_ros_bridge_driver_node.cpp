#include <ros/ros.h>                    // for ros
#include "ypspur_ros_bridge/ypspur_ros_bridge_driver_node.hpp"


int main(int argc, char** argv){

  // init ros
  ros::init(argc, argv, "ypspur_ros_bridge_driver");

  //original class
  YpspurROSBridgeDriver YRBDriver;
  
  ros::spin();
}

