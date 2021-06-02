#include <ros/ros.h>                    // for ros
#include <tf/transform_broadcaster.h>	// for tf
#include <nav_msgs/Odometry.h>	        //for odometry
#include <ypspur.h>		        // for yp-spur
#include "ypspur_ros_bridge/ypspur_ros_bridge_odom_publisher_node.hpp"  // include local header

int main(int argc, char** argv){

  // init ros
  ros::init(argc, argv, "ypspur_ros_bridge_odom_publisher");
  
  // original class
  YpspurROSBridgeOdomPublisher YRBOdomPub;

  // for quaternion
  geometry_msgs::Quaternion odom_quat;
  // for transform tf
  geometry_msgs::TransformStamped odom_trans;
  // for pusblish odometry
  nav_msgs::Odometry odom;
  // for publish joint_state
  sensor_msgs::JointState js;


  // initialize messages
  YRBOdomPub.current_time = ros::Time::now();

  odom_trans.header.stamp = YRBOdomPub.current_time;
  odom_trans.header.frame_id = YRBOdomPub.frame_id;
  odom_trans.child_frame_id = YRBOdomPub.child_frame_id;

  odom.header.stamp = YRBOdomPub.current_time;
  odom.header.frame_id = YRBOdomPub.frame_id;
  odom.child_frame_id = YRBOdomPub.child_frame_id;
 
  js.name.push_back(YRBOdomPub.left_wheel_joint);
  js.name.push_back(YRBOdomPub.right_wheel_joint);
  js.position.resize(2);
  
  // loop rate is 25.0 [Hz]
  ros::Rate r(25.0);

  ///////////////////////////////////////////////////
  // LOOP START
  ///////////////////////////////////////////////////
  while(YRBOdomPub.n.ok()){
    // get current time
    YRBOdomPub.current_time = ros::Time::now();
    // Get odometory pose information
    Spur_get_pos_GL(&YRBOdomPub.x, &YRBOdomPub.y, &YRBOdomPub.th);
    // get current velocity
    Spur_get_vel(&YRBOdomPub.vx, &YRBOdomPub.vth);
    // get current angular
    YP_get_wheel_ang(&YRBOdomPub.l_ang, &YRBOdomPub.r_ang);
    
    // TF transform section -------------------------------------------------------
    //first, we'll publish the transform over tf
    odom_trans.header.stamp = YRBOdomPub.current_time;
    odom_trans.transform.translation.x = YRBOdomPub.x;
    odom_trans.transform.translation.y = YRBOdomPub.y;
    odom_trans.transform.translation.z = 0.0;
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf::createQuaternionMsgFromYaw(YRBOdomPub.th);
    odom_trans.transform.rotation = odom_quat;
    //send the transform
    YRBOdomPub.odom_broadcaster.sendTransform(odom_trans);
    // TF transform section end ---------------------------------------------------

    // Odom section ---------------------------------------------------------------
    //next, we'll publish the odometry message over ROS
    // set time stamp
    odom.header.stamp = YRBOdomPub.current_time;
    // set the position
    odom.pose.pose.position.x = YRBOdomPub.x;
    odom.pose.pose.position.y = YRBOdomPub.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    // set the velocity
    odom.twist.twist.linear.x = YRBOdomPub.vx;
    odom.twist.twist.linear.y = YRBOdomPub.vy;
    odom.twist.twist.angular.z = YRBOdomPub.vth;
    // publish the message
    YRBOdomPub.odom_pub.publish(odom);
    // Odom section end -----------------------------------------------------------

    // Joint State section --------------------------------------------------------
    // at the end, we'll get the joint values 
    js.header.stamp = YRBOdomPub.current_time;
    js.position[0] = YRBOdomPub.l_ang;
    js.position[1] = YRBOdomPub.r_ang;
    YRBOdomPub.js_pub.publish(js);
    // Joint State section end ----------------------------------------------------
    
    r.sleep();
  }
  /////////////////////////////////////////
  // END LOOP
  ////////////////////////////////////////
  
  return 0;
}

