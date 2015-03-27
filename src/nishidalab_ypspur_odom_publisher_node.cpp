#include <ros/ros.h>                    // for ros
#include <tf/transform_broadcaster.h>	// for tf
#include <nav_msgs/Odometry.h>	        //for odometry
#include <ypspur.h>		        // for yp-spur
#include <cmath>	 	        // for math PI
#include "nishidalab_ypspur_driver/nishidalab_ypspur_odom_publisher_node.hpp"  // include local header

int main(int argc, char** argv){

  // init ros
  ros::init(argc, argv, "nishidalab_ypspur_odom_publisher");
  
  // original class
  NishidalabYpspurOdomPublisher NlabYspurOdomPub;
  // for quaternion
  geometry_msgs::Quaternion odom_quat;
  // for transform tf
  geometry_msgs::TransformStamped odom_trans;
  // for pusblish odometry
  nav_msgs::Odometry odom;

  // loop rate is 25.0 [Hz]
  ros::Rate r(25.0);

  double x, y, th;
  double vx, vth;

  ///////////////////////////////////////////////////
  // LOOP START
  ///////////////////////////////////////////////////
  while(NlabYspurOdomPub.n.ok()){
    // get current time
    NlabYspurOdomPub.current_time = ros::Time::now();
    
    // Get odometory pose information
    Spur_get_pos_GL(&x, &y, &th);//&NlabYspurOdomPub.x, &NlabYspurOdomPub.y, &NlabYspurOdomPub.th );
    NlabYspurOdomPub.x = x;
    NlabYspurOdomPub.y = y;
    NlabYspurOdomPub.th = th;

    // get current velocity
    Spur_get_vel(&vx, &vth);//&NlabYspurOdomPub.vx, &NlabYspurOdomPub.vth );
    NlabYspurOdomPub.vx = vx;
    NlabYspurOdomPub.vth = vth;
    NlabYspurOdomPub.vy = 0;

    // TF transform section -----------------------------------------------------------
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf::createQuaternionMsgFromYaw(NlabYspurOdomPub.th);

    //first, we'll publish the transform over tf
    odom_trans.header.stamp = NlabYspurOdomPub.current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = NlabYspurOdomPub.x;
    odom_trans.transform.translation.y = NlabYspurOdomPub.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    NlabYspurOdomPub.odom_broadcaster.sendTransform(odom_trans);
    // TF transform section end -------------------------------------------------------
    

    // Odom section -------------------------------------------------------------------
    //next, we'll publish the odometry message over ROS
    odom.header.stamp = NlabYspurOdomPub.current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = NlabYspurOdomPub.x;
    odom.pose.pose.position.y = NlabYspurOdomPub.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = NlabYspurOdomPub.vx;
    odom.twist.twist.linear.y = NlabYspurOdomPub.vy;
    odom.twist.twist.angular.z = NlabYspurOdomPub.vth;

    //publish the message
    NlabYspurOdomPub.odom_pub.publish(odom);
    // Odom section end ----------------------------------------------------------------

    r.sleep();
  }
  /////////////////////////////////////////
  // END LOOP
  ////////////////////////////////////////
}

