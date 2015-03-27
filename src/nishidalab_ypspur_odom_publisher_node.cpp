#include <ros/ros.h>                    // for ros
#include <tf/transform_broadcaster.h>	// for tf
#include <nav_msgs/Odometry.h>	        //for odometry
#include <ypspur.h>		        // for yp-spur
#include <cmath>	 	        // for math PI

int main(int argc, char** argv){

  // init ros
  ros::init(argc, argv, "nishidalab_ypspur_odom_publisher");

  // Yp-spur init
  if( Spur_init() < 0)
    ROS_ERROR("can't open spur");

  // set node handler
  ros::NodeHandle n;

  // set odometory publisher
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50); 

  // set tf broad caster
  tf::TransformBroadcaster odom_broadcaster;

  // odometry pose infomation
  double x, y, th;

  // velocity twist information
  double vx, vy, vth;

  // ros timer
  ros::Time current_time = ros::Time::now();
  
  // for quaternion
  geometry_msgs::Quaternion odom_quat;
  // for transform tf
  geometry_msgs::TransformStamped odom_trans;
  // for pusblish odometry
  nav_msgs::Odometry odom;

  // loop rate is 25.0 [Hz]
  ros::Rate r(25.0);


  ///////////////////////////////////////////////////
  // LOOP START
  ///////////////////////////////////////////////////
  while(n.ok()){
    // get current time
    current_time = ros::Time::now();
    
    // Get odometory pose information
    Spur_get_pos_GL(&x, &y, &th);

    // get current velocity
    Spur_get_vel(&vx, &vth);
    vy = 0;
    // TF transform section -----------------------------------------------------------
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    // TF transform section end -------------------------------------------------------
    

    // Odom section -------------------------------------------------------------------
    //next, we'll publish the odometry message over ROS
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
    // Odom section end ----------------------------------------------------------------

    r.sleep();
  }
  /////////////////////////////////////////
  // END LOOP
  ////////////////////////////////////////
}

