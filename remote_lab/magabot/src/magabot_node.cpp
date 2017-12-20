#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "magabot/Status.h"
#include "magabot/Control.h"

#define _USE_MATH_DEFINES

#ifndef NORMALIZE
    #define NORMALIZE(z) atan2(sin(z), cos(z))	// Normalize angle to domain -pi, pi 
#endif

const char* NODE_NAME = "magabot_node";

const char* ODOM_PUB_NAME = "/odom";
const unsigned int ODOM_PUB_QUEUE_SIZE = 1;
const unsigned int ODOM_PUB_RATE = 80;

const char* CMD_VEL_SUB_NAME = "cmd_vel";
const unsigned int CMD_VEL_SUB_QUEUE_SIZE = 1;

const char* MAGABOT_CONTROL_PUB_NAME = "/magabot/control";
const unsigned int MAGABOT_CONTROL_PUB_QUEUE_SIZE = 1;

const char* MAGABOT_STATUS_SUB_NAME = "/magabot/status";
const unsigned int MAGABOT_STATUS_SUB_QUEUE_SIZE = 1;

const char* ODOM_FRAME_ID = "odom";
const char* ODOM_CHILD_FRAME_ID = "base_footprint";

const float AXLE_LENGTH = 0.345;
const float WHEEL_RADIUS = 0.0465;
const float TICKS_REVOLUTION = 3900;
// const float TICKS_REVOLUTION = 7800;

const float K_TH = 2 * M_PI * WHEEL_RADIUS / (AXLE_LENGTH * TICKS_REVOLUTION);
const float K_XY = WHEEL_RADIUS * M_PI / TICKS_REVOLUTION;

const float DIST_PER_TICK = (2 * M_PI * WHEEL_RADIUS) / TICKS_REVOLUTION;

long int left_encoder_prev = 0, prev_right_encoder = 0;
double odom_x, odom_y, odom_theta = 0.0;

ros::Time current_time, prev_time;

ros::Publisher *odom_pub_ptr;
tf::TransformBroadcaster *odom_broadcaster_ptr;
ros::Publisher *magabot_control_pub_ptr;

void magabotDataCallback(const magabot::Status::ConstPtr& status_msg)
{

	double v_x;
	double v_y;
	double w_z;

	double x;
	double y;
	double th;

	double d_right, d_left, d_center;

	double theta;

	int right_encoder = status_msg->encoders.right_wheel;
	int left_encoder = status_msg->encoders.left_wheel;
	
	d_right = (right_encoder * DIST_PER_TICK - prev_right_encoder * DIST_PER_TICK);
	d_left = (left_encoder * DIST_PER_TICK - left_encoder_prev * DIST_PER_TICK);

	d_center = (d_right + d_left) / 2.0;
	theta = (d_right - d_left) / AXLE_LENGTH;

	x = d_center * cos(odom_theta);
	y = d_center * sin(odom_theta);

	odom_x += x;
	odom_y += y;
	odom_theta = NORMALIZE(odom_theta + theta);

	prev_time = current_time;
	current_time = status_msg->timestamp;

	double dt = (current_time - prev_time).toSec();

	v_x = double(x / dt);
	v_y = double(y / dt);
	w_z = double(theta / dt);

	// Since all odometry is 6DOF we'll need a quaternion created from yaw    
	geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw(odom_theta);

	// First, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_TF;
	odom_TF.header.stamp = current_time;
	odom_TF.header.frame_id = ODOM_FRAME_ID;
// 	odom_TF.header.frame_id = ODOM_CHILD_FRAME_ID;
	odom_TF.child_frame_id = ODOM_CHILD_FRAME_ID;
// 	odom_TF.child_frame_id = ODOM_FRAME_ID;
	    
	odom_TF.transform.translation.x = odom_x;
	odom_TF.transform.translation.y = odom_y;
	odom_TF.transform.translation.z = 0.0;
	odom_TF.transform.rotation = odom_quaternion;
	    
	// Send the transform
	odom_broadcaster_ptr->sendTransform(odom_TF); 		// odom->base_link transform

	// Next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = ODOM_FRAME_ID;
// 	odom.header.frame_id = ODOM_CHILD_FRAME_ID;
	    
	// Set the position
	odom.pose.pose.position.x = odom_x;
	odom.pose.pose.position.y = odom_y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quaternion;
	    
	// Set the velocity
	odom.child_frame_id = ODOM_CHILD_FRAME_ID;
// 	odom.child_frame_id = ODOM_FRAME_ID;
	odom.twist.twist.linear.x = v_x;
	odom.twist.twist.linear.y = v_y;
	odom.twist.twist.angular.z = w_z;

	odom.pose.covariance[0] = 0.001;
	odom.pose.covariance[7] = 0.001;
	odom.pose.covariance[14] = 0.001;
	odom.pose.covariance[21] = 1000;
	odom.pose.covariance[28] = 1000;
	odom.pose.covariance[35] = 1000;

	odom.twist.covariance = odom.pose.covariance;
	    
	// Publish the message
	odom_pub_ptr->publish(odom);		// odometry publisher 
}

void cmd_vel_received(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	magabot::Control magabot_control_msg;

	double v_x = cmd_vel->linear.x;
	double w_z = cmd_vel->angular.z;

	int right_wheel_velocity = (((v_x + w_z * AXLE_LENGTH) / 2) / DIST_PER_TICK) * 0.00332;
	int left_wheel_velocity = (((v_x - w_z * AXLE_LENGTH) / 2) / DIST_PER_TICK) * 0.00332;

	magabot_control_msg.set_velocity = true;

	magabot_control_msg.velocity.right_wheel = right_wheel_velocity;
	magabot_control_msg.velocity.left_wheel = left_wheel_velocity;

	magabot_control_pub_ptr->publish(magabot_control_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);

	ros::NodeHandle nh;

	// ROS publishers and subscribers
 	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(ODOM_PUB_NAME, ODOM_PUB_QUEUE_SIZE);
 	tf::TransformBroadcaster odom_broadcaster;
 	ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(CMD_VEL_SUB_NAME, CMD_VEL_SUB_QUEUE_SIZE, cmd_vel_received);
 	ros::Subscriber magabot_status_sub  = nh.subscribe<magabot::Status>(MAGABOT_STATUS_SUB_NAME, MAGABOT_STATUS_SUB_QUEUE_SIZE, magabotDataCallback);
	ros::Publisher magabot_control_pub = nh.advertise<magabot::Control>(MAGABOT_CONTROL_PUB_NAME, MAGABOT_CONTROL_PUB_QUEUE_SIZE);

	odom_pub_ptr = &odom_pub;
	odom_broadcaster_ptr = &odom_broadcaster;
	magabot_control_pub_ptr = &magabot_control_pub;

	ros::spin(); //trigger callbacks and prevents exiting

	return 0;
}
