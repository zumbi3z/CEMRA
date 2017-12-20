#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "magabot/Control.h"

#define _USE_MATH_DEFINES

const char* NODE_NAME = "Controller";

const char* PUB_NAME = "/magabot/Control";
const unsigned int PUB_QUEUE_SIZE = 1000;

const char* SUB_NAME = "cmd_vel";
const unsigned int SUB_QUEUE_SIZE = 1000;

const float AXLE_LENGTH = 0.345;
const float WHEEL_RADIUS = 0.0465;
// const float TICKS_PER_REV = 3900;
const float TICKS_REVOLUTION = 7800;
const float DIST_PER_TICK = (2 * M_PI * WHEEL_RADIUS) / TICKS_REVOLUTION;

ros::Publisher * magabot_control_pub_ptr;

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

	ros::Publisher magabot_control_pub = nh.advertise<magabot::Control>(PUB_NAME, PUB_QUEUE_SIZE);
	ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(SUB_NAME, SUB_QUEUE_SIZE, cmd_vel_received);

	magabot_control_pub_ptr = &magabot_control_pub;

	ros::spin(); //trigger callbacks and prevents exiting

	return 0;
}
