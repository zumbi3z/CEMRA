#include <stdlib.h>
#include <stdio.h>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>
#include "magabot/Status.h"

const char* NODE_NAME = "Sensors";
const unsigned int PUB_QUEUE_SIZE = 1000;
const char* PUB_NAME[] = {
	"/magabot/sensors/left_sonar",
	"/magabot/sensors/center_left_sonar",
	"/magabot/sensors/center_sonar",
	"/magabot/sensors/center_right_sonar",
	"/magabot/sensors/right_sonar"
};

const char* SUB_NAME = "/magabot/Status";
const unsigned int SUB_QUEUE_SIZE = 1000;

const unsigned int N_SONARS = sizeof(magabot::Status::sonars) / sizeof(magabot::Status::sonars[0]);

const std::string SONARS_FRAME_IDS[N_SONARS] = {
	"left_sonar",
	"center_left_sonar",
	"center_sonar",
	"center_right_sonar",
	"right_sonar"
};

ros::Publisher pub_sonars[N_SONARS];

// ros::Publisher pub_sonar_center;
// ros::Publisher pub_sonar_centerLeft;
// ros::Publisher pub_sonar_centerRight;
// ros::Publisher pub_sonar_left;
// ros::Publisher pub_sonar_right;

ros::Subscriber sub_magabot_status;

void magabotDataCallback(const magabot::Status::ConstPtr& magabot_status_msg)
{
	sensor_msgs::Range msg_sonars[N_SONARS];

	// sensor_msgs::Range sonar_center;
	// sensor_msgs::Range sonar_centerLeft;
	// sensor_msgs::Range sonar_centerRight;
	// sensor_msgs::Range sonar_left;
	// sensor_msgs::Range sonar_right;

	for(int i = 0; i < N_SONARS; i++)
		msg_sonars[i].header.stamp = magabot_status_msg->timestamp;

	// sonar_center.stamp = magabot_status_msg->timestamp;
	// sonar_centerLeft.stamp = magabot_status_msg->timestamp;
	// sonar_centerRight.stamp = magabot_status_msg->timestamp;
	// sonar_left.stamp = magabot_status_msg->timestamp;
	// sonar_right.stamp = magabot_status_msg->timestamp;

	for (int i = 0; i < N_SONARS; ++i)
		msg_sonars[i].header.frame_id = SONARS_FRAME_IDS[i];

	// sonar_center.frame_id = "centerSonar";
	// sonar_left.frame_id = "leftSonar";
	// sonar_right.frame_id = "rightSonar";
	// sonar_centerLeft.frame_id = "centerLeftSonar";
	// sonar_centerRight.frame_id = "centerRightSonar";

	for(int i = 0; i < N_SONARS; i++)
		msg_sonars[i].radiation_type = sensor_msgs::Range::ULTRASOUND;

	// sonar_center.radiaton_type = sensor_msgs::Range::ULTRASOUND;
	// sonar_centerLeft.radiaton_type = sensor_msgs::Range::ULTRASOUND;
	// sonar_centerRight.radiaton_type = sensor_msgs::Range::ULTRASOUND;
	// sonar_left.radiaton_type = sensor_msgs::Range::ULTRASOUND;
	// sonar_right.radiaton_type = sensor_msgs::Range::ULTRASOUND;

	for(int i = 0; i < N_SONARS; i++)
		msg_sonars[i].field_of_view = 0.6;

	// sonar_center.field_of_view = 0.6;
	// sonar_centerLeft.field_of_view = 0.6;
	// sonar_centerRight.field_of_view = 0.6;
	// sonar_left.field_of_view = 0.6;
	// sonar_right.field_of_view = 0.6;

	for(int i = 0; i < N_SONARS; i++)
		msg_sonars[i].min_range = 0.17;

	// sonar_center.min_range = 0.17;
	// sonar_centerLeft.min_range = 0.17;
	// sonar_centerRight.min_range = 0.17;
	// sonar_left.min_range = 0.17;
	// sonar_right.min_range = 0.17;

	for (int i = 0; i < N_SONARS; i++)
		msg_sonars[i].max_range = 6.0;

	// sonar_center.max_range = 6.0;
	// sonar_centerLeft.max_range = 6.0;
	// sonar_centerRight.max_range = 6.0;
	// sonar_left.max_range = 6.0;
	// sonar_right.max_range = 6.0;

	for (int i = 0; i < N_SONARS; ++i)
		msg_sonars[i].range = 0.01 * magabot_status_msg->sonars[i];

	// sonar_left.range = 0.01 * magabot_status_msg->sonars[0];
	// sonar_centerLeft.range = 0.01 * magabot_status_msg->sonars[1];
	// sonar_center.range = 0.01 * magabot_status_msg->sonars[2];
	// sonar_centerRight.range = 0.01 * magabot_status_msg->sonars[3];
	// sonar_right.range = 0.01 * magabot_status_msg->sonars[4];

	for (int i = 0; i < N_SONARS; ++i)
	{
		pub_sonars[i].publish(msg_sonars[i]);
	}

	// pub_sonar_center.publish(sonar_left);
	// pub_sonar_centerLeft.publish(sonar_centerLeft);
	// pub_sonar_centerRight.publish(sonar_centerRight);
	// pub_sonar_left.publish(sonar_left);
	// pub_sonar_right.publish(sonar_right);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);

	ros::NodeHandle nh;

	// pub_sonar_center = nh.advertise<sensor_msgs::Range>(PUB_NAME, PUB_QUEUE_SIZE);
	// pub_sonar_centerLeft = nh.advertise<sensor_msgs::Range>(PUB_NAME, PUB_QUEUE_SIZE);
	// pub_sonar_centerRight = nh.advertise<sensor_msgs::Range>(PUB_NAME, PUB_QUEUE_SIZE);
	// pub_sonar_right = nh.advertise<sensor_msgs::Range>(PUB_NAME, PUB_QUEUE_SIZE);
	// pub_sonar_left = nh.advertise<sensor_msgs::Range>(PUB_NAME, PUB_QUEUE_SIZE);

	for (int i = 0; i < N_SONARS; ++i)
		pub_sonars[i] = nh.advertise<sensor_msgs::Range>(PUB_NAME[i], PUB_QUEUE_SIZE);  // PODEM TODOS TER O MESAMO NOME ?! =0

	sub_magabot_status = nh.subscribe(SUB_NAME, SUB_QUEUE_SIZE, magabotDataCallback);

	ros::spin();

	return 0;

}