//C++ Libraries
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>

//ROS libraries
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;

//local defines
#define NROBOTS_MAX 10
#define RATE 100

int main(int argc, char** argv){

	//Initialize ROS
	ros::init(argc, argv,"bag_merge");
	ros::NodeHandle nh;

	//get number of robots in simulations
	int nrobots; 
	nh.getParam("/bag_merge/nrobots", nrobots);
	cout << "Number of robots in simulation: " << nrobots << endl;
	string bag_folder_name;
	nh.getParam("/bag_merge/bag_folder_name", bag_folder_name);
	cout << "bag folder name: " << bag_folder_name << endl;
	string bag_name;
	nh.getParam("/bag_merge/bag_name", bag_name);
	cout << "bag name: " << bag_name << endl;
	int counter_max;
	nh.getParam("/bag_merge/counter_max", counter_max);
	cout << "counter max: " << counter_max << endl;

	//initialize publishers
	ros::Publisher cam_pose_gt_pub[NROBOTS_MAX];
	ros::Publisher estimated_pose_gt_pub[NROBOTS_MAX];
	for(int k = 0; k < nrobots; k++){

		char pub_topic_name_chrp[100];
		sprintf(pub_topic_name_chrp, "robot%d/cam_pose_GT", k + 1);
		cam_pose_gt_pub[k] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pub_topic_name_chrp, 1);
		sprintf(pub_topic_name_chrp, "robot%d/estimated_pose_GT", k + 1);
		estimated_pose_gt_pub[k] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pub_topic_name_chrp, 1);

	}

	//open files with ground truth data
	ifstream cam_pose_gt_file[NROBOTS_MAX];
	ifstream estimated_pose_gt_file[NROBOTS_MAX];
	for(int k = 0; k < nrobots; k++){

		char file_name_chrp[400];
		sprintf(file_name_chrp, "%s/%srobot%d_cam.txt", bag_folder_name.c_str(), bag_name.c_str(), k + 1);
		cam_pose_gt_file[k].open(file_name_chrp);
		if(!cam_pose_gt_file[k].is_open())
			cout << "File '" << file_name_chrp << "' NOT opened." << endl;
		sprintf(file_name_chrp, "%s/%srobot%d_arena.txt", bag_folder_name.c_str(), bag_name.c_str(), k + 1);
		estimated_pose_gt_file[k].open(file_name_chrp);
		if(!estimated_pose_gt_file[k].is_open())
			cout << "File '" << file_name_chrp << "' NOT opened." << endl;

	}

	//initialize loop rate
	ros::Rate loop_rate(RATE);

	//main loop
	int count = 0;
	while(ros::ok()){

		//sleep acording to loop rate
		loop_rate.sleep();

		//go through the files with the ground truth data
		for(int k = 0; k < nrobots; k++){

			//read a line of the files and publish (cam_pose_gt_file)
			string line;
			geometry_msgs::PoseWithCovarianceStamped msg;
			if( cam_pose_gt_file[k].is_open() )
				for(int l = 0; l < counter_max; l++)
					if( getline(cam_pose_gt_file[k], line) ){

						//read file
						sscanf((const char*)line.c_str(), "%lf %lf %lf %lf %lf %lf %lf",&msg.pose.pose.position.x,\
						                                                                &msg.pose.pose.position.y,\
						                                                                &msg.pose.pose.position.z,\
						                                                                &msg.pose.pose.orientation.w,\
						                                                                &msg.pose.pose.orientation.x,\
						                                                                &msg.pose.pose.orientation.y,\
						                                                                &msg.pose.pose.orientation.z);

						//include stamp and publish
						if(l == (counter_max - 1)){
							/*cout << "cam_line_" << count << ": " << msg.pose.pose.position.x << " "\
							                                     << msg.pose.pose.position.y << " "\
							                                     << msg.pose.pose.position.z << " "\
							                                     << msg.pose.pose.orientation.w << " "\
							                                     << msg.pose.pose.orientation.x << " "\
							                                     << msg.pose.pose.orientation.y << " "\
							                                     << msg.pose.pose.orientation.z << " " << endl;*/
							msg.header.stamp = ros::Time::now();
							cam_pose_gt_pub[k].publish(msg);
						}

					}

			//read a line of the files and publish (estimated_pose_gt_file)
			if( estimated_pose_gt_file[k].is_open() )
				for(int l = 0; l < counter_max; l++)
					if( getline(estimated_pose_gt_file[k], line) ){

						//read file
						sscanf((const char*)line.c_str(), "%lf %lf %lf %lf %lf %lf %lf",&msg.pose.pose.position.x,\
						                                                                &msg.pose.pose.position.y,\
						                                                                &msg.pose.pose.position.z,\
						                                                                &msg.pose.pose.orientation.w,\
						                                                                &msg.pose.pose.orientation.x,\
						                                                                &msg.pose.pose.orientation.y,\
						                                                                &msg.pose.pose.orientation.z);

						//include stamp and publish
						if(l == (counter_max - 1)){
							/*cout << "estimated_line_" << count << ": " << msg.pose.pose.position.x << " "\
							                                           << msg.pose.pose.position.y << " "\
							                                           << msg.pose.pose.position.z << " "\
							                                           << msg.pose.pose.orientation.w << " "\
							                                           << msg.pose.pose.orientation.x << " "\
							                                           << msg.pose.pose.orientation.y << " "\
							                                           << msg.pose.pose.orientation.z << " " << endl;*/
							msg.header.stamp = ros::Time::now();
							estimated_pose_gt_pub[k].publish(msg);
						}

					}

		}

		//read (prediction and update) messages
		ros::spinOnce();
		count++;

	}

    return 0;
}
