//C++ Libraries
#include <stdio.h>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>

//ROS libraries
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//user libraries
//#include "cam/detections.h"
#include "cam/blob_detection.h"
#include "cam/marker_detection.h"
//#include "cam/debug.h" //Comment or uncomment this for verbose
#include "yaml-cpp/yaml.h"

#include "cam/QuadPose.h"
#include "cam/QuadPoseList.h"

using namespace std;
using namespace YAML;
using namespace Eigen;

#define MAX_DEVIATION 0.2
#define RATE 50
#define LASER_COMPARISON 1
#define PI 3.14159265

vector<marker>* detected_quads; //Remember, marker struct = one quad
MatrixXd* objects;
unsigned char* bufb,* buf;
blob* blp;
int nblobs,nobjects;

int toggle_first_pose = 0;
cam::QuadPose first_pose;
double value = 0.0;
double last_value = 0.0;
int width;
int height;

//static ros::Publisher rgb_image_pub;
static ros::Publisher bin_image_pub;
static ros::Publisher cam_eval;

geometry_msgs::PoseStamped msg;

void image_reception_callback(const sensor_msgs::ImageConstPtr& msg){

	//get image
	const sensor_msgs::Image img = *msg;

	//Copy image to the buffer
	unsigned long int cnt=0;
    for(unsigned int l=0;l<img.height;l++){
        for(unsigned int k=0;k<img.width;k++){
            buf[cnt]=img.data[cnt + 0 ];
            buf[cnt+1]=img.data[cnt + 1];
            buf[cnt+2]=img.data[cnt + 2];
            cnt+=3;
            
        }
    }
    /*
    //publishing usefull information
    sensor_msgs::Image rgb_img = *msg;
    fillImage(rgb_img,"rgb8",img.height,img.width,img.step,buf);
    rgb_image_pub.publish(rgb_img);
	*/
    for(unsigned int l=0;l<img.height;l++)
        for(unsigned int k=0;k<img.width;k++)
        	if(bufb[l*img.width + k] > 0) 
                bufb[l*img.width + k] = 255;
    sensor_msgs::Image bin_img = *msg;
    bufb = get_binary_image();
    detected_quads = get_markers();
    fillImage(bin_img,"mono8",img.height,img.width,img.step/3,bufb);
    bin_image_pub.publish(bin_img);

	//detect robots
    track_markers(buf, 3, img.width, img.height);	
	detected_quads = get_markers();

	//for each detected robot
	for(vector<marker>::iterator it = detected_quads->begin(); it != detected_quads->end(); ++it){

		//check if there were any errors in the detection of the robot
		if(it->failures == 0){

			//fill positioning structure that is going to be sent to ROS
			geometry_msgs::PoseWithCovarianceStamped robot_pose;
			robot_pose.header.stamp = ros::Time::now();
			robot_pose.header.frame_id = it->name;
			robot_pose.pose.pose.position.x = it->T.col(3)[0];
			robot_pose.pose.pose.position.y = it->T.col(3)[1];
			robot_pose.pose.pose.position.z = it->T.col(3)[2];
			Matrix3d rotation;
			rotation.col(0) = it->T.col(0);
			rotation.col(1) = it->T.col(1);
			rotation.col(2) = it->T.col(2);
			Vector3d ea = rotation.eulerAngles(2, 1, 0); //the order of the angles matter
			Quaternion<double> q(rotation);
			double roll = ea[2];
			double pitch = ea[1];
			double yaw = ea[0];
			robot_pose.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);

			//send positioning structure to ROS
			cam_eval.publish(robot_pose);

		}

	}
	
    return;
}

 
void readImageParams(std::string param_name){

	//get ros node handle
	ros::NodeHandle nh; // is the same handle as in the main, seems ros is global
	std::string full_name;

	full_name = param_name + "width";
	nh.getParam(full_name, width);
	full_name = param_name + "height";
	nh.getParam(full_name, height);

	//cout <<"WORKS: " << width << " " << height << endl;

}

int main(int argc, char** argv){

	//Initialize ROS
	ros::init(argc, argv,"node/");
	ros::NodeHandle nh;

	//initialize node parameters
	readImageParams("/node/");

	//Initialize the sensor
	init_binary_img(width, height);
	buf = (unsigned char*)malloc(width*height*3*sizeof(unsigned char));
    bufb = get_binary_image();
    blp = get_blobs();
    initialize_markers("/node/");
    
    //subscribers
    ros::Subscriber image_sub=nh.subscribe("camera1/image_raw", 1, image_reception_callback); // only 1 in buffer size to drop other images if processing is not finished

    //publishers
    //rgb_image_pub=nh.advertise<sensor_msgs::Image>("node/rgb_image",1);
    bin_image_pub=nh.advertise<sensor_msgs::Image>("node/binary_image",1);
    cam_eval = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot2/cam_pose",1);
    ros::Rate loop_rate(RATE);
   
    //main loop
    while(ros::ok()){
        loop_rate.sleep();
		ros::spinOnce();
    }
    return EXIT_SUCCESS;
}
