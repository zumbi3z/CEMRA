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
#define VERBOSE

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

static ros::Publisher rgb_image_pub;
static ros::Publisher bin_image_pub;
static ros::Publisher cam_eval[10];

geometry_msgs::PoseStamped msg;

//color for the bounding boxes
double colormap[] = {0.2422, 0.1504, 0.6603,
                     0.0067, 0.7312, 0.7660,
                     0.3909, 0.8029, 0.4354,
                     0.9035, 0.7330, 0.1774,
                     0.9597, 0.9135, 0.1423};

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

	//detect robots
    track_markers(buf, 3, img.width, img.height);	
	detected_quads = get_markers();

	//for each detected robot
	#ifdef VERBOSE
	cout << "Detected robots:" << endl;
	#endif
	for(vector<marker>::iterator it = detected_quads->begin(); it != detected_quads->end(); ++it){

		//check if there were any errors in the detection of the robot
		if(it->failures == 0){

			//fill positioning structure that is going to be sent to ROS
			#ifdef VERBOSE
			cout << it->name << endl;
			#endif
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
			if(!strcmp(robot_pose.header.frame_id.c_str(), "robot1")) cam_eval[0].publish(robot_pose);
			else if(!strcmp(robot_pose.header.frame_id.c_str(), "robot2")) cam_eval[1].publish(robot_pose);
			else if(!strcmp(robot_pose.header.frame_id.c_str(), "robot3")) cam_eval[2].publish(robot_pose);
			else if(!strcmp(robot_pose.header.frame_id.c_str(), "robot4")) cam_eval[3].publish(robot_pose);
			else if(!strcmp(robot_pose.header.frame_id.c_str(), "unknown")) cam_eval[4].publish(robot_pose);

			/*if( (robot_pose.pose.pose.position.x < 2.50) || (robot_pose.pose.pose.position.y < 1.1) )
				cam_eval[1].publish(robot_pose);*/

		}

	}

	//publishing binary image
	for(unsigned int l=0;l<img.height;l++)
        for(unsigned int k=0;k<img.width;k++)
        	if(bufb[l*img.width + k] > 0)
                bufb[l*img.width + k] = 255;
    for(vector<marker>::iterator it = detected_quads->begin(); it != detected_quads->end(); ++it){
		for(int k=it->imx0,l=it->imy0;k<it->imxf;k++) bufb[l*img.width  + k]=255;
		for(int k=it->imx0,l=it->imyf;k<it->imxf;k++) bufb[l*img.width + k]=255;
		for(int k=it->imx0,l=it->imy0;l<it->imyf;l++) bufb[l*img.width  + k]=255;
		for(int k=it->imxf,l=it->imy0;l<it->imyf;l++) bufb[l*img.width + k]=255;

		//if(strcmp(it->name,"unknown")){
		    for(int l=it->imy0_LED;l<it->imyf_LED;l++)
			for(int k=it->imx0_LED;k<it->imxf_LED;k++)
			    if(bufb[l*img.width+ k]!=0)
				bufb[l*img.width + k]=255;

		    for(int k=it->imx0_LED,l=it->imy0_LED;k<it->imxf_LED;k++) bufb[l*img.width + k]=255;
		    for(int k=it->imx0_LED,l=it->imyf_LED;k<it->imxf_LED;k++) bufb[l*img.width + k]=255;
		    for(int k=it->imx0_LED,l=it->imy0_LED;l<it->imyf_LED;l++) bufb[l*img.width + k]=255;
		    for(int k=it->imxf_LED,l=it->imy0_LED;l<it->imyf_LED;l++) bufb[l*img.width + k]=255;
		    
		//}
	}
    sensor_msgs::Image bin_img = *msg;
    bufb = get_binary_image();
    detected_quads = get_markers();
    fillImage(bin_img,"mono8",img.height,img.width,img.step/3,bufb);
    bin_image_pub.publish(bin_img);


    //publishing rgb image
    for(vector<marker>::iterator it = detected_quads->begin(); it != detected_quads->end(); ++it){

    	//attribute color according to robot name
    	double r, g, b;
    	std::string robot_name = it->name;
    	robot_name[strlen("robot")] = '\0'; //detect robot in the name
    	if(strcmp(robot_name.c_str(), "robot") == 0){
    		int n = atoi( &it->name[strlen("robot")] ) - 1;
    		cout << "ROBOT number: " << n << endl;
    		r = colormap[3*n + 0];
    		g = colormap[3*n + 1];
    		b = colormap[3*n + 2];
    	}
    	else {r = g = b = 0.7;}

    	//write bounding boxes around detected robots
		for(int k=it->imx0,l=it->imy0;k<it->imxf;k++)
			{buf[3*(l*img.width + k) + 0] = 255*b; buf[3*(l*img.width + k) + 1] = 255*g; buf[3*(l*img.width + k) + 2] = 255*r;}
		for(int k=it->imx0,l=it->imyf;k<it->imxf;k++)
			{buf[3*(l*img.width + k) + 0] = 255*b; buf[3*(l*img.width + k) + 1] = 255*g; buf[3*(l*img.width + k) + 2] = 255*r;}
		for(int k=it->imx0,l=it->imy0;l<it->imyf;l++)
			{buf[3*(l*img.width + k) + 0] = 255*b; buf[3*(l*img.width + k) + 1] = 255*g; buf[3*(l*img.width + k) + 2] = 255*r;}
		for(int k=it->imxf,l=it->imy0;l<it->imyf;l++)
			{buf[3*(l*img.width + k) + 0] = 255*b; buf[3*(l*img.width + k) + 1] = 255*g; buf[3*(l*img.width + k) + 2] = 255*r;}
	}
    sensor_msgs::Image rgb_img = *msg;
    fillImage(rgb_img,"bgr8",img.height,img.width,img.step,buf);
    rgb_image_pub.publish(rgb_img);

	
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
    rgb_image_pub=nh.advertise<sensor_msgs::Image>("node/rgb_image",1);
    bin_image_pub=nh.advertise<sensor_msgs::Image>("node/binary_image",1);
    cam_eval[0] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot1/cam_pose",1);
    cam_eval[1] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot2/cam_pose",1);
    cam_eval[2] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot3/cam_pose",1);
    cam_eval[3] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot4/cam_pose",1);
    cam_eval[4] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/unknown/cam_pose",1);
    ros::Rate loop_rate(RATE);
   
    //main loop
    while(ros::ok()){
        loop_rate.sleep();
		ros::spinOnce();
    }
    return EXIT_SUCCESS;
}
