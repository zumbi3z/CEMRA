//C++ Libraries
#include <stdio.h>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>

//ROS libraries
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

//user libraries
#include "estimator.h"
#include "model.h"

using namespace std;
using namespace Eigen;

//local defines
#define RATE 20

//variables
typedef struct t_nparams{
	void* actuation_params;
	void* measurement_params;
} nparams;

nparams node_params;
estimator* est;

//functions

size_t get_executable_path (char* buffer, size_t len)
{
  char* path_end;
  if (readlink ("/proc/self/exe", buffer, len) <= 0)
    return -1;
  path_end = strrchr (buffer, '/');
  if (path_end == NULL)
    return -1;
  ++path_end;
  *path_end = '\0';
  return (size_t) (path_end - buffer);
}


//initializing parameters for this node
int readConfigurationParams(std::string param_name){

	//get ros node handle
	ros::NodeHandle nh; //is the same handle as in the main, seems ros is global
	std::string full_name;

	//get path for file names
	char file_name_path[300];
	get_executable_path(file_name_path, 300);
	string file_name(file_name_path);

	//initialize actuation and measurement models
	if( (node_params.actuation_params = modelConfigure(file_name + "../params/actuation_params.txt")) == (void*)NULL)
		{cout << "Problem initializing actuation model." << endl; return 0;}

	if( (node_params.measurement_params = modelConfigure(file_name + "../params/measurement_params.txt")) == (void*)NULL){
		{cout << "Problem initializing measurement model." << endl; return 0;}
	}

	//initialize estimator
	est = estimatorInit("robot", "differential_drive", NULL);
	VectorXd x(3,1); x << 0, 0, 0;
	estimatorReset(est, x, ros::Time::now().toSec());

	//return success
	return 1;

}

//prediction callback
void prediction_callback(const geometry_msgs::TwistPtr& msg){

	//extract actuation from message
	VectorXd u(2, 1);
	u(0) = msg->linear.x;
	u(1) = msg->angular.z;

	//add actuation noise
	MatrixXd Q(2, 2);
	if(modelSetActuationNoise("differential_drive", node_params.actuation_params, u, Q) == 0){
		cout << "actuation type '" << "differential_drive" << "' not found. Aborting. . ." << endl;
		return ;
	}

	cout << "Prediction callback:\n" << u << "\n" << Q << endl;
	cout << "Is estimator running: " << est->running << endl;

	//Kalman predict
	if(est->running)
		estimatorPredict(est, "differential_drive", u, Q, ros::Time::now().toSec());

}

//measurement callback
void measurement_callback(const geometry_msgs::PoseWithCovarianceStampedPtr& msg){


	//extract measurement from message
	VectorXd x3D(3, 1);
	x3D << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

	//add measurement noise
	MatrixXd R3D;
	if(modelSetMeasurementNoise("camera_pose", node_params.measurement_params, x3D, R3D) == 0){
		cout << "measurement type '" << "camera_pose" << "' not found. Aborting. . ." << endl;
		return ;
	}

	//cout << "update_measurement\n" << x3D << "\n" << R3D << endl;

	//project measurement
	VectorXd x2D;
	MatrixXd R2D;
	if(modelConvertMeasurement("project_camera_pose", "camera_pose", node_params.measurement_params, x3D, R3D, x2D, R2D) == 0){
		cout << "cannot apply '" << "project_camera_pose" << "' to measurement type '" << "camera_pose" << "'. Aborting. . ." << endl;
		return ;
	}

	//cout << "update_measurement_transformed\n" << x2D << "\n" << R2D << endl;

	//Kalman update (select correct estimator)
	if(est->running)
		estimatorUpdate(est, "2d_position", x2D, R2D);
	else
		estimatorReset(est, x2D, ros::Time::now().toSec());

}


int main(int argc, char** argv){

	//Initialize ROS
	ros::init(argc, argv,"estimator/");
	ros::NodeHandle nh;

	//intialize parameters
	if(readConfigurationParams("/estimator/") == 0)
		return 0; //terminate the problem earlier

	//subscribe to topics
	ros::Subscriber prediction_sub = nh.subscribe("cmd_vel_fixed", 1, prediction_callback);
	ros::Subscriber update_sub = nh.subscribe("cam_pose", 1, measurement_callback);

	//initialize publishers
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimated_pose", 1);
	geometry_msgs::PoseWithCovarianceStamped msg_to_publish;

	//initialize loop rate
    ros::Rate loop_rate(RATE);
   
    //main loop
    while(ros::ok()){

    	//sleep acording to loop rate
        loop_rate.sleep();

        //age estimator (it will stop running after a while)
        estimatorIncrement(est);

        //publish data from estimator (if estimator is running)
        if(est->running){
	        msg_to_publish.header.stamp = ros::Time::now();
	        msg_to_publish.pose.pose.position.x = est->x(0);
	        msg_to_publish.pose.pose.position.y = est->x(1);
	        msg_to_publish.pose.pose.position.z = est->x(2); //0.0;
	        msg_to_publish.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, est->x(2));
	        msg_to_publish.pose.covariance[0] = est->P(0,0); msg_to_publish.pose.covariance[1] = est->P(0,1); msg_to_publish.pose.covariance[2] = est->P(0,2);
	        msg_to_publish.pose.covariance[3] = est->P(1,0); msg_to_publish.pose.covariance[4] = est->P(1,1); msg_to_publish.pose.covariance[5] = est->P(1,2);
			msg_to_publish.pose.covariance[6] = est->P(2,0); msg_to_publish.pose.covariance[7] = est->P(2,1); msg_to_publish.pose.covariance[8] = est->P(2,2);
	        pose_pub.publish(msg_to_publish);
    	}

        //read (prediction and update) messages
		ros::spinOnce();

    }

    return 0;
}
