#include "model.h"
#include <fstream>

//initializing state-space model
int modelInit(string model_name, VectorXd& x, MatrixXd& P, int& dim, MatrixXd& eye_kalman){

	//go through possible models
	if(strcmp(model_name.c_str(), "differential_drive") == 0){
		x = VectorXd(3, 1);
		P = MatrixXd(3, 3);
		dim = 3;
		eye_kalman = MatrixXd::Identity(3, 3);
		return 1;
	}
	else return 0; //no model was found

	return 0;

}

//generating motion model
int modelMotionModel(string model_name, void* parameters, string actuation_type, VectorXd x, VectorXd u, MatrixXd& F, VectorXd& f, MatrixXd& W, double dt){

	//go through possible models
	if(strcmp(actuation_type.c_str(), "differential_drive") == 0){
		return modelDifferentialDrive(model_name, parameters, x, u, F, f, W, dt);
	}
	else return 0; //no model was found

	return 0;

}

//generating measurement model
int modelMeasurementModel(string model_name, void* parameters, string measurement_type, VectorXd x, VectorXd m, MatrixXd& H, VectorXd& h){
	
	//go through possible models
	if(strcmp(measurement_type.c_str(), "2d_position") == 0){
		return model2dPosition(model_name, parameters, x, m, H, h);
	}
	else return 0; //no model was found

	return 0;
}

//--------------------------------------------------------motion models--------------------------------------------------------//
int modelDifferentialDrive(string model_name, void* parameters, VectorXd x, VectorXd u, MatrixXd& F, VectorXd& f, MatrixXd& W, double dt){

	//checking for correct state-space model
	if(strcmp(model_name.c_str(), "differential_drive") != 0) return 0;

	//decoding actuation into linear and angular speeds
	double v = u(0);
	double w = u(1);

	//model
	f = VectorXd(3, 1);
	f(0) = x(0) + (dt*v)*cos(x(2));
	f(1) = x(1) + (dt*v)*sin(x(2));
	f(2) = x(2) + (dt*w);

	//Jacobian
	F = MatrixXd(3, 3);
	F << 1, 0, -(dt*v)*sin(x(2)),
	     0, 1,  (dt*v)*cos(x(2)),
	     0, 0,  1;

	//noise term
	W = MatrixXd(3, 2);
	W << dt*cos(x(2)), 0,
	     dt*sin(x(2)), 0,
	                0, dt;

	//success in computing the motion model
	return 1;

}

//--------------------------------------------------------measurement models--------------------------------------------------------//
int model2dPosition(string model_name, void* parameters, VectorXd x, VectorXd m, MatrixXd& H, VectorXd& h){

	//checking for correct state-space model
	if(strcmp(model_name.c_str(), "differential_drive") != 0) return 0;

	//model
	h = m;

	//jacobian
	H = MatrixXd(2, 3);
	H << 1, 0, 0,
	     0, 1, 0;

	//success in computing the measurement model
	return 1;

}

//--------------------------------------------------------noise models--------------------------------------------------------//

//model structures
typedef struct t_differential_drive_parameters{
	double q1;
	double q2;
} differential_drive_parameters;
typedef struct t_camera_parameters{
	double vp;
	double vh;
	Matrix3d R;
	Vector3d t;
} camera_parameters;

//model configurations
void* DifferentialDriveModelInit(ifstream& infile){

	//create parameter structure
	differential_drive_parameters* ddrive = new differential_drive_parameters;

	//get parameters
	string garbadge;
	if( !(infile >> garbadge >> ddrive->q1) ) {delete ddrive; return NULL;}
	if( !(infile >> garbadge >> ddrive->q2) ) {delete ddrive; return NULL;}

	//cout << "differential_drive_params: " << ddrive->q1 << " " << ddrive->q2 << endl;

	//return structure
	return (void*)ddrive;

}
void* CameraModelInit(ifstream& infile){

	//create parameter structure
	camera_parameters* cam = new camera_parameters;

	//get parameters
	string garbadge;
	if( !(infile >> garbadge >> cam->vp) ) {delete cam; return NULL;}
	if( !(infile >> garbadge >> cam->vh) ) {delete cam; return NULL;}
	if( !(infile >> garbadge >> cam->R(0,0) >> cam->R(0,1) >> cam->R(0,2) >>\
		                        cam->R(1,0) >> cam->R(1,1) >> cam->R(1,2) >>\
		                        cam->R(2,0) >> cam->R(2,1) >> cam->R(2,2)) ) {delete cam; return NULL;}
	if( !(infile >> garbadge >> cam->t(0) >> cam->t(1) >> cam->t(2)) ) {delete cam; return NULL;}

	//cout << "cam_pose_params: " << cam->vp << " " << cam->vh << "\n" << cam->R << "\n" << cam->t << endl;

	//return structure
	return (void*)cam;

}


//individual models
void DifferentialDriveModel(differential_drive_parameters* ddrive, VectorXd u, MatrixXd& Q){

	//the error is a percentage of the actuation
	Q = MatrixXd::Identity(2, 2);
	Q(0, 0) = 4 * (ddrive->q1 * u(0)) * (ddrive->q1 * u(0));
	Q(1, 1) = 4 * (ddrive->q2 * u(1)) * (ddrive->q2 * u(1));

	//giving a minimum noise
	if(Q(0, 0) < 0.005 * 0.005) Q(0, 0) = 4 * 0.005 * 0.005;
	if(Q(1, 1) < 0.03 * 0.03) Q(1, 1) = 4 * 0.03 * 0.03;

	//ignorying drifts or bias that might occur

}
void LongitudinalTransversalModel(camera_parameters* cam, VectorXd m, MatrixXd& R){

	//longitudinal-transversal model (this one does not depend on range)
	R = MatrixXd(3,3);
	R << cam->vp, 0, 0, 0, cam->vh, 0, 0, 0, cam->vh;

	//apply to measurement
	Matrix3d Rot;
	Vector3d x1, x2, u;
	double theta, ct, st;
	x1<<1,0,0; x1/=x1.norm();
	x2<<m(0),m(1),m(2); x2/=x2.norm();
	u=x1.cross(x2); u/=u.norm();
	theta=acos(x1.dot(x2)/(x1.norm()*x2.norm()));
	ct=cos(theta); st=sin(theta);
	Rot<<ct+u(0)*u(0)*(1-ct),u(0)*u(1)*(1-ct)-u(2)*st,u(0)*u(2)*(1-ct)+u(1)*st,
           u(1)*u(0)*(1-ct)+u(2)*st,ct+u(1)*u(1)*(1-ct),u(1)*u(2)*(1-ct)-u(0)*st,
           u(2)*u(0)*(1-ct)-u(1)*st,u(2)*u(1)*(1-ct)+u(0)*st,ct+u(2)*u(2)*(1-ct);
    R = Rot*R*Rot.transpose();

}

//general purpouse functions
void* modelConfigure(string file_name){

	//open file
	ifstream infile(file_name);

	//read name of the model
	string model_name;
	if( !(infile >> model_name) ) return (void*)NULL;



	//go through possible models
	if(strcmp(model_name.c_str(), "differential_drive") == 0)
		return DifferentialDriveModelInit(infile);
	else if(strcmp(model_name.c_str(), "camera_pose") == 0)
		return CameraModelInit(infile);
	else return (void*)NULL;

}
int modelSetActuationNoise(string actuation_type, void* parameters, VectorXd u, MatrixXd& Q){

	//go through possible actuation types
	if(strcmp(actuation_type.c_str(), "differential_drive") == 0){
		DifferentialDriveModel( (differential_drive_parameters*)parameters, u, Q);
		return 1;
	}
	else return 0; //no matching actuation type was found

	return 0;

}
int modelSetMeasurementNoise(string measurement_type, void* parameters, VectorXd m, MatrixXd& R){

	//go through possible measurement types
	if(strcmp(measurement_type.c_str(), "camera_pose") == 0){
		LongitudinalTransversalModel( (camera_parameters*)parameters, m, R);
		return 1;
	}
	else return 0; //no matching measurement type was found

	return 0;

}

//--------------------------------------------------------model convertions--------------------------------------------------------//
//takes a vehicle pose measurement acquired by the camera and computes its projection in the ground plane
//the uncertainty of the measurement is transformed accordingly
void measurementProjection(camera_parameters* cam, VectorXd x3D, MatrixXd R3D, VectorXd& x2D, MatrixXd& R2D){

	//defining the projection matrix
	MatrixXd Pr(2, 3);
	MatrixXd Pr2;
	Pr << 1, 0, 0,
	      0, 1, 0;

	//computing 2D measurement (also transforming the uncertainty)
	x2D = Pr * (cam->R*x3D + cam->t);
	Pr2 = (Pr*cam->R);
	R2D = (Pr*cam->R) * R3D * Pr2.transpose();

}

int modelConvertMeasurement(string transformation_type, string measurement_type, void* parameters, VectorXd m1, MatrixXd R1, VectorXd& m2, MatrixXd& R2){

	//go through possible measurement transformations
	if(strcmp(transformation_type.c_str(), "project_camera_pose") == 0){
		if(strcmp(measurement_type.c_str(), "camera_pose") == 0){
			measurementProjection( (camera_parameters*)parameters, m1, R1, m2, R2);
			return 1;
		}
		else return 0;
	}
	else return 0; //no matching measurement transformation was found

	return 0;

}
