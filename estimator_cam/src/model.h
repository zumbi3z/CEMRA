#ifndef __MODEL__
#define __MODEL__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
using namespace std;
using namespace Eigen;

//initializing state-space model
int modelInit(string model_name, VectorXd& x, MatrixXd& P, int& dim, MatrixXd& eye_kalman);

//generating motion model
int modelMotionModel(string model_name, void* parameters, string actuation_type, VectorXd x, VectorXd u, MatrixXd& F, VectorXd& f, MatrixXd& W, double dt);

//generating measurement model
int modelMeasurementModel(string model_name, void* parameters, string measurement_type, VectorXd x, VectorXd m, MatrixXd& H, VectorXd& h);

//motion models
int modelDifferentialDrive(string model_name, void* parameters, VectorXd x, VectorXd u, MatrixXd& F, VectorXd& f, MatrixXd& W, double dt);

//measurement models
int model2dPosition(string model_name, void* parameters, VectorXd x, VectorXd m, MatrixXd& H, VectorXd& h);

//noise models
void* modelConfigure(string file_name);
int modelSetActuationNoise(string actuation_type, void* parameters, VectorXd u, MatrixXd& Q);
int modelSetMeasurementNoise(string measurement_type, void* parameters, VectorXd m, MatrixXd& R);

//model conversions
int modelConvertMeasurement(string transformation_type, string measurement_type, void* parameters, VectorXd m1, MatrixXd R1, VectorXd& m2, MatrixXd& R2);

#endif //__MODEL__
