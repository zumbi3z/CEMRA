#ifndef __ESTIMATOR__
#define __ESTIMATOR__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
using namespace std;
using namespace Eigen;

typedef struct t_estimator{
	string name;
	string model_name;
	VectorXd x;
	MatrixXd P;
	int dim;
	MatrixXd eye_kalman;
	void* parameters;
	double t_last;
	int timeout, t;
	int running;
} estimator;

//create estimator and initialize its parameters (acording to the given space-state model)
estimator* estimatorInit(string name, string model_name, void* parameters);

//insert initial conditions into the estimator (the estimator will be marked as running)
void estimatorReset(estimator* est, VectorXd x, VectorXd p, double t_current, int start_estimator);

//Kalman predict
void estimatorPredict(estimator* est, string actuation_type, VectorXd u, MatrixXd& Q, double t_current);

//Kalman random walk
void estimatorRandomWalk(estimator* est, string actuation_type, VectorXd u, double t_current);

//Kalman update
void estimatorUpdate(estimator* est, string measurement_type, VectorXd m, MatrixXd& R);

//clean old estimators
void estimatorIncrement(estimator* est);

#endif //__ESTIMATOR__
