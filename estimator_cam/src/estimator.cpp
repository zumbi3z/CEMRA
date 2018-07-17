#include "estimator.h"
#include "model.h"

//create estimator and initialize its parameters (acording to the given space-state model)
estimator* estimatorInit(string name, string model_name, void* parameters){

	//create estimator object
	estimator* est = new estimator;

	//fill the estimator parameters
	est->name = name;
	est->model_name = model_name;
	if(modelInit(model_name, est->x, est->P, est->dim, est->eye_kalman) == 0){
		cout << "Model name '" << model_name << "' was not found. Aborting estimator initialization..." << endl;
		return NULL;
	}
	est->parameters = parameters; //these parameters need to have beed initialized before
	est->timeout = 10;
	est->t = 0;
	est->running = 0;

	//cout << "estimator:\n" << est->name << "\n" << est->model_name << "\n" << est->x << "\n" << est->P <<"\n" << est->dim << endl; 

	//return estimator to the user
	return est;

}

//insert initial conditions into the estimator (the estimator will be marked as running)
void estimatorReset(estimator* est, VectorXd x, VectorXd p, double t_current, int start_estimator){

	//insert initial conditions
	est->x = x;
	est->P = MatrixXd::Identity(est->dim, est->dim);
	for(int k = 0; k < est->dim; k++)
		est->P(k, k) = p(k);

	//initializing time
	est->t_last = t_current;

	//reset timout counter
	est->t = 0;

	//run estimator
	est->running = start_estimator;

	//cout << "estimator:\n" << est->name << "\n" << est->model_name << "\n" << est->x << "\n" << est->P <<"\n" << est->dim << "\n" << est->t_last << endl;

}

//Kalman predict
void estimatorPredict(estimator* est, string actuation_type, VectorXd u, MatrixXd& Q, double t_current){

	//do nothing if estimator is not running
	if(!est->running) return ;

	//compute period of time between predictions
	double dt =  t_current - est->t_last;
	est->t_last = t_current;

	//generate motion model
	MatrixXd F;
	MatrixXd W;
	VectorXd f;
	if(modelMotionModel(est->model_name, est->parameters, actuation_type, est->x, u, F, f, W, dt) == 0){
		cout << "Actuation model '" << actuation_type << "' was not found. Aborting predit..." << endl;
		return ;
	}

	//cout << "Predict:\n" << u << "\n" << F << "\n" << W << "\n" << dt << endl;
	//cout << "Start:\n" << est->x << "\n" << est->P << endl;

	//apply Kalman predict
	est->x = f;
	est->P = F*est->P*F.transpose() + W*Q*W.transpose();

	//cout << "Result:\n" << est->x << "\n" << est->P << endl;

	//reset timout counter
	est->t = 0;

}

//Kalman random walk
void estimatorRandomWalk(estimator* est, string actuation_type, VectorXd u, double t_current){

	//do nothing if estimator is not running
	if(!est->running) return ;

	//compute period of time between predictions
	double dt =  t_current - est->t_last;
	est->t_last = t_current;

	//generate motion model
	MatrixXd F;
	MatrixXd W;
	VectorXd f;
	if(modelMotionModel(est->model_name, est->parameters, actuation_type, est->x, u, F, f, W, dt) == 0){
		cout << "Actuation model '" << actuation_type << "' was not found. Aborting predit..." << endl;
		return ;
	}

	//cout << "Predict:\n" << u << "\n" << F << "\n" << W << "\n" << dt << endl;
	//cout << "Start:\n" << est->x << "\n" << est->P << endl;

	//apply Kalman predict
	MatrixXd Q = 0.1*MatrixXd::Identity(u.size(), u.size());
	MatrixXd Q1 = 0.0025*MatrixXd::Identity(est->dim, est->dim);
	est->P = F*est->P*F.transpose() + Q1 + W*Q*W.transpose();
	//est->P += Q;
}

//Kalman update
void estimatorUpdate(estimator* est, string measurement_type, VectorXd m, MatrixXd& R){

	//do nothing if estimator is not running
	if(!est->running) return ;

	//generate measurement model
	MatrixXd H;
	VectorXd h;
    if(modelMeasurementModel(est->model_name, est->parameters, measurement_type, est->x, m, H, h) == 0){
		cout << "Measurement model '" << measurement_type << "' was not found. Aborting update..." << endl;
		return ;
	}

	//cout << "Update:\n" << m << "\n" << H << "\n" << h << "\n" << est->eye_kalman << endl;
	//cout << "Start:\n" << est->x << "\n" << est->P << endl;

	//apply Kalman update
	MatrixXd R1 = H*est->P*H.transpose() + R;
	MatrixXd K = est->P*H.transpose()*R1.inverse();
	est->x = est->x + K*(h - H*est->x);
	est->P = (est->eye_kalman - K*H)*est->P;

	//cout << "Result:\n" << est->x << "\n" << est->P << endl;

	//reset timout counter
	est->t = 0;

}

//clean old estimators
void estimatorIncrement(estimator* est){

	//do nothing if estimator is not running
	if(!est->running) return ;

	//increment estimator timer
	est->t++;
	if(est->t > est->timeout)
		est->running = 0; //shutdown estimator if enough time has passed without predicts or updates
}
