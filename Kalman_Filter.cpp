#include "Kalman_Filter.h"

KalmanFilter::KalmanFilter(	
		double dt,
		const Eigen::MatrixXd& A,
		const Eigen::MatrixXd& C,
		const Eigen::MatrixXd& Q,
		const Eigen::MatrixXd& R,
		const Eigen::MatrixXd& P)
	: A(A), C(C), Q(Q), R(R), P0(P), m(C.rows()), n(A.rows()), dt(dt)
	, initialized(false), I(n, n), x_hat(n), x_hat_new(n)
{
	I.setIdentity();
}

KalmanFilter::KalmanFilter(){}

void KalmanFilter::init()
{
	x_hat.setZero();
	P = P0;
  	t0 = 0;
  	t = t0;
  	initialized = true;
}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0)
{
	x_hat = x0;
	P = P0;
	this->t0 = t0;
	t = t0;
	initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& z)
{
	if(!initialized)
    	throw std::runtime_error("Filter is not initialized!");

    //PREDICT
    x_hat_new = A * x_hat;     //x(k|k-1) = A*x(k-1) 
    P = A*P*A.transpose() + Q; //P(k|k-1) = A*P(k-1)*A + Q

    //CORRECT
    S = C*P*C.transpose() + R;             //S(K) = C*P(k|k-1)*C + R
    //K = P*C.transpose()*S.inverse();	   //K(K) = P(k|k-1)*C'*S(K).inv
    K = P*C.transpose()*(S+C*P*C.transpose()).inverse();

    y = z-C*x_hat_new;            //y(k) = z(k) - C*x(k|k-1)
    x_hat_new += K*y;             //x(k) = x(k|k-1) + K*y

    P = (I - K*C)*P;               //P(k) = (I-KC)*P(k|k-1)
    x_hat = x_hat_new;

    t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& z, double dt, const Eigen::MatrixXd A)
{
	this->A = A;
	this->dt = dt;
	update(z);
}
