#include <Eigen/Dense>

class KalmanFilter{

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
	KalmanFilter();

	KalmanFilter(
		double dt,
		const Eigen::MatrixXd& A,
		const Eigen::MatrixXd& C,
		const Eigen::MatrixXd& Q,
		const Eigen::MatrixXd& R,
		const Eigen::MatrixXd& P
	);

  	/**
  	* Initialize the filter with initial states as zero.
  	*/
	  void init();

  	/**
  	* Initialize the filter with a guess for initial states.
  	*/
  	void init(double t0, const Eigen::VectorXd& x0);

  	/**
  	* Update the estimated state based on measured values. The
  	* time step is assumed to remain constant.
  	*/
  	void update(const Eigen::VectorXd& z);

  	/**
  	* Update the estimated state based on measured values,
  	* using the given time step and dynamics matrix.
  	*/
  	void update(const Eigen::VectorXd& z, double dt, const Eigen::MatrixXd A);

  	Eigen::VectorXd getState(){ return x_hat; }
  	double getTime() { return t; };

private:
  	// Matrices for computation
  	Eigen::MatrixXd A, C, Q, R, P, S, K, P0;
    Eigen::VectorXd y;                    //measurement residual
  	// System dimensions
  	int m, n;                             //m: measurement  n:state

  	// Initial and current time
  	double t0, t;

  	// Discrete time step
  	double dt;

  	// Is the filter initialized?
  	bool initialized;

  	// n-size identity
  	Eigen::MatrixXd I;

  	// Estimated states
  	Eigen::VectorXd x_hat, x_hat_new;
};
