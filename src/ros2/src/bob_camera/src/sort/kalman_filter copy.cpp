#include "include/kalman_filter.h"


SORT::KalmanFilter::KalmanFilter(unsigned int num_states, unsigned int num_obs) :
        num_states_(num_states), num_obs_(num_obs) {
    /*** Predict ***/
    // State vector
    x_ = Eigen::VectorXd::Zero(num_states);

    // Predicted(a prior) state vector
    x_predict_ = Eigen::VectorXd::Zero(num_states);

    // State transition matrix F_
    F_ = Eigen::MatrixXd::Zero(num_states, num_states);

    // Error covariance matrix P
    P_ = Eigen::MatrixXd::Zero(num_states, num_states);

    // Predicted(a prior) error covariance matrix
    // gives an estimate of the uncertainty or confidence in the predicted state (x_predict_)
    P_predict_ = Eigen::MatrixXd::Zero(num_states, num_states);

    // Covariance matrix of process noise
    Q_ = Eigen::MatrixXd::Zero(num_states, num_states);

    /*** Update ***/
    // Observation matrix
    H_ = Eigen::MatrixXd::Zero(num_obs, num_states);

    // Covariance matrix of observation noise
    R_ = Eigen::MatrixXd::Zero(num_obs, num_obs);

    log_likelihood_delta_ = 0.0;
    NIS_ = 0.0;
    alpha_ = 1.0f; // Fading memory filter > 1 e.g. 4
    Q_scale_factor_ = 15.0f;
    eps_max_ = 1.6f;
    count_ = 0;
}

void SORT::KalmanFilter::Coast() 
{
    x_predict_ = F_ * x_;
    P_predict_ = pow(alpha_,2) * F_ * P_ * F_.transpose() + Q_;
}

std::tuple<double, double, double> SORT::KalmanFilter::covarianceEllipse(const Eigen::MatrixXd& P_predict, double deviations)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(P_predict, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::VectorXd s = svd.singularValues();

    double orientation = atan2(U(1, 0), U(0, 0));
    double width = deviations * sqrt(s(0));
    double height = deviations * sqrt(s(1));

    return std::make_tuple(width, height, orientation);
}


void SORT::KalmanFilter::Predict() 
{
    Coast();
    x_ = x_predict_;
    P_ = P_predict_;

    // double deviations = 1.0;
    // auto newEllipse = covarianceEllipse(P_predict_, deviations);

    // // Apply smoothing to the ellipse parameters
    // constexpr double smoothingFactor = 0.95;  // Adjust this value based on your requirements

    // std::get<0>(ellipse_) = smoothingFactor * std::get<0>(ellipse_) + (1.0 - smoothingFactor) * std::get<0>(newEllipse);
    // std::get<1>(ellipse_) = smoothingFactor * std::get<1>(ellipse_) + (1.0 - smoothingFactor) * std::get<1>(newEllipse);
    // std::get<2>(ellipse_) = smoothingFactor * std::get<2>(ellipse_) + (1.0 - smoothingFactor) * std::get<2>(newEllipse);
}

Eigen::VectorXd SORT::KalmanFilter::PredictionToObservation(const Eigen::VectorXd &state) 
{
    return (H_*state);
}


void SORT::KalmanFilter::Update(const Eigen::VectorXd& z) 
{
    
    Eigen::VectorXd z_predict = PredictionToObservation(x_predict_); // Predicted observation based on prior state
    Eigen::VectorXd y = z - z_predict; // Innovation: Difference between observed and predicted measurements
    Eigen::MatrixXd Ht = H_.transpose(); // Transposed observation matrix
    Eigen::MatrixXd S = H_ * P_predict_ * Ht + R_; // Innovation covariance
    NIS_ = y.transpose() * S.inverse() * y; // Normalized Innovation Squared: Measures consistency of filter

    // Basic adaptive filtering
    if(NIS_ > eps_max_)
    {
        Q_ *= Q_scale_factor_;
        count_ += 1;
    }
    else if(count_ > 0)
    {
        Q_ /= Q_scale_factor_;
        count_ -= 1;
    }

    /* 
    P_predict provides confidence in prediction. Smaller values mean higher confidence.
    Discrepancies between actual error (y) and P_predict may point to inaccuracies in filter's model or noise settings.
    */
    /*
    std::cout << "\nP_predict (Predicted Error Covariance):\n" << P_predict_ << std::endl;
    std::cout << "\nZ (Actual Observation):\n" << z << std::endl;
    std::cout << "\nZ_pred (Predicted Observation):\n" << z_predict << std::endl;
    std::cout << "\ny (Innovation):\n" << y << std::endl;
    std::cout << "\nS (Innovation Covariance):\n" << S << std::endl;
    std::cout << "\nNIS (Normalized Innovation Squared): " << NIS_ << std::endl;
    std::cout << "=================================\n" << std::endl << std::endl;
    */

    // Compute Kalman gain
    Eigen::MatrixXd K = P_predict_ * Ht * S.inverse();

    // Update state estimation based on Kalman gain and innovation
    x_ = x_predict_ + K * y;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_states_, num_states_);
    // Joseph form
    // P_ = (I - K * H_) * P_predict_ * (I - K * H_).transpose() + K * R_ * K.transpose();
    // Update error covariance using Joseph form (commented) or Optimal gain
    P_ = (I - K * H_) * P_predict_;

    // Calculate the log-likelihood of the measurement
    // float log_likelihood = CalculateLogLikelihood(y, S);

    // Output or store the log-likelihood value for further analysis or adaptive filtering
    // std::cout << "Log-Likelihood: " << log_likelihood << std::endl;
}

// Quantifies how probable a given measurement (or observation) is, given the predicted state and the associated uncertainties
// Not presently used
float SORT::KalmanFilter::CalculateLogLikelihood(const Eigen::VectorXd& y, const Eigen::MatrixXd& S) {
    float log_likelihood;

    // Note: Computing log(M.determinant()) in Eigen C++ is risky for large matrices since it may overflow or underflow.
    // compute the Cholesky decomposition of the innovation covariance matrix, because it is symmetric
    // S = L * L^T = U^T * U
    // then retrieve factor L in the decomposition
    auto& L = S.llt().matrixL();

    // find log determinant of innovation covariance matrix
    float log_determinant = 0;
    for (unsigned int i = 0; i < S.rows(); i++)
        log_determinant += log(L(i, i));
    log_determinant *= 2;

    // log-likelihood expression for current iteration
    log_likelihood = -0.5 * (y.transpose() * S.inverse() * y + num_obs_ * log(2 * M_PI) + log_determinant);

    if (std::isnan(log_likelihood)) {
    	log_likelihood = -1e50;
	}

	return log_likelihood;
}