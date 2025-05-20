#pragma once

#include <cmath>
#include <tuple>

namespace SORT
{

    template <int STATE_DIM, int OBS_DIM>
    KalmanFilter<STATE_DIM, OBS_DIM>::KalmanFilter()
    {
        /*** Predict ***/
        // State vector
        x_.setZero();

        // Predicted(a prior) state vector
        x_predict_.setZero();

        // State transition matrix F_
        F_.setZero();

        // Error covariance matrix P
        P_.setZero();

        // Predicted(a prior) error covariance matrix
        // gives an estimate of the uncertainty or confidence in the predicted state (x_predict_)
        P_predict_.setZero();

        // Covariance matrix of process noise
        Q_.setZero();

        /*** Update ***/
        // Observation matrix
        H_.setZero();

        // Covariance matrix of observation noise
        R_.setZero();

        log_likelihood_delta_ = 0.0;
        NIS_ = 0.0;
        alpha_ = 1.0f; // Fading memory filter > 1 e.g. 4
        Q_scale_factor_ = 15.0f;
        eps_max_ = 1.6f;
        count_ = 0;
    }

    template <int STATE_DIM, int OBS_DIM>
    void KalmanFilter<STATE_DIM, OBS_DIM>::Coast()
    {
        x_predict_ = F_ * x_;
        P_predict_ = (alpha_ * alpha_) * F_ * P_ * F_.transpose() + Q_;
    }

    // template <int STATE_DIM, int OBS_DIM>
    // std::tuple<double, double, double> KalmanFilter<STATE_DIM, OBS_DIM>::covarianceEllipse(
    //     const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &P_predict, double deviations)
    // {
    //     // Extract the position covariance (2x2) for ellipse calculation
    //     Eigen::Matrix2d pos_cov = P_predict.template block<2, 2>(0, 0);

    //     // Use Eigen's SVD for the 2x2 position covariance
    //     Eigen::JacobiSVD<Eigen::Matrix2d> svd(pos_cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    //     Eigen::Matrix2d U = svd.matrixU();
    //     Eigen::Vector2d s = svd.singularValues();

    //     const double orientation = atan2(U(1, 0), U(0, 0));
    //     const double width = deviations * sqrt(s(0));
    //     const double height = deviations * sqrt(s(1));

    //     return {width, height, orientation};
    // }

    template <int STATE_DIM, int OBS_DIM>
    std::tuple<double, double, double> KalmanFilter<STATE_DIM, OBS_DIM>::covarianceEllipse(
        const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &P_predict, double deviations)
    {
        // Extract the 2x2 position covariance block (always the top-left corner)
        const auto &a = P_predict(0, 0);
        const auto &b = P_predict(0, 1);
        const auto &c = P_predict(1, 0);
        const auto &d = P_predict(1, 1);

        // Calculate eigenvalues directly (faster than SVD for 2x2 matrix)
        const double trace = a + d;
        const double det = a * d - b * c;
        const double discriminant = std::sqrt(std::max(0.0, trace * trace / 4 - det));
        const double eigenvalue1 = trace / 2 + discriminant;
        const double eigenvalue2 = trace / 2 - discriminant;

        // Calculate orientation using direct formula for 2x2
        const double orientation = std::atan2(eigenvalue1 - a, b);
        const double width = deviations * std::sqrt(std::max(0.0, eigenvalue1));
        const double height = deviations * std::sqrt(std::max(0.0, eigenvalue2));

        return {width, height, orientation};
    }

    template <int STATE_DIM, int OBS_DIM>
    void KalmanFilter<STATE_DIM, OBS_DIM>::Predict()
    {
        Coast();
        x_ = x_predict_;
        P_ = P_predict_;

        // Store the ellipse parameters after prediction
        ellipse_ = covarianceEllipse(P_, 1.0);
    }

    template <int STATE_DIM, int OBS_DIM>
    Eigen::Matrix<double, OBS_DIM, 1> KalmanFilter<STATE_DIM, OBS_DIM>::PredictionToObservation(
        const Eigen::Matrix<double, STATE_DIM, 1> &state)
    {
        return (H_ * state);
    }

    template <int STATE_DIM, int OBS_DIM>
    void KalmanFilter<STATE_DIM, OBS_DIM>::Update(const Eigen::Matrix<double, OBS_DIM, 1> &z)
    {
        // More efficient calculation of innovation (measurement residual)
        const Eigen::Matrix<double, OBS_DIM, 1> y = z - H_ * x_predict_;

        // Compute innovation covariance
        const Eigen::Matrix<double, OBS_DIM, OBS_DIM> S = H_ * P_predict_ * H_.transpose() + R_;

        // Use LDLT decomposition instead of direct inversion - faster and more stable
        const Eigen::Matrix<double, OBS_DIM, OBS_DIM> S_inv = S.ldlt().solve(Eigen::Matrix<double, OBS_DIM, OBS_DIM>::Identity());

        // Use noalias to avoid temporary matrix allocation
        const Eigen::Matrix<double, STATE_DIM, OBS_DIM> K = P_predict_ * H_.transpose() * S_inv;

        // Use noalias() to avoid temporary allocation
        x_.noalias() = x_predict_ + K * y;

        // Using Joseph form for better numerical stability
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> I = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
        P_ = (I - K * H_) * P_predict_ * (I - K * H_).transpose() + K * R_ * K.transpose();

        // Update ellipse parameters after state update
        ellipse_ = covarianceEllipse(P_, 1.0);
    }

    template <int STATE_DIM, int OBS_DIM>
    float KalmanFilter<STATE_DIM, OBS_DIM>::CalculateLogLikelihood(
        const Eigen::Matrix<double, OBS_DIM, 1> &y,
        const Eigen::Matrix<double, OBS_DIM, OBS_DIM> &S,
        const Eigen::Matrix<double, OBS_DIM, OBS_DIM> &S_inv)
    {
        // Use S which is already calculated in Update to avoid duplicate inversion
        double log_determinant = S.determinant() > 0 ? log(S.determinant()) : 0;

        // Compute log-likelihood
        float log_likelihood = -0.5f * (y.transpose() * S_inv * y + OBS_DIM * log(2 * M_PI) + log_determinant);

        return log_likelihood;
    }

} // namespace SORT