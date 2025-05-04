#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>

// abstract class for Kalman filter
// implementation could be KF/EKF/UKF...
namespace SORT 
{
    class KalmanFilter 
    {
    public:
        /**
         * user need to define H matrix & R matrix
         * @param num_states
         * @param num_obs
         */
        // constructor
        explicit KalmanFilter(unsigned int num_states, unsigned int num_obs);

        // destructor
        virtual ~KalmanFilter() = default;

        /**
         * Coast state and state covariance using the process model
         * User can use this function without change the internal
         * tracking state x_
         */
        virtual void Coast();

        std::tuple<double, double, double> covarianceEllipse(const Eigen::MatrixXd& P_predict, double deviations);

        /**
         * Predict without measurement update
         */
        void Predict();

        /**
         * This function maps the true state space into the observed space
         * using the observation model
         * User can implement their own method for more complicated models
         */
        virtual Eigen::VectorXd PredictionToObservation(const Eigen::VectorXd &state);

        /**
         * Updates the state by using Extended Kalman Filter equations
         * @param z The measurement at k+1
         */
        virtual void Update(const Eigen::VectorXd &z);

        // State vector
        Eigen::VectorXd x_, x_predict_;

        // Error covariance matrix
        Eigen::MatrixXd P_, P_predict_;

        // State transition matrix
        Eigen::MatrixXd F_;

        // Covariance matrix of process noise
        Eigen::MatrixXd Q_;

        // measurement matrix
        Eigen::MatrixXd H_;

        // covariance matrix of observation noise
        Eigen::MatrixXd R_;

        unsigned int num_states_, num_obs_;

        float log_likelihood_delta_;

        float NIS_;

        float alpha_;

        float Q_scale_factor_;

        float eps_max_; 

        int count_;

        std::tuple<double, double, double> ellipse_;
    };
}