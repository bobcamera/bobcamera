#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>

// Template class for Kalman filter
// implementation could be KF/EKF/UKF...
namespace SORT
{
    template <int STATE_DIM, int OBS_DIM>
    class KalmanFilter
    {
    public:
        /**
         * user need to define H matrix & R matrix
         */
        // constructor
        explicit KalmanFilter();

        // destructor
        virtual ~KalmanFilter() = default;

        /**
         * Coast state and state covariance using the process model
         * User can use this function without change the internal
         * tracking state x_
         */
        virtual void Coast();

        std::tuple<double, double, double> covarianceEllipse(const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &P_predict, double deviations = 1.0);

        /**
         * Predict without measurement update
         */
        void Predict();

        /**
         * This function maps the true state space into the observed space
         * using the observation model
         * User can implement their own method for more complicated models
         */
        virtual Eigen::Matrix<double, OBS_DIM, 1> PredictionToObservation(const Eigen::Matrix<double, STATE_DIM, 1> &state);

        /**
         * Updates the state by using Extended Kalman Filter equations
         * @param z The measurement at k+1
         */
        virtual void Update(const Eigen::Matrix<double, OBS_DIM, 1> &z);

        /**
         * Calculate marginal log-likelihood to evaluate different parameter choices
         */
        float CalculateLogLikelihood(const Eigen::Matrix<double, OBS_DIM, 1> &y, const Eigen::Matrix<double, OBS_DIM, OBS_DIM> &S,
                                     const Eigen::Matrix<double, OBS_DIM, OBS_DIM> &S_inv);

        // State vector
        Eigen::Matrix<double, STATE_DIM, 1> x_, x_predict_;

        // Error covariance matrix
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_, P_predict_;

        // State transition matrix
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> F_;

        // Covariance matrix of process noise
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q_;

        // measurement matrix
        Eigen::Matrix<double, OBS_DIM, STATE_DIM> H_;

        // covariance matrix of observation noise
        Eigen::Matrix<double, OBS_DIM, OBS_DIM> R_;

        // These constants remain for compatibility with existing code
        static constexpr unsigned int num_states_ = STATE_DIM;
        static constexpr unsigned int num_obs_ = OBS_DIM;

        float log_likelihood_delta_;
        float NIS_;
        float alpha_;
        float Q_scale_factor_;
        float eps_max_;
        int count_;

        std::tuple<double, double, double> ellipse_;
    };
}

// Include the implementation
#include "kalman_filter.hpp"