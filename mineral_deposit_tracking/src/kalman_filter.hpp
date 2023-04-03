#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <Eigen/Dense>

namespace mineral_deposit_tracking {

template<int StateSize>
class KalmanFilter {
    public:
        using VectorType = Eigen::Matrix<double, StateSize, 1>;
        using MatrixType = Eigen::Matrix<double, StateSize, StateSize>;

        const VectorType & GetEstimate() const {
            return estimate_;
        }

        const MatrixType & GetEstimateCovariance() const {
            return estimate_covariance_;
        }

        explicit KalmanFilter(const MatrixType &transition, const MatrixType &process, const MatrixType &observation)
                : transition_matrix_(transition),
                  process_covariance_(process),
                  observation_matrix_(observation),
                  estimate_(VectorType::Zero()),
                  estimate_covariance_(MatrixType::Identity() * 500) {
            GetEstimate();
        }

        void Reset(VectorType initial_state, MatrixType initial_covariance) {
            estimate_ = initial_state;
            estimate_covariance_ = initial_covariance;
        }

        void TimeUpdate() {
            estimate_ = transition_matrix_ * estimate_;
            estimate_covariance_ = transition_matrix_ * estimate_covariance_ * transition_matrix_.transpose() + process_covariance_;
        }

        void MeasurementUpdate(const VectorType &measurement, const MatrixType &measurement_covariance) {
            const MatrixType innovation_covariance = (observation_matrix_ * estimate_covariance_ * observation_matrix_.transpose()) + measurement_covariance;
            const MatrixType gain = estimate_covariance_ * observation_matrix_.transpose() * innovation_covariance.inverse();
            estimate_ = estimate_ + (gain * (measurement - (observation_matrix_ * estimate_)));
            const MatrixType tmp = MatrixType::Identity() - (gain * observation_matrix_);
            estimate_covariance_ = (tmp * estimate_covariance_ * tmp.transpose()) + (gain * measurement_covariance * gain.transpose());
        }


    private:
        const MatrixType transition_matrix_; // A
        const MatrixType process_covariance_; // Q
        const MatrixType observation_matrix_; // H
        VectorType estimate_;
        MatrixType estimate_covariance_;

};

}

#endif