#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "imu_gnss_fusion/utils.h"

namespace cg {

constexpr int kStateDim = 15;
constexpr double kG = 9.81007;

class KF {
   public:
    struct State {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Matrix<double, kStateDim, kStateDim> cov;

        Eigen::Vector3d p_GI;
        Eigen::Vector3d v_GI;
        Eigen::Matrix3d r_GI;
        Eigen::Vector3d acc_bias;
        Eigen::Vector3d gyr_bias;

        double timestamp;

        State() {
            cov.setZero();

            p_GI.setZero();
            v_GI.setZero();
            r_GI.setIdentity();

            acc_bias.setZero();
            gyr_bias.setZero();
        }
    };
    using StatePtr = std::shared_ptr<State>;
    StatePtr state_ptr_;

    KF(double acc_n, double gyr_n, double acc_w, double gyr_w) : gravity_(Eigen::Vector3d(0, 0, -kG)),
                                                                 acc_noise_(acc_n),
                                                                 gyr_noise_(gyr_n),
                                                                 acc_bias_noise_(acc_w),
                                                                 gyr_bias_noise_(gyr_w) {
        state_ptr_ = std::make_shared<State>();

        const double kDegreeToRadian = M_PI / 180.;
        const double sigma_rp = 10. * kDegreeToRadian;
        const double sigma_yaw = 100. * kDegreeToRadian;

        state_ptr_->cov.block<3, 3>(0, 0) = 100. * Eigen::Matrix3d::Identity();                 // position std: 10 m
        state_ptr_->cov.block<3, 3>(3, 3) = 100. * Eigen::Matrix3d::Identity();                 // velocity std: 10 m/s
        state_ptr_->cov.block<2, 2>(6, 6) = sigma_rp * sigma_rp * Eigen::Matrix2d::Identity();  // roll pitch std 10 degree
        state_ptr_->cov(8, 8) = sigma_yaw * sigma_yaw;                                          // yaw std: 100 degree
        state_ptr_->cov.block<3, 3>(9, 9) = 0.0004 * Eigen::Matrix3d::Identity();               // Acc bias
        state_ptr_->cov.block<3, 3>(12, 12) = 0.0004 * Eigen::Matrix3d::Identity();             // Gyro bias
    }

    void predict(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) {
        const double delta_t = curr_imu->timestamp - last_imu->timestamp;
        const double delta_t2 = delta_t * delta_t;

        State last_state = *state_ptr_;

        const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + curr_imu->acc) - last_state.acc_bias;
        const Eigen::Vector3d gyr_unbias = 0.5 * (last_imu->gyr + curr_imu->gyr) - last_state.gyr_bias;

        // Normal state
        // Using P58. of "Quaternion kinematics for the error-state Kalman Filter".
        state_ptr_->p_GI = last_state.p_GI + last_state.v_GI * delta_t + 0.5 * (last_state.r_GI * acc_unbias + gravity_) * delta_t2;
        state_ptr_->v_GI = last_state.v_GI + (last_state.r_GI * acc_unbias + gravity_) * delta_t;
        const Eigen::Vector3d delta_angle_axis = gyr_unbias * delta_t;
        if (delta_angle_axis.norm() > 1e-12) {
            state_ptr_->r_GI = last_state.r_GI * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();
        }
        // Error-state. Not needed.

        // covariance of the error-state
        Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
        Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * delta_t;
        Fx.block<3, 3>(3, 6) = -state_ptr_->r_GI * cg::skew_matrix(acc_unbias) * delta_t;
        Fx.block<3, 3>(3, 9) = -state_ptr_->r_GI * delta_t;
        if (delta_angle_axis.norm() > 1e-12) {
            Fx.block<3, 3>(6, 6) = Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix().transpose();
        } else {
            Fx.block<3, 3>(6, 6).setIdentity();
        }
        Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * delta_t;

        Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
        Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

        Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
        Qi.block<3, 3>(0, 0) = delta_t2 * acc_noise_ * Eigen::Matrix3d::Identity();
        Qi.block<3, 3>(3, 3) = delta_t2 * gyr_noise_ * Eigen::Matrix3d::Identity();
        Qi.block<3, 3>(6, 6) = delta_t * acc_bias_noise_ * Eigen::Matrix3d::Identity();
        Qi.block<3, 3>(9, 9) = delta_t * gyr_bias_noise_ * Eigen::Matrix3d::Identity();

        state_ptr_->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();

        // timestamp
        state_ptr_->timestamp = curr_imu->timestamp;
    }

    void update_measurement(const Eigen::Vector3d &p_G_Gps, const Eigen::Matrix3d &cov, const Eigen::Vector3d &I_p_Gps) {
        const Eigen::Vector3d &p_GI = state_ptr_->p_GI;
        const Eigen::Matrix3d &r_GI = state_ptr_->r_GI;

        // residual
        Eigen::Vector3d residual = p_G_Gps - (p_GI + r_GI * I_p_Gps);

        // jacobian
        Eigen::Matrix<double, 3, 15> H;
        H.setZero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        H.block<3, 3>(0, 6) = -r_GI * cg::skew_matrix(I_p_Gps);

        // measurement covariance
        const Eigen::Matrix3d &R = cov;

        // compute K
        const Eigen::MatrixXd &P = state_ptr_->cov;
        const Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        const Eigen::VectorXd delta_x = K * residual;

        // update state vector
        state_ptr_->p_GI += delta_x.block<3, 1>(0, 0);
        state_ptr_->v_GI += delta_x.block<3, 1>(3, 0);
        if (delta_x.block<3, 1>(6, 0).norm() > 1e-12) {
            state_ptr_->r_GI *= Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(), delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
        }
        state_ptr_->acc_bias += delta_x.block<3, 1>(9, 0);
        state_ptr_->gyr_bias += delta_x.block<3, 1>(12, 0);

        // update state covariance
        const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
        state_ptr_->cov = I_KH * P * I_KH.transpose() + K * R * K.transpose();
    }

    ~KF() {}

   private:
    Eigen::Vector3d gravity_;
    double acc_noise_;
    double gyr_noise_;
    double acc_bias_noise_;
    double gyr_bias_noise_;
};

using KFPtr = std::unique_ptr<KF>;

}  // namespace cg