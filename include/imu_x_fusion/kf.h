#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "imu_x_fusion/utils.h"

namespace cg {

constexpr int kStateDim = 15;
constexpr int kNoiseDim = 12;
constexpr double kG = 9.81007;

const double kDegreeToRadian = M_PI / 180.;

using MatrixSD = Eigen::Matrix<double, kStateDim, kStateDim>;

class KF {
   public:
    struct State {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // error-state
        MatrixSD cov;

        // nominal-state
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

    KF() = delete;

    KF(const KF &) = delete;

    explicit KF(double acc_n = 1e-2, double gyr_n = 1e-4, double acc_w = 1e-6, double gyr_w = 1e-8)
        : acc_noise_(acc_n),
          gyr_noise_(gyr_n),
          acc_bias_noise_(acc_w),
          gyr_bias_noise_(gyr_w) {
        state_ptr_ = std::make_shared<State>();

        const double sigma_rp = 10. * kDegreeToRadian;
        const double sigma_yaw = 100. * kDegreeToRadian;
    }

    void set_cov(double sigma_p, double sigma_v, double sigma_rp, double sigma_yaw, double sigma_ba, double sigma_bg) {
        state_ptr_->cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sigma_p * sigma_p;        // position std: sigma_p m
        state_ptr_->cov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * sigma_v * sigma_v;        // velocity std: sigma_v m/s
        state_ptr_->cov.block<2, 2>(6, 6) = Eigen::Matrix2d::Identity() * sigma_rp * sigma_rp;      // roll pitch std
        state_ptr_->cov(8, 8) = sigma_yaw * sigma_yaw;                                              // yaw std
        state_ptr_->cov.block<3, 3>(9, 9)   = Eigen::Matrix3d::Identity() * sigma_ba * sigma_ba;    // Acc bias
        state_ptr_->cov.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * sigma_bg * sigma_bg;    // Gyr bias
    }    

    /**
     * @brief predict procedure
     * @ref ESKF 5.4: System kinematics in discrete time
     * @param last_imu 
     * @param curr_imu 
     */
    void predict(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) {
        const double dt = curr_imu->timestamp - last_imu->timestamp;
        const double dt2 = dt * dt;

        State last_state = *state_ptr_;

        // timestamp
        state_ptr_->timestamp = curr_imu->timestamp;

        //
        // ESKF 5.4.1 The nominal state kinematics
        //

        // p v R
        const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + curr_imu->acc) - last_state.acc_bias;
        const Eigen::Vector3d gyr_unbias = 0.5 * (last_imu->gyr + curr_imu->gyr) - last_state.gyr_bias;
        const Eigen::Vector3d acc_nominal = last_state.r_GI * acc_unbias + Eigen::Vector3d(0, 0, -kG);
        state_ptr_->p_GI = last_state.p_GI + last_state.v_GI * dt + 0.5 * acc_nominal * dt2;
        state_ptr_->v_GI = last_state.v_GI + acc_nominal * dt;
        const Eigen::Vector3d delta_angle_axis = gyr_unbias * dt;
        double norm_delta_angle = delta_angle_axis.norm();
        Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
        if (norm_delta_angle > DBL_EPSILON) {
            dR = Eigen::AngleAxisd(norm_delta_angle, delta_angle_axis.normalized()).toRotationMatrix();
            state_ptr_->r_GI = last_state.r_GI * dR;
        }

        //
        // ESKF 5.4.3 The error-state Jacobian and perturbation matrices
        //

        // Fx
        MatrixSD Fx = MatrixSD::Identity();
        Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        Fx.block<3, 3>(3, 6) = -state_ptr_->r_GI * cg::skew_matrix(acc_unbias) * dt;
        Fx.block<3, 3>(3, 9) = -state_ptr_->r_GI * dt;
        if (norm_delta_angle > DBL_EPSILON) {
            Fx.block<3, 3>(6, 6) = dR.transpose();
        } else {
            Fx.block<3, 3>(6, 6).setIdentity();
        }
        Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

        // Fi
        Eigen::Matrix<double, kStateDim, kNoiseDim> Fi = Eigen::Matrix<double, kStateDim, kNoiseDim>::Zero();
        Fi.block<12, kNoiseDim>(3, 0) = Eigen::Matrix<double, 12, kNoiseDim>::Identity();

        // Qi
        Eigen::Matrix<double, kNoiseDim, kNoiseDim> Qi = Eigen::Matrix<double, kNoiseDim, kNoiseDim>::Zero();
        Qi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt2 * acc_noise_;
        Qi.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * dt2 * gyr_noise_;
        Qi.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * dt * acc_bias_noise_;
        Qi.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * dt * gyr_bias_noise_;

        // P: error-state covariance
        state_ptr_->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();
    }

    /**
     * @brief measurement update procedure
     * @ref ESKF 6: Fusing IMU with complementary sensory data
     * @tparam H_type 
     * @tparam Res_type 
     * @tparam R_type 
     * @param H 
     * @param V 
     * @param r 
     */
    template <class H_type, class Res_type, class R_type>
    void update_measurement(
        const Eigen::MatrixBase<H_type> &H, const Eigen::MatrixBase<R_type> &V, const Eigen::MatrixBase<Res_type> &r) {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(H_type);
        EIGEN_STATIC_ASSERT_FIXED_SIZE(R_type);

        // compute K
        const MatrixSD &P = state_ptr_->cov;
        const R_type S = H * P * H.transpose() + V;
        const Eigen::Matrix<double, kStateDim, R_type::RowsAtCompileTime> K = P * H.transpose() * S.inverse();

        // delta_x
        const Eigen::Matrix<double, kStateDim, 1> delta_x = K * r;

        // update: nominal-state + observation error
        state_ptr_->p_GI += delta_x.block<3, 1>(0, 0);
        state_ptr_->v_GI += delta_x.block<3, 1>(3, 0);
        const Eigen::Vector3d &dR = delta_x.block<3, 1>(6, 0);
        if (dR.norm() > DBL_EPSILON) {
            state_ptr_->r_GI *= Eigen::AngleAxisd(dR.norm(), dR.normalized()).toRotationMatrix();
        }
        state_ptr_->acc_bias += delta_x.block<3, 1>(9, 0);
        state_ptr_->gyr_bias += delta_x.block<3, 1>(12, 0);

        // update: error-state covariance
        const MatrixSD I_KH = MatrixSD::Identity() - K * H;
        state_ptr_->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
    }

    static bool init_rot_from_imudata(Eigen::Matrix3d &r_GI, const std::deque<ImuDataConstPtr> &imu_buf) {
        // mean and std of IMU accs
        Eigen::Vector3d sum_acc(0., 0., 0.);
        for (const auto imu_data : imu_buf) {
            sum_acc += imu_data->acc;
        }
        const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buf.size();
        printf("[cggos %s] mean_acc: (%f, %f, %f)!!!\n", __FUNCTION__, mean_acc[0], mean_acc[1], mean_acc[2]);

        Eigen::Vector3d sum_err2(0., 0., 0.);
        for (const auto imu_data : imu_buf) sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
        const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buf.size()).cwiseSqrt();

        // acc std limit: 3
        if (std_acc.maxCoeff() > 3.0) {
            printf("[cggos %s] Too big acc std: (%f, %f, %f)!!!\n", __FUNCTION__, std_acc[0], std_acc[1], std_acc[2]);
            return false;
        }

        // Compute rotation.
        // ref: https://github.com/rpng/open_vins/blob/master/ov_core/src/init/InertialInitializer.cpp

        // Three axises of the ENU frame in the IMU frame.
        // z-axis
        const Eigen::Vector3d &z_axis = mean_acc.normalized();

        // x-axis
        Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
        x_axis.normalize();

        // y-axis
        Eigen::Vector3d y_axis = z_axis.cross(x_axis);
        y_axis.normalize();

        Eigen::Matrix3d r_IG;
        r_IG.block<3, 1>(0, 0) = x_axis;
        r_IG.block<3, 1>(0, 1) = y_axis;
        r_IG.block<3, 1>(0, 2) = z_axis;

        r_GI = r_IG.transpose();

        return true;
    }    

    ~KF() {}

   private:
    double acc_noise_;
    double gyr_noise_;
    double acc_bias_noise_;
    double gyr_bias_noise_;
};

using KFPtr = std::unique_ptr<KF>;

}  // namespace cg