#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <deque>
#include <iostream>

#include "common/utils.h"
#include "common/view.hpp"
#include "estimator/ekf.hpp"
#include "sensor/imu.hpp"

namespace cg {

constexpr int kMeasDim = 6;

ANGULAR_ERROR State::kAngError = ANGULAR_ERROR::LOCAL_ANGULAR_ERROR;

class EKFFusionNode {
 public:
  EKFFusionNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : viewer_(nh) {
    std::string topic_vo = "/odom_vo";
    std::string topic_imu = "/imu0";

    nh.getParam("topic_vo", topic_vo);
    nh.getParam("topic_imu", topic_imu);

    std::cout << "topic_vo: " << topic_vo << std::endl;
    std::cout << "topic_imu: " << topic_imu << std::endl;

    double acc_n, gyr_n, acc_w, gyr_w, sigma_pv, sigma_rp, sigma_yaw;
    nh.param("acc_noise", acc_n, 1e-2);
    nh.param("gyr_noise", gyr_n, 1e-4);
    nh.param("acc_bias_noise", acc_w, 1e-6);
    nh.param("gyr_bias_noise", gyr_w, 1e-8);

    nh.param("init_sigma_pv", sigma_pv, 0.01);
    nh.param("init_sigma_rp", sigma_rp, 0.01);
    nh.param("init_sigma_yaw", sigma_yaw, 5.0);

    sigma_rp *= kDegreeToRadian;
    sigma_yaw *= kDegreeToRadian;

    ekf_ptr_ = std::make_unique<EKF>(acc_n, gyr_n, acc_w, gyr_w);
    ekf_ptr_->state_ptr_->set_cov(sigma_pv, sigma_pv, sigma_rp, sigma_yaw, acc_w, gyr_w);

    // imu_sub_ = nh.subscribe<sensor_msgs::Imu>(topic_imu, 10, boost::bind(&EKF::imu_callback, ekf_ptr_.get(), _1));
    imu_sub_ = nh.subscribe(topic_imu, 10, &EKFFusionNode::imu_callback, this);
    vo_sub_ = nh.subscribe(topic_vo, 10, &EKFFusionNode::vo_callback, this);

    Tcb = getTransformEigen(pnh, "cam0/T_cam_imu");
  }

  ~EKFFusionNode() {}

  void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
    ImuDataPtr imu_data_ptr = std::make_shared<ImuData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
    imu_data_ptr->acc[1] = imu_msg->linear_acceleration.y;
    imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
    imu_data_ptr->gyr[0] = imu_msg->angular_velocity.x;
    imu_data_ptr->gyr[1] = imu_msg->angular_velocity.y;
    imu_data_ptr->gyr[2] = imu_msg->angular_velocity.z;

    if (!ekf_ptr_->imu_model_.push_data(imu_data_ptr, initialized_)) return;

    ekf_ptr_->predict(last_imu_ptr_, imu_data_ptr);

    last_imu_ptr_ = imu_data_ptr;
  }

  void vo_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &vo_msg);

  Eigen::Matrix<double, kMeasDim, kStateDim> measurementH(const Eigen::Quaterniond &vo_q, const Eigen::Isometry3d &T);

  void check_jacobian(const Eigen::Quaterniond &vo_q, const Eigen::Isometry3d &T);

 private:
  bool initialized_ = false;

  ImuDataConstPtr last_imu_ptr_;

  ros::Subscriber imu_sub_;
  ros::Subscriber vo_sub_;

  Eigen::Isometry3d Tcb;
  Eigen::Isometry3d Tvw;
  Eigen::Isometry3d TvoB;  // for publish

  EKFPtr ekf_ptr_;
  Viewer viewer_;
};

/**
 * @brief h(x)/delta X
 *
 * @param vo_q
 * @param T
 * @return
 */
Eigen::Matrix<double, kMeasDim, kStateDim> EKFFusionNode::measurementH(const Eigen::Quaterniond &vo_q,
                                                                       const Eigen::Isometry3d &T) {
  Eigen::Matrix<double, kMeasDim, kStateDim> H;
  H.setZero();

  const Eigen::Matrix3d &Rvw = Tvw.linear();

  Eigen::Quaterniond q_vw(Rvw);
  Eigen::Quaterniond q_cb(Tcb.linear());
  Eigen::Quaterniond q(ekf_ptr_->state_ptr_->Rwb_);

  switch (ekf_ptr_->kJacobMeasurement_) {
    case JACOBIAN_MEASUREMENT::HX_X: {
      H.block<3, 3>(0, 0) = Rvw;
      if (State::kAngError == ANGULAR_ERROR::LOCAL_ANGULAR_ERROR) {
        H.block<3, 3>(0, 6) = -Rvw * T.linear() * skew_matrix(Tcb.inverse().translation());
        H.block<3, 3>(3, 6) =
            (quat_left_matrix((q_vw * q).normalized()) * quat_right_matrix(q_cb.conjugate())).block<3, 3>(1, 1);
      } else {
        H.block<3, 3>(0, 6) = -Rvw * skew_matrix(T.linear() * Tcb.inverse().translation());
        H.block<3, 3>(3, 6) =
            (quat_left_matrix(q_vw) * quat_right_matrix((q * q_cb.conjugate()).normalized())).block<3, 3>(1, 1);
      }
    } break;
    case JACOBIAN_MEASUREMENT::NEGATIVE_RX_X: {
      Eigen::Matrix4d m4;
      H.block<3, 3>(0, 0) = -Rvw;
      if (State::kAngError == ANGULAR_ERROR::LOCAL_ANGULAR_ERROR) {
        H.block<3, 3>(0, 6) = Rvw * T.linear() * skew_matrix(Tcb.inverse().translation());
        m4 = quat_left_matrix((vo_q.conjugate() * q_vw * q).normalized()) * quat_right_matrix(q_cb.conjugate());
        H.block<3, 3>(3, 6) = -m4.block<3, 3>(1, 1);
      } else {
        H.block<3, 3>(0, 6) = -Rvw * skew_matrix(T.linear() * Tcb.inverse().translation());
        m4 = quat_left_matrix(q_vw) * quat_right_matrix((q * (vo_q * q_cb).conjugate()).normalized());
        H.block<3, 3>(3, 6) = -m4.block<3, 3>(1, 1);
      }
      H *= -1.0;
    } break;
  }

  return H;
}

void EKFFusionNode::vo_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &vo_msg) {
  Eigen::Vector3d vo_p;
  Eigen::Quaterniond vo_q;
  vo_p.x() = vo_msg->pose.pose.position.x;
  vo_p.y() = vo_msg->pose.pose.position.y;
  vo_p.z() = vo_msg->pose.pose.position.z;
  vo_q.x() = vo_msg->pose.pose.orientation.x;
  vo_q.y() = vo_msg->pose.pose.orientation.y;
  vo_q.z() = vo_msg->pose.pose.orientation.z;
  vo_q.w() = vo_msg->pose.pose.orientation.w;

  Eigen::Isometry3d Tvo;  // VO in frame V --> Tc0cn
  Tvo.linear() = vo_q.toRotationMatrix();
  Tvo.translation() = vo_p;

  const Eigen::Matrix<double, kMeasDim, kMeasDim> &R =
      Eigen::Map<const Eigen::Matrix<double, kMeasDim, kMeasDim>>(vo_msg->pose.covariance.data());

  if (!initialized_) {
    if (!(initialized_ = ekf_ptr_->imu_model_.init(*ekf_ptr_->state_ptr_, vo_msg->header.stamp.toSec(), last_imu_ptr_)))
      return;

    Eigen::Isometry3d Tb0bm;
    Tb0bm.linear() = ekf_ptr_->state_ptr_->Rwb_;
    Tb0bm.translation().setZero();

    const Eigen::Isometry3d &Tc0cm = Tvo;

    Tvw = Tc0cm * Tcb * Tb0bm.inverse();  // c0 --> visual frame V, b0 --> world frame W

    printf("[cggos %s] System initialized.\n", __FUNCTION__);

    return;
  }

  // IEKF iteration update, same with EKF when n_ite = 1
  int n_ite = 10;
  Eigen::Matrix<double, kMeasDim, kStateDim> H_i;
  Eigen::Matrix<double, kStateDim, kMeasDim> K_i;
  for (int i = 0; i < n_ite; i++) {
    if (i == 0) *ekf_ptr_->state_ptr_i_ = *ekf_ptr_->state_ptr_;

    // x_i
    const Eigen::Isometry3d &Twb_i = ekf_ptr_->state_ptr_i_->pose();

    // measurement estimation h(x_i), Twb in frame V --> Tc0cn
    const Eigen::Isometry3d &Twb_in_V = Tvw * Twb_i * Tcb.inverse();

    // measurement jacobian H
    H_i = measurementH(vo_q, Twb_i);

    // for debug
    check_jacobian(vo_q, Twb_i);

    // residual = z - h(x_i)
    Eigen::Matrix<double, kMeasDim, 1> residual;
    residual.topRows(3) = Tvo.translation() - Twb_in_V.translation();
    residual.bottomRows(3) = State::rotation_residual(Tvo.linear(), Twb_in_V.linear());

    // residual -= H (x_prior - x_i)
    Eigen::Matrix<double, kStateDim, 1> delta_x = *ekf_ptr_->state_ptr_ - *ekf_ptr_->state_ptr_i_;
    residual -= H_i * delta_x;

    std::cout << "res: " << residual.transpose() << std::endl;

    ekf_ptr_->update_K(H_i, R, K_i);
    *ekf_ptr_->state_ptr_i_ = *ekf_ptr_->state_ptr_ + K_i * residual;
  }

  // update state and cov
  *ekf_ptr_->state_ptr_ = *ekf_ptr_->state_ptr_i_;
  ekf_ptr_->update_P(H_i, R, K_i);

  std::cout << "acc bias: " << ekf_ptr_->state_ptr_->acc_bias.transpose() << std::endl;
  std::cout << "gyr bias: " << ekf_ptr_->state_ptr_->gyr_bias.transpose() << std::endl;

  // view
  // for publish, Tvo in frame W --> Tb0bn
  TvoB = Tvw.inverse() * Tvo * Tcb;
  viewer_.publish_vo(*ekf_ptr_->state_ptr_, TvoB);
}

void EKFFusionNode::check_jacobian(const Eigen::Quaterniond &vo_q, const Eigen::Isometry3d &Twb) {
  Eigen::Vector3d delta(0.0012, -0.00034, -0.00056);

  // perturbation on t
  Eigen::Isometry3d T1 = Twb;
  T1.translation() += delta;

  // perturbation on R
  Eigen::Isometry3d T2 = Twb;
  T2.linear() = State::rotation_update(T2.linear(), State::delta_rot_mat(delta, 1));

  Eigen::Isometry3d Tx0 = Tvw * Twb * Tcb.inverse();
  Eigen::Isometry3d Tx1 = Tvw * T1 * Tcb.inverse();
  Eigen::Isometry3d Tx2 = Tvw * T2 * Tcb.inverse();

  auto H = measurementH(vo_q, Twb);

  std::cout << "---------------------" << std::endl;
  std::cout << "(purt t) p res: " << (Tx1.translation() - Tx0.translation()).transpose() << std::endl;
  std::cout << "(purt t) p Hx: " << (H.block<3, 3>(0, 0) * delta).transpose() << std::endl;

  std::cout << "(purt R) p res: " << (Tx2.translation() - Tx0.translation()).transpose() << std::endl;
  std::cout << "(purt R) p Hx: " << (H.block<3, 3>(0, 6) * delta).transpose() << std::endl;

  std::cout << "(purt R) q res: " << State::rotation_residual(Tx2.linear(), Tx0.linear()).transpose() << std::endl;
  std::cout << "(purt R) q Hx: " << (H.block<3, 3>(3, 6) * delta).transpose() << std::endl;
  std::cout << "---------------------" << std::endl;
}

}  // namespace cg

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_vo_fusion");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  cg::EKFFusionNode fusion_node(nh, pnh);

  ros::spin();

  return 0;
}
