#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <deque>
#include <iostream>

#include "imu_x_fusion/kf.hpp"

namespace cg {

constexpr int kMeasDim = 6;

class FusionNode {
 public:
  FusionNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
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

    kf_ptr_ = std::make_unique<KF>(acc_n, gyr_n, acc_w, gyr_w);
    kf_ptr_->set_cov(sigma_pv, sigma_pv, sigma_rp, sigma_yaw, acc_w, gyr_w);

    imu_sub_ = nh.subscribe<sensor_msgs::Imu>(topic_imu, 10, boost::bind(&KF::imu_callback, kf_ptr_.get(), _1));
    vo_sub_ = nh.subscribe(topic_vo, 10, &FusionNode::vo_callback, this);

    path_pub_ = nh.advertise<nav_msgs::Path>("path_est", 10);
    path_pub_vo_ = nh.advertise<nav_msgs::Path>("path_vo", 10);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_est", 10);

    Tcb = getTransformEigen(pnh, "cam0/T_cam_imu");
  }

  ~FusionNode() {}

  void vo_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &vo_msg);

  Eigen::Matrix<double, kMeasDim, kStateDim> measurementH(const Eigen::Quaterniond &vo_q, const Eigen::Isometry3d &T);

  Eigen::Vector3d measurement_res(const Eigen::Isometry3d &Tobs, const Eigen::Isometry3d &Test);

  void check_jacobian(const Eigen::Quaterniond &vo_q, const Eigen::Isometry3d &T);

  void publish();

 private:
  ros::Subscriber imu_sub_;
  ros::Subscriber vo_sub_;

  ros::Publisher odom_pub_;
  ros::Publisher path_pub_;
  ros::Publisher path_pub_vo_;

  nav_msgs::Path nav_path_;
  nav_msgs::Path nav_path_vo_;

  Eigen::Isometry3d Tcb;
  Eigen::Isometry3d Tvw;
  Eigen::Isometry3d TvoB;  // for publish

  KFPtr kf_ptr_;
};

Eigen::Vector3d FusionNode::measurement_res(const Eigen::Isometry3d &Tobs, const Eigen::Isometry3d &Test) {
  Eigen::Quaterniond q_res;
  switch (kf_ptr_->kRotPerturbation_) {
    case ANGULAR_ERROR::LOCAL_ANGULAR_ERROR:
      q_res = Eigen::Quaterniond(Test.linear().transpose() * Tobs.linear());
      break;
    case ANGULAR_ERROR::GLOBAL_ANGULAR_ERROR:
      q_res = Eigen::Quaterniond(Tobs.linear() * Test.linear().transpose());
      break;
  }
  return 2.0 * q_res.vec() / q_res.w();
}

/**
 * @brief h(x)/delta X
 *
 * @param vo_q
 * @param T
 * @return
 */
Eigen::Matrix<double, kMeasDim, kStateDim> FusionNode::measurementH(const Eigen::Quaterniond &vo_q,
                                                                    const Eigen::Isometry3d &T) {
  Eigen::Matrix<double, kMeasDim, kStateDim> H;
  H.setZero();

  const Eigen::Matrix3d &Rvw = Tvw.linear();

  Eigen::Quaterniond q_vw(Rvw);
  Eigen::Quaterniond q_cb(Tcb.linear());
  Eigen::Quaterniond q(kf_ptr_->state_ptr_->r_GI);

  switch (kf_ptr_->kJacobMeasurement_) {
    case JACOBIAN_MEASUREMENT::HX_X: {
      H.block<3, 3>(0, 0) = Rvw;

      // LOCAL_PERTURBATION
      H.block<3, 3>(0, 6) = -Rvw * T.linear() * skew_matrix(Tcb.inverse().translation());

      Eigen::Matrix4d m4;
      switch (kf_ptr_->kRotPerturbation_) {
        case ROTATION_PERTURBATION::LOCAL_PERTURBATION:
          m4 = quat_left_matrix((q_vw * q).normalized()) * quat_right_matrix(q_cb.conjugate());
          break;
        case ROTATION_PERTURBATION::GLOBAL_PERTURBATION:
          m4 = quat_left_matrix(q_vw) * quat_right_matrix((q * q_cb.conjugate()).normalized());
          break;
      }
      H.block<3, 3>(3, 6) = m4.block<3, 3>(1, 1);
    } break;
    case JACOBIAN_MEASUREMENT::NEGATIVE_RX_X: {
      H.block<3, 3>(0, 0) = -Rvw;

      // LOCAL_PERTURBATION
      H.block<3, 3>(0, 6) = Rvw * T.linear() * skew_matrix(Tcb.inverse().translation());

      Eigen::Matrix4d m4;
      if (kf_ptr_->kAngError_ == ANGULAR_ERROR::LOCAL_ANGULAR_ERROR) {
        // LOCAL_PERTURBATION
        m4 = quat_left_matrix((vo_q.conjugate() * q_vw * q).normalized()) * quat_right_matrix(q_cb.conjugate());
      } else {
        if (kf_ptr_->kRotPerturbation_ == ROTATION_PERTURBATION::LOCAL_PERTURBATION) {
          m4 = quat_left_matrix((q_vw * q).normalized()) * quat_right_matrix((vo_q * q_cb).conjugate().normalized());
        } else {
          m4 = quat_left_matrix(q_vw) * quat_right_matrix((q * (vo_q * q_cb).conjugate()).normalized());
        }
      }
      H.block<3, 3>(3, 6) = -m4.block<3, 3>(1, 1);

      H *= -1.0;
    } break;
  }

  return H;
}

void FusionNode::vo_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &vo_msg) {
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

  if (!kf_ptr_->initialized_) {
    if (kf_ptr_->imu_buf_.size() < kf_ptr_->kImuBufSize) {
      printf("[cggos %s] ERROR: Not Enough IMU data for Initialization!!!\n", __FUNCTION__);
      return;
    }

    kf_ptr_->last_imu_ptr_ = kf_ptr_->imu_buf_.back();
    if (std::abs(vo_msg->header.stamp.toSec() - kf_ptr_->last_imu_ptr_->timestamp) > 0.05) {
      printf("[cggos %s] ERROR: timestamps are not synchronized!!!\n", __FUNCTION__);
      return;
    }

    kf_ptr_->state_ptr_->timestamp = kf_ptr_->last_imu_ptr_->timestamp;

    if (!kf_ptr_->init_rot_from_imudata(kf_ptr_->state_ptr_->r_GI, kf_ptr_->imu_buf_)) return;

    Eigen::Isometry3d Tb0bm;
    Tb0bm.linear() = kf_ptr_->state_ptr_->r_GI;
    Tb0bm.translation().setZero();

    const Eigen::Isometry3d &Tc0cm = Tvo;

    Tvw = Tc0cm * Tcb * Tb0bm.inverse();  // c0 --> visual frame V, b0 --> world frame W

    printf("[cggos %s] System initialized.\n", __FUNCTION__);

    return;
  }

  // for publish, Tvo in frame W --> Tb0bn
  TvoB = Tvw.inverse() * Tvo * Tcb;

  // IEKF iteration update
  int n_ite = 10;
  for (int i = 0; i < n_ite; i++) {
    if (i == 0) kf_ptr_->state_ptr_->update_iekf();

    Eigen::Isometry3d Twb_i;
    Twb_i.linear() = kf_ptr_->state_ptr_->r_GI_i;
    Twb_i.translation() = kf_ptr_->state_ptr_->p_GI_i;

    // measurement estimation h(x_i), Twb in frame V --> Tc0cn
    Eigen::Isometry3d Twb_in_V = Tvw * Twb_i * Tcb.inverse();

    // measurement jacobian
    const auto &H_i = measurementH(vo_q, Twb_i);

    // residual = z - h(x_i)
    Eigen::Matrix<double, kMeasDim, 1> residual;
    residual.topRows(3) = Tvo.translation() - Twb_in_V.translation();
    residual.bottomRows(3) = measurement_res(Tvo, Twb_in_V);

    // residual -= H (x_prior - x_i)
    Eigen::Matrix<double, kStateDim, 1> delta_x;
    delta_x.block<3, 1>(0, 0) = kf_ptr_->state_ptr_->p_GI - kf_ptr_->state_ptr_->p_GI_i;
    delta_x.block<3, 1>(3, 0) = kf_ptr_->state_ptr_->v_GI - kf_ptr_->state_ptr_->v_GI_i;
    Eigen::Quaterniond q_res = Eigen::Quaterniond(kf_ptr_->state_ptr_->r_GI_i.transpose() * kf_ptr_->state_ptr_->r_GI);
    delta_x.block<3, 1>(6, 0) = 2.0 * q_res.vec() / q_res.w();
    delta_x.block<3, 1>(9, 0) = kf_ptr_->state_ptr_->acc_bias - kf_ptr_->state_ptr_->acc_bias_i;
    delta_x.block<3, 1>(12, 0) = kf_ptr_->state_ptr_->gyr_bias - kf_ptr_->state_ptr_->gyr_bias_i;
    residual -= H_i * delta_x;

    std::cout << "res: " << residual.transpose() << std::endl;

    check_jacobian(vo_q, Twb_i);

    kf_ptr_->update_measurement(H_i, R, residual, i, n_ite, kf_ptr_->kAngError_);
  }

  std::cout << "acc bias: " << kf_ptr_->state_ptr_->acc_bias.transpose() << std::endl;
  std::cout << "gyr bias: " << kf_ptr_->state_ptr_->gyr_bias.transpose() << std::endl;

  publish();
}

void FusionNode::check_jacobian(const Eigen::Quaterniond &vo_q, const Eigen::Isometry3d &Twb) {
  Eigen::Vector3d delta(0.0012, -0.00034, -0.00056);

  // perturbation on t
  Eigen::Isometry3d T1 = Twb;
  T1.translation() += delta;

  // perturbation on R
  Eigen::Quaterniond delta_q;
  delta_q.w() = 1;
  delta_q.vec() = 0.5 * delta;
  Eigen::Isometry3d T2 = Twb;
  switch (kf_ptr_->kAngError_) {
    case ANGULAR_ERROR::LOCAL_ANGULAR_ERROR:
      T2.linear() *= delta_q.toRotationMatrix();
      break;
    case ANGULAR_ERROR::GLOBAL_ANGULAR_ERROR:
      T2.linear() = delta_q.toRotationMatrix() * T2.linear();
      break;
  }

  Eigen::Isometry3d TvoN1 = Tvw * T1 * Tcb.inverse();
  Eigen::Isometry3d TvoN2 = Tvw * T2 * Tcb.inverse();
  Eigen::Isometry3d Twb_in_V = Tvw * Twb * Tcb.inverse();

  auto H1 = measurementH(vo_q, T1);
  auto H2 = measurementH(vo_q, T2);

  std::cout << "---------------------" << std::endl;
  std::cout << "(purt t) p res: " << (TvoN1.translation() - Twb_in_V.translation()).transpose() << std::endl;
  std::cout << "(purt t) p Hx: " << (H1.block<3, 3>(0, 0) * delta).transpose() << std::endl;

  std::cout << "(purt R) p res: " << (TvoN2.translation() - Twb_in_V.translation()).transpose() << std::endl;
  std::cout << "(purt R) p Hx: " << (H2.block<3, 3>(0, 6) * delta).transpose() << std::endl;

  std::cout << "(purt R) q res: " << measurement_res(TvoN2, Twb_in_V).transpose() << std::endl;
  std::cout << "(purt R) q Hx: " << (H2.block<3, 3>(3, 6) * delta).transpose() << std::endl;
  std::cout << "---------------------" << std::endl;
}

void FusionNode::publish() {
  // publish the odometry
  std::string fixed_id = "global";
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = fixed_id;
  odom_msg.child_frame_id = "odom";

  Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
  T_wb.linear() = kf_ptr_->state_ptr_->r_GI;
  T_wb.translation() = kf_ptr_->state_ptr_->p_GI;
  tf::poseEigenToMsg(T_wb, odom_msg.pose.pose);
  tf::vectorEigenToMsg(kf_ptr_->state_ptr_->v_GI, odom_msg.twist.twist.linear);
  const Eigen::Matrix3d &P_pp = kf_ptr_->state_ptr_->cov.block<3, 3>(0, 0);
  const Eigen::Matrix3d &P_po = kf_ptr_->state_ptr_->cov.block<3, 3>(0, 6);
  const Eigen::Matrix3d &P_op = kf_ptr_->state_ptr_->cov.block<3, 3>(6, 0);
  const Eigen::Matrix3d &P_oo = kf_ptr_->state_ptr_->cov.block<3, 3>(6, 6);
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> P_imu_pose = Eigen::Matrix<double, 6, 6>::Zero();
  P_imu_pose << P_pp, P_po, P_op, P_oo;
  for (int i = 0; i < 36; i++) odom_msg.pose.covariance[i] = P_imu_pose.data()[i];
  odom_pub_.publish(odom_msg);

  // publish the path
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = odom_msg.header;
  pose_stamped.pose = odom_msg.pose.pose;
  nav_path_.header = pose_stamped.header;
  nav_path_.poses.push_back(pose_stamped);
  path_pub_.publish(nav_path_);

  // publish vo path
  geometry_msgs::Pose pose_vo;
  tf::poseEigenToMsg(TvoB, pose_vo);
  geometry_msgs::PoseStamped pose_stamped_vo;
  pose_stamped_vo.header = pose_stamped.header;
  pose_stamped_vo.pose = pose_vo;
  nav_path_vo_.header = pose_stamped_vo.header;
  nav_path_vo_.poses.push_back(pose_stamped_vo);
  path_pub_vo_.publish(nav_path_vo_);
}

}  // namespace cg

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_vo_fusion");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  cg::FusionNode fusion_node(nh, pnh);

  ros::spin();

  return 0;
}
