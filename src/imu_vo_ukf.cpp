#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <iostream>

#include "imu_x_fusion/ukf.hpp"

namespace cg {

constexpr int kMeasDim = 6;

class UKFFusionNode {
 public:
  UKFFusionNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
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

    ukf_ptr_ = std::make_unique<UKF>(acc_n, gyr_n, acc_w, gyr_w);
    ukf_ptr_->set_cov(sigma_pv, sigma_pv, sigma_rp, sigma_yaw, acc_w, gyr_w);

    imu_sub_ = nh.subscribe<sensor_msgs::Imu>(topic_imu, 10, boost::bind(&UKF::imu_callback, ukf_ptr_.get(), _1));
    vo_sub_ = nh.subscribe(topic_vo, 10, &UKFFusionNode::vo_callback, this);

    path_pub_ = nh.advertise<nav_msgs::Path>("path_est", 10);
    path_pub_vo_ = nh.advertise<nav_msgs::Path>("path_vo", 10);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_est", 10);

    Tcb = getTransformEigen(pnh, "cam0/T_cam_imu");
  }

  ~UKFFusionNode() {}

  void vo_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &vo_msg);

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

  UKFPtr ukf_ptr_;
};

void UKFFusionNode::vo_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &vo_msg) {
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

  if (!ukf_ptr_->initialized_) {
    if (ukf_ptr_->imu_buf_.size() < ukf_ptr_->kImuBufSize) {
      printf("[cggos %s] ERROR: Not Enough IMU data for Initialization!!!\n", __FUNCTION__);
      return;
    }

    ukf_ptr_->last_imu_ptr_ = ukf_ptr_->imu_buf_.back();
    if (std::abs(vo_msg->header.stamp.toSec() - ukf_ptr_->last_imu_ptr_->timestamp) > 0.05) {
      printf("[cggos %s] ERROR: timestamps are not synchronized!!!\n", __FUNCTION__);
      return;
    }

    ukf_ptr_->state_ptr_->timestamp = ukf_ptr_->last_imu_ptr_->timestamp;

    if (!ukf_ptr_->init_rot_from_imudata(ukf_ptr_->state_ptr_->r_GI, ukf_ptr_->imu_buf_)) return;

    ukf_ptr_->predicted_x_ = ukf_ptr_->state_ptr_->vec();
    ukf_ptr_->predicted_P_ = ukf_ptr_->state_ptr_->cov;

    Eigen::Isometry3d Tb0bm;
    Tb0bm.linear() = ukf_ptr_->state_ptr_->r_GI;
    Tb0bm.translation().setZero();

    const Eigen::Isometry3d &Tc0cm = Tvo;

    Tvw = Tc0cm * Tcb * Tb0bm.inverse();  // c0 --> visual frame V, b0 --> world frame W

    printf("[cggos %s] System initialized.\n", __FUNCTION__);

    return;
  }

  // for publish, Tvo in frame W --> Tb0bn
  TvoB = Tvw.inverse() * Tvo * Tcb;

  // predict measurement sigma points
  Eigen::Isometry3d Twb;
  Eigen::MatrixXd predicted_meas_sp_mat = Eigen::MatrixXd::Zero(kMeasDim, kSigmaPointsNum);
  for (int i = 0; i < kSigmaPointsNum; i++) {
    const auto &sp = ukf_ptr_->predicted_sp_mat_.col(i);

    Twb.translation() = sp.segment<3>(0);
    Twb.linear() = rot_vec_to_mat(sp.segment<3>(6));

    // measurement estimation h(x), Twb in frame V --> Tc0cn
    const Eigen::Isometry3d &Twb_in_V = Tvw * Twb * Tcb.inverse();

    predicted_meas_sp_mat.col(i).head(3) = Twb_in_V.translation();
    predicted_meas_sp_mat.col(i).bottomRows(3) = rot_mat_to_vec(Twb_in_V.linear());
  }

  // predict measurement mean
  Eigen::VectorXd predicted_z = Eigen::VectorXd::Zero(kMeasDim);
  for (int c = 0; c < kSigmaPointsNum; c++) {
    predicted_z += ukf_ptr_->weights_[c] * predicted_meas_sp_mat.col(c);
  }

  // predict measurement covariance
  Eigen::MatrixXd predicted_S = Eigen::MatrixXd::Zero(kMeasDim, kMeasDim);
  Eigen::VectorXd dz = Eigen::VectorXd(kMeasDim);
  for (int c = 0; c < kSigmaPointsNum; c++) {
    dz = predicted_meas_sp_mat.col(c) - predicted_z;
    predicted_S += ukf_ptr_->weights_[c] * dz * dz.transpose();
  }
  predicted_S += R;

  // compute Tc
  Eigen::VectorXd dx;
  Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(kStateDim, kMeasDim);
  for (int c = 0; c < kSigmaPointsNum; c++) {
    dx = ukf_ptr_->predicted_sp_mat_.col(c) - ukf_ptr_->predicted_x_;
    dz = predicted_meas_sp_mat.col(c) - predicted_z;
    Tc += ukf_ptr_->weights_[c] * dx * dz.transpose();
  }

  // update
  Eigen::Matrix<double, kMeasDim, 1> vec_vo;
  vec_vo.segment<3>(0) = Tvo.translation();
  vec_vo.segment<3>(0) = rot_mat_to_vec(Tvo.linear());
  dz = vec_vo - predicted_z;

  std::cout << "res: " << dz.transpose() << std::endl;

  Eigen::MatrixXd K = Tc * predicted_S.inverse();

  ukf_ptr_->state_ptr_->from_vec(ukf_ptr_->predicted_x_);
  *ukf_ptr_->state_ptr_ = *ukf_ptr_->state_ptr_ + K * dz;
  ukf_ptr_->state_ptr_->cov = ukf_ptr_->predicted_P_ - K * predicted_S * K.transpose();

  std::cout << "acc bias: " << ukf_ptr_->state_ptr_->acc_bias.transpose() << std::endl;
  std::cout << "gyr bias: " << ukf_ptr_->state_ptr_->gyr_bias.transpose() << std::endl;

  publish();
}

void UKFFusionNode::publish() {
  // publish the odometry
  std::string fixed_id = "global";
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = fixed_id;
  odom_msg.child_frame_id = "odom";

  Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
  T_wb.linear() = ukf_ptr_->state_ptr_->r_GI;
  T_wb.translation() = ukf_ptr_->state_ptr_->p_GI;
  tf::poseEigenToMsg(T_wb, odom_msg.pose.pose);
  tf::vectorEigenToMsg(ukf_ptr_->state_ptr_->v_GI, odom_msg.twist.twist.linear);
  const Eigen::Matrix3d &P_pp = ukf_ptr_->state_ptr_->cov.block<3, 3>(0, 0);
  const Eigen::Matrix3d &P_po = ukf_ptr_->state_ptr_->cov.block<3, 3>(0, 6);
  const Eigen::Matrix3d &P_op = ukf_ptr_->state_ptr_->cov.block<3, 3>(6, 0);
  const Eigen::Matrix3d &P_oo = ukf_ptr_->state_ptr_->cov.block<3, 3>(6, 6);
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
  ros::init(argc, argv, "imu_vo_ukf_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  cg::UKFFusionNode fusion_node(nh, pnh);

  ros::spin();

  return 0;
}