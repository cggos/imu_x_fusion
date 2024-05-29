#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <iostream>

#include "common/view.hpp"
#include "estimator/ukf.hpp"
#include "sensor/imu.hpp"

namespace cg {

ANGULAR_ERROR State::kAngError = ANGULAR_ERROR::LOCAL_ANGULAR_ERROR;

class UKFFusionNode {
 public:
  UKFFusionNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : viewer_(nh) {
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

    ukf_ptr_ = std::make_unique<UKF>(sigma_pv, sigma_pv, sigma_rp, sigma_yaw, acc_w, gyr_w);
    ukf_ptr_->imu_model_ = std::make_shared<IMU>(ukf_ptr_->state_ptr_, acc_n, gyr_n, acc_w, gyr_w);
    ukf_ptr_->predictor_ptr_ = std::dynamic_pointer_cast<IMU>(ukf_ptr_->imu_model_);
    // ukf_ptr_->observer_ptr_ = std::make_shared<Odom6Dof>();

    imu_sub_ = nh.subscribe<sensor_msgs::Imu>(topic_imu, 10, boost::bind(&UKFFusionNode::imu_callback, this, _1));
    vo_sub_ = nh.subscribe(topic_vo, 10, &UKFFusionNode::vo_callback, this);

    Tcb = Utils::getTransformEigen(pnh, "cam0/T_cam_imu");
  }

  ~UKFFusionNode() {}

  void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
    Eigen::Vector3d acc, gyr;
    acc[0] = imu_msg->linear_acceleration.x;
    acc[1] = imu_msg->linear_acceleration.y;
    acc[2] = imu_msg->linear_acceleration.z;
    gyr[0] = imu_msg->angular_velocity.x;
    gyr[1] = imu_msg->angular_velocity.y;
    gyr[2] = imu_msg->angular_velocity.z;

    ukf_ptr_->predict(std::make_shared<ImuData>(imu_msg->header.stamp.toSec(), acc, gyr));
  }

  void vo_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &vo_msg);

 private:
  ros::Subscriber imu_sub_;
  ros::Subscriber vo_sub_;

  Eigen::Isometry3d Tcb;
  Eigen::Isometry3d Tvw;

  UKFPtr ukf_ptr_;
  Viewer viewer_;
};

void UKFFusionNode::vo_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &vo_msg) {
  Eigen::Isometry3d Tvo;  // VO in frame V --> Tc0cn
  tf::poseMsgToEigen(vo_msg->pose.pose, Tvo);

  ukf_ptr_->measurement_noise_cov_ =
      Eigen::Map<const Eigen::Matrix<double, kMeasDim, kMeasDim>>(vo_msg->pose.covariance.data());

  if (!ukf_ptr_->predictor_ptr_->inited_) {
    if (!ukf_ptr_->predictor_ptr_->init(vo_msg->header.stamp.toSec())) return;

    Eigen::Isometry3d Tb0bm;
    Tb0bm.linear() = ukf_ptr_->state_ptr_->Rwb_;
    Tb0bm.translation().setZero();

    const Eigen::Isometry3d &Tc0cm = Tvo;

    Tvw = Tc0cm * Tcb * Tb0bm.inverse();  // c0 --> visual frame V, b0 --> world frame W

    printf("[cggos %s] System initialized.\n", __FUNCTION__);

    return;
  }

  ukf_ptr_->update(Tvw, Tcb, Tvo);

  std::cout << "---------------------" << std::endl;
  std::cout << "acc bias: " << ukf_ptr_->state_ptr_->acc_bias.transpose() << std::endl;
  std::cout << "gyr bias: " << ukf_ptr_->state_ptr_->gyr_bias.transpose() << std::endl;
  std::cout << "---------------------" << std::endl;

  // view
  // for publish, Tvo in frame W --> Tb0bn
  Eigen::Isometry3d TvoB = Tvw.inverse() * Tvo * Tcb;
  viewer_.publish_vo(*ukf_ptr_->state_ptr_, TvoB);
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