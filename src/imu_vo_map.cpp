#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <deque>
#include <iostream>
#include <memory>

#include "common/utils.hpp"
#include "common/view.hpp"
#include "estimator/map.hpp"
#include "estimator/map_cs.hpp"
#include "sensor/imu.hpp"
#include "sensor/odom_6dof.hpp"

// choose one of the four
#define WITH_DIY 1    // User Defined
#define WITH_CS 0     // with Ceres-Solver
#define WITH_G2O 0    // with G2O, TODO
#define WITH_GTSAM 0  // with GTSAM, TODO

#if WITH_GTSAM
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#endif

namespace cg {

ANGULAR_ERROR State::kAngError = ANGULAR_ERROR::LOCAL_ANGULAR_ERROR;

class MAPFusionNode {
 public:
  MAPFusionNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : viewer_(nh) {
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

    map_ptr_ = std::make_shared<MAP>(acc_n, gyr_n, acc_w, gyr_w);
    // map_ptr_->state_ptr_->set_cov(sigma_pv, sigma_pv, sigma_rp, sigma_yaw, acc_w, gyr_w);
    map_ptr_->observer_ptr_ = std::make_shared<Odom6Dof>();

    // init bias
    Eigen::Vector3d acc_bias(-0.0108563, 0.0796346, 0.136003);
    Eigen::Vector3d gyr_bias(0.00224079, 0.0218608, 0.0736346);
    map_ptr_->state_ptr_->set_bias(acc_bias, gyr_bias);

    imu_sub_ = nh.subscribe<sensor_msgs::Imu>(topic_imu, 10, boost::bind(&MAP::imu_callback, map_ptr_.get(), _1));
    vo_sub_ = nh.subscribe(topic_vo, 10, &MAPFusionNode::vo_callback, this);

    Tcb = Utils::getTransformEigen(pnh, "cam0/T_cam_imu");
  }

  ~MAPFusionNode() {}

  void vo_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &vo_msg);

 private:
  ros::Subscriber imu_sub_;
  ros::Subscriber vo_sub_;

  Eigen::Isometry3d Tcb;
  Eigen::Isometry3d Tvw;

  MAPPtr map_ptr_;
  Viewer viewer_;
};

void MAPFusionNode::vo_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &vo_msg) {
  Eigen::Isometry3d Tvo;  // VO in frame V --> Tc0cn
  tf::poseMsgToEigen(vo_msg->pose.pose, Tvo);

  const Eigen::Matrix<double, kMeasDim, kMeasDim> &R =
      Eigen::Map<const Eigen::Matrix<double, kMeasDim, kMeasDim>>(vo_msg->pose.covariance.data());

  if (!map_ptr_->inited_) {
    if (!map_ptr_->init(vo_msg->header.stamp.toSec())) return;

    Eigen::Isometry3d Tb0bm;
    Tb0bm.linear() = map_ptr_->state_ptr_->Rwb_;
    Tb0bm.translation().setZero();

    const Eigen::Isometry3d &Tc0cm = Tvo;

    Tvw = Tc0cm * Tcb * Tb0bm.inverse();  // c0 --> visual frame V, b0 --> world frame W

    std::dynamic_pointer_cast<Odom6Dof>(map_ptr_->observer_ptr_)->set_params(Tvw, Tcb);

    printf("[cggos %s] System initialized.\n", __FUNCTION__);

    return;
  }

  State state_est;
#if WITH_DIY
  // G-N iteration update, same with EKF when n_ite = 1
  int n_ite = 50;
  double lambda = 1.0;
  double cost = 0, last_cost = 0;
  Eigen::Matrix<double, kStateDim, 1> dx;
  Eigen::Matrix<double, kStateDim, 1> b;
  Eigen::Matrix<double, kMeasDim, kStateDim> J;
  Eigen::Matrix<double, kStateDim, kStateDim> H;
  const auto &InfoMat = Eigen::Matrix<double, kMeasDim, kMeasDim>::Identity();  // R.inverse();
  for (int i = 0; i < n_ite; i++) {
    b = Eigen::Matrix<double, kStateDim, 1>::Zero();
    H = Eigen::Matrix<double, kStateDim, kStateDim>::Zero();

    if (i == 0) state_est = *map_ptr_->state_ptr_;

    const Eigen::Isometry3d &Twb_i = state_est.pose();  // x_i

    J = map_ptr_->observer_ptr_->measurement_jacobian(Twb_i.matrix(), Tvo.matrix());

    map_ptr_->observer_ptr_->check_jacobian(Twb_i.matrix(), Tvo.matrix());  // for debug

    auto residual = map_ptr_->observer_ptr_->measurement_residual(Twb_i.matrix(), Tvo.matrix());

    std::cout << "res: " << residual.transpose() << std::endl;

    cost = residual.squaredNorm();

    std::cout << "iteration " << i << " cost: " << std::cout.precision(12) << cost << ", last cost: " << last_cost
              << std::endl;

    if (i > 0 && cost >= last_cost) {  // cost increase, update is not good
      lambda *= 10.0f;
    } else {
      lambda /= 10.0f;
    }

    H.noalias() += J.transpose() * InfoMat * J + Eigen::Matrix<double, kStateDim, kStateDim>::Identity() * lambda;
    b.noalias() += J.transpose() * InfoMat * residual;

    double cond_num = Utils::condition_number(H);
    std::cout << "cond num of H: " << cond_num << std::endl;
    // if (cond_num > 1e5) H = H.diagonal().asDiagonal();

    dx = H.ldlt().solve(b);

    state_est = state_est + dx;

    last_cost = cost;
  }
#endif

#if WITH_CS
  Eigen::Matrix<double, kStateDim, 1> state_vec = map_ptr_->state_ptr_->vec();

  Eigen::Matrix<double, 3, 1> vec_p = state_vec.segment<3>(0);
  Eigen::Matrix<double, 3, 1> vec_R = state_vec.segment<3>(6);
  {
    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);  // ceres::CauchyLoss(1.0)
    ceres::CostFunction *cost_function = new MAPCostFunctor(map_ptr_, Tcb, Tvw, Tvo, R);
    // ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    // problem.AddParameterBlock(vec_pose, 7, local_parameterization);
    problem.AddResidualBlock(cost_function, loss_function, vec_p.data(), vec_R.data());

    ceres::Solver::Options options;
    options.dynamic_sparsity = true;
    options.max_num_iterations = 100;
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    options.minimizer_type = ceres::TRUST_REGION;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
  }
  state_vec.segment<3>(0) = vec_p;
  state_vec.segment<3>(6) = vec_R;

  state_est.from_vec(state_vec);
#endif

#if WITH_G2O
#endif

#if WITH_GTSAM
  gtsam::NonlinearFactorGraph::shared_ptr graph(new gtsam::NonlinearFactorGraph);

  Eigen::Quaterniond q_wb(map_ptr_->state_ptr_->Rwb_);
  gtsam::Rot3 Rwb = gtsam::Rot3::Quaternion(q_wb.w(), q_wb.x(), q_wb.y(), q_wb.z());
  gtsam::Point3 twb = map_ptr_->state_ptr_->p_wb_;

  gtsam::Key id = 0;
  gtsam::Values::shared_ptr initial(new gtsam::Values);
  initial->insert(id, gtsam::Pose3(Rwb, twb));
#endif

  // // update state and cov
  // ekf_ptr_->update_P(H_i, R, K_i);
  *map_ptr_->state_ptr_ = state_est;

  std::cout << "acc bias: " << map_ptr_->state_ptr_->acc_bias.transpose() << std::endl;
  std::cout << "gyr bias: " << map_ptr_->state_ptr_->gyr_bias.transpose() << std::endl;

  // view
  // for publish, Tvo in frame W --> Tb0bn
  Eigen::Isometry3d TvoB = Tvw.inverse() * Tvo * Tcb;
  viewer_.publish_vo(*map_ptr_->state_ptr_, TvoB);
}

}  // namespace cg

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_vo_fusion");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  cg::MAPFusionNode fusion_node(nh, pnh);

  ros::spin();

  return 0;
}
