#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <deque>
#include <iostream>
#include <memory>

#include "common/utils.hpp"
#include "common/view.hpp"
#include "estimator/map.hpp"
#include "sensor/imu.hpp"
#include "sensor/odom_6dof.hpp"

// choose one of the four
#define WITH_DIY 1    // User Defined
#define WITH_CS 0     // with Ceres-Solver
#define WITH_G2O 0    // with G2O
#define WITH_GTSAM 0  // with GTSAM, TODO

#if WITH_DIY
enum OptType { kGN, kLM };
#elif WITH_CS
#include "estimator/map_cs.hpp"
#elif WITH_G2O
#include "estimator/map_g2o.hpp"
#elif WITH_GTSAM
#include "estimator/map_gtsam.hpp"
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
    factor_odom6dof_ptr_ = std::make_shared<Odom6Dof>();

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
  FactorPtr factor_odom6dof_ptr_;

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

    std::dynamic_pointer_cast<Odom6Dof>(factor_odom6dof_ptr_)->set_params(Tvw, Tcb);

    printf("[cggos %s] System initialized.\n", __FUNCTION__);

    return;
  }

  State &state_est = *map_ptr_->state_ptr_;
  auto InfoMat = Eigen::Matrix<double, kMeasDim, kMeasDim>::Identity();  // R.inverse();
#if WITH_DIY
  OptType opt_type = OptType::kLM;

  Eigen::Isometry3d Twb_i = state_est.pose();

  // G-N iteration update, same with EKF when n_ite = 1
  int n_ite = 50;
  double lambda = opt_type == OptType::kLM ? 1.0 : 0.0;
  double cost = 0, last_cost = 0;
  Eigen::Matrix<double, kMeasDim, kStateDim> Jall;
  Eigen::Matrix<double, kMeasDim, 6> J;
  Eigen::Matrix<double, 6, 1> dx;
  Eigen::Matrix<double, 6, 1> b;
  Eigen::Matrix<double, 6, 6> H;
  for (int i = 0; i < n_ite; i++) {
    Jall = -1.0 * factor_odom6dof_ptr_->measurement_jacobian(Twb_i.matrix(), Tvo.matrix());
    J.leftCols(3) = Jall.leftCols(3);
    J.rightCols(3) = Jall.block<6, 3>(0, 6);
    // factor_odom6dof_ptr_->check_jacobian(Twb_i.matrix(), Tvo.matrix());  // for debug

    auto residual = factor_odom6dof_ptr_->measurement_residual(Twb_i.matrix(), Tvo.matrix());
    // std::cout << "res: " << residual.transpose() << std::endl;

    cost = residual.squaredNorm();
    std::cout << "iteration " << i << " cost: " << std::cout.precision(12) << cost << ", last cost: " << last_cost
              << std::endl;
    if (opt_type == OptType::kGN)
      if (i > 0 && cost > last_cost) break;
    if (opt_type == OptType::kLM) {
      if (i > 0 && cost >= last_cost)
        lambda *= 10.0f;
      else
        lambda /= 10.0f;
    }
    last_cost = cost;

    b = Eigen::Matrix<double, 6, 1>::Zero();
    H = Eigen::Matrix<double, 6, 6>::Zero();
    H.noalias() += J.transpose() * InfoMat * J + Eigen::Matrix<double, 6, 6>::Identity() * lambda;
    b.noalias() += -1.0 * J.transpose() * InfoMat * residual;

    double cond_num = Utils::condition_number(H);
    std::cout << "cond num of H: " << cond_num << std::endl;
    // if (cond_num > 1e5) H = H.diagonal().asDiagonal();

    dx = H.ldlt().solve(b);

    State::update_pose(Twb_i, dx);
  }
  state_est.set_pose(Twb_i);
#endif

#if WITH_CS
  auto vec_pq = state_est.vec_pq();
  auto vec_p = vec_pq.head(3);
  auto vec_q = vec_pq.tail(4);
  {
    ceres::Problem problem;
    ceres::LossFunction *loss_function = nullptr;
    // loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    ceres::CostFunction *cost_function = new MAPCostFunctor(factor_odom6dof_ptr_, Tcb, Tvw, Tvo, R);
    problem.AddParameterBlock(vec_p.data(), 3);
    problem.AddParameterBlock(vec_q.data(), 4, new QuatLocalParameterization());  // ceres::EigenQuaternionManifold()
    problem.AddResidualBlock(cost_function, loss_function, vec_p.data(), vec_q.data());

    ceres::Solver::Options options;
    // options.dynamic_sparsity = true;
    options.max_num_iterations = 50;
    // options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    // options.minimizer_type = ceres::TRUST_REGION;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
  }
  vec_pq << vec_p, vec_q;
  state_est.set_vec_pq(vec_pq);
#endif

#if WITH_G2O
  g2o::OptimizationAlgorithm *solver = nullptr;

  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> Block;  // 每个误差项 优化变量维度，误差值维度
  // Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();  // 线性方程求解器
  // Block *solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));        // 矩阵块求解器
  // solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));

  std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverDense<Block::PoseMatrixType>());
  std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));
  // solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
  solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  // solver = new g2o::OptimizationAlgorithmDogleg(std::move(solver_ptr));

  // (dynamic_cast<g2o::OptimizationAlgorithmLevenberg*>(solver))->setUserLambdaInit(1.0);

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);
  bool stop_flag = false;
  optimizer.setForceStopFlag(&stop_flag);

  // g2o::VertexSE3 *v_pose = new g2o::VertexSE3();
  VertexPose *v_pose = new VertexPose();
  v_pose->setEstimate(state_est.pose());
  v_pose->setId(0);
  v_pose->setFixed(false);
  v_pose->setMarginalized(false);
  optimizer.addVertex(v_pose);

  EdgePose *e_pose = new EdgePose(factor_odom6dof_ptr_);
  e_pose->setId(0);
  e_pose->setVertex(0, v_pose);
  e_pose->setMeasurement(Tvo);
  {
    e_pose->setInformation(InfoMat);

    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
    e_pose->setRobustKernel(rk);
    rk->setDelta(0.1);  // e_pose->th_huber_
  }
  optimizer.addEdge(e_pose);

  double lambda = 1.0;
  int its[2] = {30, 30};
  for (int i = 0; i < 2; i++) {
    if (i == 1) v_pose->A().noalias() += Eigen::Matrix<double, 6, 6>::Identity() * lambda;

    v_pose->setEstimate(state_est.pose());

    optimizer.initializeOptimization();
    optimizer.optimize(its[i]);
  }

  // g2o::VertexSE3 *pose_est = dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(0));
  state_est.set_pose(v_pose->estimate());
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
