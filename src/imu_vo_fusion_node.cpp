#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Core>
#include <deque>
#include <fstream>
#include <iostream>

#include "imu_x_fusion/kf.h"

namespace cg {

class FusionNode {
   public:
    FusionNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
        // ROS sub & pub
        std::string topic_vo = "/odom_vo";
        std::string topic_imu = "/imu0";

        double acc_n, gyr_n, acc_w, gyr_w;
        nh.param("acc_noise", acc_n, 1e-2);
        nh.param("gyr_noise", gyr_n, 1e-4);
        nh.param("acc_bias_noise", acc_w, 1e-6);
        nh.param("gyr_bias_noise", gyr_w, 1e-8);

        nh.getParam("topic_vo", topic_vo);
        nh.getParam("topic_imu", topic_imu);

        std::cout << "topic_vo: " << topic_vo << std::endl;
        std::cout << "topic_imu: " << topic_imu << std::endl;

        kf_ptr_ = std::make_unique<KF>(acc_n, gyr_n, acc_w, gyr_w);

        const double sigma_pv = 0.0005;
        const double sigma_rp  = 0.01 * kDegreeToRadian;
        const double sigma_yaw = 500 * kDegreeToRadian;
        kf_ptr_->set_cov(sigma_pv, sigma_pv, sigma_rp, sigma_yaw, acc_w, gyr_w);

        imu_sub_ = nh.subscribe(topic_imu, 10, &FusionNode::imu_callback, this);
        vo_sub_ = nh.subscribe(topic_vo, 10, &FusionNode::vo_callback, this);

        path_pub_ = nh.advertise<nav_msgs::Path>("nav_path", 10);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("nav_odom", 10);
        path_pub_vo_ = nh.advertise<nav_msgs::Path>("nav_path_vo", 10);

        Tcb = getTransformEigen(pnh, "cam0/T_cam_imu");

        std::cout << "Tcb: \n" << Tcb.matrix() << std::endl;
    }

    ~FusionNode() {}

    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
    void vo_callback(const nav_msgs::OdometryConstPtr &vo_msg);

    bool init_rot_from_imudata(Eigen::Matrix3d &r_GI);

    Eigen::Matrix<double, 6, 15> measurementH(const Eigen::Quaterniond &vo_q, const Eigen::Isometry3d &T);

    void publish_save_state();

   private:
    ros::Subscriber imu_sub_;
    ros::Subscriber vo_sub_;

    ros::Publisher odom_pub_;
    ros::Publisher path_pub_;
    ros::Publisher path_pub_vo_;

    nav_msgs::Path nav_path_;
    nav_msgs::Path nav_path_vo_;

    bool initialized_ = false;
    const int kImuBufSize = 200;
    std::deque<ImuDataConstPtr> imu_buf_;
    ImuDataConstPtr last_imu_ptr_;

    Eigen::Isometry3d Tcb;
    Eigen::Isometry3d Tb0bm;
    Eigen::Isometry3d Tc0cm;

    Eigen::Isometry3d TvoB;

    // KF
    KFPtr kf_ptr_;
};

void FusionNode::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
    ImuDataPtr imu_data_ptr = std::make_shared<ImuData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
    imu_data_ptr->acc[1] = imu_msg->linear_acceleration.y;
    imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
    imu_data_ptr->gyr[0] = imu_msg->angular_velocity.x;
    imu_data_ptr->gyr[1] = imu_msg->angular_velocity.y;
    imu_data_ptr->gyr[2] = imu_msg->angular_velocity.z;

    if (!initialized_) {
        imu_buf_.push_back(imu_data_ptr);
        if (imu_buf_.size() > kImuBufSize) imu_buf_.pop_front();
        return;
    }

    kf_ptr_->predict(last_imu_ptr_, imu_data_ptr);

    last_imu_ptr_ = imu_data_ptr;

    // imu_buf_.push_back(imu_data_ptr);
}

/**
 * @brief h(x)/delta X
 * 
 * @param vo_q 
 * @param T 
 * @return Eigen::Matrix<double, 6, 15> 
 */
Eigen::Matrix<double, 6, 15> FusionNode::measurementH(const Eigen::Quaterniond &vo_q, const Eigen::Isometry3d &T) {
    Eigen::Matrix<double, 6, 15> H;
    H.setZero();
    Eigen::Matrix3d R00 = Tc0cm.linear() * Tcb.linear() * Tb0bm.linear().transpose();
    H.block<3, 3>(0, 0) = -R00;
    H.block<3, 3>(0, 6) = R00 * T.linear() * skew_matrix(Tcb.inverse().translation());

    Eigen::Quaterniond q0(kf_ptr_->state_ptr_->r_GI);
    Eigen::Quaterniond q2(R00);
    Eigen::Quaterniond q1(vo_q.toRotationMatrix() * Tcb.linear());
    Eigen::Matrix4d m4 = quat_left_matrix((q2 * q0).normalized()) * quat_right_matrix(q1.conjugate());
    H.block<3, 3>(3, 6) = -m4.block<3,3>(1,1);

    H *= -1.0;

    return H;
}

void FusionNode::vo_callback(const nav_msgs::OdometryConstPtr &vo_msg) {
    Eigen::Vector3d vo_p;
    Eigen::Quaterniond vo_q;
    vo_p.x() = vo_msg->pose.pose.position.x;
    vo_p.y() = vo_msg->pose.pose.position.y;
    vo_p.z() = vo_msg->pose.pose.position.z;
    vo_q.x() = vo_msg->pose.pose.orientation.x;
    vo_q.y() = vo_msg->pose.pose.orientation.y;
    vo_q.z() = vo_msg->pose.pose.orientation.z;
    vo_q.w() = vo_msg->pose.pose.orientation.w;

    Eigen::Isometry3d Tvo;
    Tvo.linear() = vo_q.toRotationMatrix();
    Tvo.translation() = vo_p;    

    if (!initialized_) {
        if (imu_buf_.size() < kImuBufSize) {
            printf("[cggos %s] ERROR: Not Enough IMU data for Initialization!!!\n", __FUNCTION__);
            return;
        }

        last_imu_ptr_ = imu_buf_.back();
        if (std::abs(vo_msg->header.stamp.toSec() - last_imu_ptr_->timestamp) > 0.1) {
            printf("[cggos %s] ERROR: timestamps are not synchronized!!!\n", __FUNCTION__);
            return;
        }

        kf_ptr_->state_ptr_->timestamp = last_imu_ptr_->timestamp;

        std::cout << std::endl << std::endl;

        if (!init_rot_from_imudata(kf_ptr_->state_ptr_->r_GI)) return;

        Tb0bm.linear() = kf_ptr_->state_ptr_->r_GI;
        Tb0bm.translation().setZero();

        Tc0cm = Tvo;

        std::cout << "Tc0cm: " << Tc0cm.translation().transpose() << std::endl;

        initialized_ = true;

        printf("[cggos %s] System initialized.\n", __FUNCTION__);

        return;
    }

    // if(vo_msg->header.stamp.toSec() < kf_ptr_->state_ptr_->timestamp) return;
    // for(auto i = imu_buf_.begin(); i!=imu_buf_.end();) {
    //     double ts = (*i)->timestamp;
    //     double ts_vo = vo_msg->header.stamp.toSec();
    //     double ts_state = kf_ptr_->state_ptr_->timestamp;
    //     if(ts <= ts_state) {
    //         i = imu_buf_.erase(i);
    //     } else if(ts>ts_state && ts < ts_vo) {
    //         // std::cout << "[IMU]: " << std::fixed << std::setprecision(9) << ts << std::endl;
    //         kf_ptr_->predict(last_imu_ptr_, *i);
    //         last_imu_ptr_ = *i;
    //         ++i;
    //         publish_save_state();
    //     } else {
    //         break;
    //     }
    // }

    Eigen::Isometry3d Tb;
    Tb.linear() = kf_ptr_->state_ptr_->r_GI;
    Tb.translation() = kf_ptr_->state_ptr_->p_GI;

    Eigen::Isometry3d Tbvo = Tc0cm * Tcb * Tb0bm.inverse() * Tb * Tcb.inverse();

    // for publish
    TvoB = Tb0bm * Tcb.inverse() * Tc0cm.inverse() * Tvo * Tcb;

    // std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    // std::cout << "VO p: "   << Tvo.translation().transpose() << std::endl;
    // std::cout << "B p VO: " << Tbvo.translation().transpose() << std::endl;
    // std::cout << "VO p B: " << TvoB.translation().transpose() << std::endl;
    // std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

    // residual
    Eigen::Matrix<double, 6, 1> residual;
    residual.topRows(3) = Tvo.translation() - Tbvo.translation();
    residual.bottomRows(3) = 2.0 * Eigen::Quaterniond(Tvo.linear() * Tbvo.linear().transpose()).vec();

    std::cout << "res: " << residual.transpose() << std::endl;

    // jacobian
    auto H = measurementH(vo_q, Tb);

#if 1
    // check jacobian
    {
        Eigen::Vector3d delta(0.0001, -0.0003, 0.0005);

        Eigen::Isometry3d T1;
        T1.linear() = kf_ptr_->state_ptr_->r_GI;
        T1.translation() = kf_ptr_->state_ptr_->p_GI + delta;

        Eigen::Isometry3d TvoN1 = Tc0cm * Tcb * Tb0bm.inverse() * T1 * Tcb.inverse();

        auto H1 = measurementH(vo_q, T1);

        std::cout << "---------------------" << std::endl;
        std::cout << "p res: " << (TvoN1.translation() - Tbvo.translation()).transpose() << std::endl;
        std::cout << "p Hx: " << (H1.block<3,3>(0, 0) * delta).transpose() << std::endl;

        ////////////////////////////////////////////////////////////

        Eigen::Quaterniond delta_q;
        delta_q.w() = 1;
        delta_q.vec() = 0.5 * delta;

        Eigen::Isometry3d T2;
        T2.linear() = kf_ptr_->state_ptr_->r_GI * delta_q.toRotationMatrix();
        T2.translation() = kf_ptr_->state_ptr_->p_GI;

        Eigen::Isometry3d TvoN2 = Tc0cm * Tcb * Tb0bm.inverse() * T2 * Tcb.inverse();

        auto H2 = measurementH(vo_q, T2);

        std::cout << "q res: " << (TvoN2.translation() - Tbvo.translation()).transpose() << std::endl;
        std::cout << "q Hx: " << (H2.block<3,3>(0, 6) * delta).transpose() << std::endl;

        std::cout << "R q res: " << (2.0 * Eigen::Quaterniond(TvoN2.linear() * Tbvo.linear().transpose()).vec()).transpose() << std::endl;
        std::cout << "R q Hx: " << (H2.block<3,3>(3, 6) * delta).transpose() << std::endl;
        std::cout << "---------------------" << std::endl;
    }
#endif

    Eigen::Matrix<double, 6, 6> R = Eigen::Map<const Eigen::Matrix<double, 6, 6>>(vo_msg->pose.covariance.data());
    kf_ptr_->update_measurement(H, R, residual);

    publish_save_state();
}

bool FusionNode::init_rot_from_imudata(Eigen::Matrix3d &r_GI) {
    // mean and std of IMU accs
    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (const auto imu_data : imu_buf_) {
        sum_acc += imu_data->acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buf_.size();
    printf("[cggos %s] mean_acc: (%f, %f, %f)!!!\n", __FUNCTION__, mean_acc[0], mean_acc[1], mean_acc[2]);

    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (const auto imu_data : imu_buf_) sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buf_.size()).cwiseSqrt();

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

void FusionNode::publish_save_state() {
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
    Eigen::Matrix3d P_pp = kf_ptr_->state_ptr_->cov.block<3, 3>(0, 0);
    Eigen::Matrix3d P_po = kf_ptr_->state_ptr_->cov.block<3, 3>(0, 6);
    Eigen::Matrix3d P_op = kf_ptr_->state_ptr_->cov.block<3, 3>(6, 0);
    Eigen::Matrix3d P_oo = kf_ptr_->state_ptr_->cov.block<3, 3>(6, 6);
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> P_imu_pose = Eigen::Matrix<double, 6, 6>::Zero();
    P_imu_pose << P_pp, P_po, P_op, P_oo;
    for (int i = 0; i < 36; i++)
        odom_msg.pose.covariance[i] = P_imu_pose.data()[i];
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
    ros::init(argc, argv, "imu_gnss_fusion");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    cg::FusionNode fusion_node(nh, pnh);

    ros::spin();

    return 0;
}
