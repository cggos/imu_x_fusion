#pragma once

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "common/state.hpp"

namespace cg {
class Viewer {
 public:
  Viewer(ros::NodeHandle &nh) {
    path_pub_ = nh.advertise<nav_msgs::Path>("path_est", 10);
    path_pub_vo_ = nh.advertise<nav_msgs::Path>("path_vo", 10);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_est", 10);
  }

  void publish_vo(const State &state, const Eigen::Isometry3d &TvoB) {
    // publish the odometry
    std::string fixed_id = "global";
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = fixed_id;
    odom_msg.child_frame_id = "odom";

    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.linear() = state.Rwb_;
    T_wb.translation() = state.p_wb_;
    tf::poseEigenToMsg(T_wb, odom_msg.pose.pose);
    tf::vectorEigenToMsg(state.v_wb_, odom_msg.twist.twist.linear);
    const Eigen::Matrix3d &P_pp = state.cov.block<3, 3>(0, 0);
    const Eigen::Matrix3d &P_po = state.cov.block<3, 3>(0, 6);
    const Eigen::Matrix3d &P_op = state.cov.block<3, 3>(6, 0);
    const Eigen::Matrix3d &P_oo = state.cov.block<3, 3>(6, 6);
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

  void publish_gnss(const State &state) {
    // publish the odometry
    std::string fixed_id = "global";
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = fixed_id;
    odom_msg.header.stamp = ros::Time::now();
    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.linear() = state.Rwb_;
    T_wb.translation() = state.p_wb_;
    tf::poseEigenToMsg(T_wb, odom_msg.pose.pose);
    tf::vectorEigenToMsg(state.v_wb_, odom_msg.twist.twist.linear);
    Eigen::Matrix3d P_pp = state.cov.block<3, 3>(0, 0);
    Eigen::Matrix3d P_po = state.cov.block<3, 3>(0, 6);
    Eigen::Matrix3d P_op = state.cov.block<3, 3>(6, 0);
    Eigen::Matrix3d P_oo = state.cov.block<3, 3>(6, 6);
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
  }

 private:
  ros::Publisher odom_pub_;
  ros::Publisher path_pub_;
  ros::Publisher path_pub_vo_;

  nav_msgs::Path nav_path_;
  nav_msgs::Path nav_path_vo_;
};
}  // namespace cg