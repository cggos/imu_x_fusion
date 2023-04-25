/**
 * @file kaist_pub.cpp
 * @author Gavin Gao (cggos@outlook.com)
 * @brief
 * @ref https://github.com/Wallong/VINS-GPS-Wheel
 * @version 0.1
 * @date 2022-10-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <cv_bridge/cv_bridge.h>
#include <imu_x_fusion/Encoder.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <signal.h>

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "/home/cg/Downloads/KAIST/urban39-pankyo";

bool LoadSensorData(string& sensor_data_file, unordered_map<string, string>* time_data_map) {
  ifstream data_file(sensor_data_file);
  if (!data_file.is_open()) {
    cerr << "[LoadSensorData]: Failed to open sensor data file.";
    return false;
  }
  string line_str, time_str;
  while (getline(data_file, line_str)) {
    stringstream ss(line_str);
    if (!getline(ss, time_str, ',')) {
      cerr << "[LoadSensorData]: Find a bad line in the file.: " << line_str;
      return false;
    }
    time_data_map->emplace(time_str, line_str);
  }
  return true;
}

int main(int argc, char* argv[]) {
  if (argc != 2) {
    cerr << "./kaist_pub PATH_TO_FOLDER/YOUR_PATH_TO_DATASET/KAIST/urban28/urban28-pankyo \n"
         << "For example: ./kaist_pub /home/cg/Downloads/KAIST/urban39-pankyo" << endl;
    return -1;
  }
  sData_path = argv[1];

  ros::init(argc, argv, "KAIST_pub");
  ros::NodeHandle nh("~");
  ros::Publisher imu_publisher;
  ros::Publisher img_publisher;
  ros::Publisher encoder_publisher;
  ros::Publisher gps_publisher;

  imu_publisher = nh.advertise<sensor_msgs::Imu>("/kaist/imu/data_raw", 100, true);
  img_publisher = nh.advertise<sensor_msgs::Image>("/kaist/stereo/left/image_raw", 100, true);
  encoder_publisher = nh.advertise<imu_x_fusion::Encoder>("/kaist/encoder/data_raw", 100, true);
  gps_publisher = nh.advertise<sensor_msgs::NavSatFix>("/kaist/gps/data_raw", 100, true);

  unordered_map<string, string> time_encoder_map;
  string encoder_data_path = sData_path + "/sensor_data/encoder.csv";
  if (!LoadSensorData(encoder_data_path, &time_encoder_map)) {
    cerr << "[PublishData]: Failed to load encoder data.";
    return -1;
  }

  unordered_map<string, string> time_imu_map;
  string imu_data_path = sData_path + "/sensor_data/xsens_imu.csv";
  if (!LoadSensorData(imu_data_path, &time_imu_map)) {
    cerr << "[PublishData]: Failed to load imu data.";
    return -1;
  }

  string data_stamp_path = sData_path + "/sensor_data/data_stamp.csv";
  ifstream file_data_stamp(data_stamp_path);
  if (!file_data_stamp.is_open()) {
    cerr << "[PublishData]: Failed to open data_stamp file.";
    return -1;
  }

  unordered_map<string, string> time_gps_map;
  string gps_data_path = sData_path + "/sensor_data/gps.csv";
  if (!LoadSensorData(gps_data_path, &time_gps_map)) {
    cerr << "[PublishData]: Failed to load GPS data.";
    return -1;
  }

  vector<string> line_data_vec;
  line_data_vec.reserve(17);
  string line_str, value_str;
  while (getline(file_data_stamp, line_str) && ros::ok()) {
    line_data_vec.clear();
    stringstream ss(line_str);
    while (getline(ss, value_str, ',')) {
      line_data_vec.push_back(value_str);
    }

    constexpr double kToSecond = 1e-9;
    const string time_str = line_data_vec[0];
    const double timestamp = stod(time_str) * kToSecond;

    const string& sensor_type = line_data_vec[1];
    if (sensor_type == "stereo") {
      const string img_file = sData_path + "/image/stereo_left/" + time_str + ".png";
      const cv::Mat raw_image = cv::imread(img_file, cv::IMREAD_ANYDEPTH);
      if (raw_image.empty()) {
        cerr << "[PublishData]: Failed to open image at time: " << time_str;
        return -1;
      }

      cv::Mat color_img;
      cv::cvtColor(raw_image, color_img, cv::COLOR_BayerRG2RGB);

      cv::Mat gray_img;
      cv::cvtColor(color_img, gray_img, cv::COLOR_RGB2GRAY);

      sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_img).toImageMsg();
      ;
      ros::Time stamp(timestamp);
      img_msg->header.stamp = stamp;

      img_publisher.publish(img_msg);
      // pSystem->PubImageData(timestamp, gray_img);
    }

    if (sensor_type == "encoder") {
      if (time_encoder_map.find(time_str) == time_encoder_map.end()) {
        ROS_ERROR("[PublishData]: Failed to find encoder data at time: %s", time_str);
      }
      const string& encoder_str = time_encoder_map.at(time_str);
      stringstream encoder_ss(encoder_str);
      line_data_vec.clear();
      while (getline(encoder_ss, value_str, ',')) {
        line_data_vec.push_back(value_str);
      }

      imu_x_fusion::Encoder encoder_msg;
      ros::Time stamp(timestamp);
      encoder_msg.header.stamp = stamp;
      encoder_msg.header.frame_id = "encoder_frame";
      encoder_msg.left_encoder = std::stoi(line_data_vec[1]);
      encoder_msg.right_encoder = std::stoi(line_data_vec[2]);
      encoder_publisher.publish(encoder_msg);
    }

    if (sensor_type == "gps") {
      if (time_gps_map.find(time_str) == time_gps_map.end()) {
        ROS_ERROR("[PublishData]: Failed to find gps data at time: %s", time_str);
      }
      const string& gps_str = time_gps_map.at(time_str);
      stringstream gps_ss(gps_str);
      line_data_vec.clear();
      while (getline(gps_ss, value_str, ',')) {
        line_data_vec.push_back(value_str);
      }

      sensor_msgs::NavSatFix gps_msg;
      ros::Time stamp(timestamp);
      gps_msg.header.stamp = stamp;
      gps_msg.header.frame_id = "gps_frame";
      gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
      std::stoi(line_data_vec[1]);
      gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
      gps_msg.latitude = std::stod(line_data_vec[1]);
      gps_msg.longitude = std::stod(line_data_vec[2]);
      gps_msg.altitude = std::stod(line_data_vec[3]);
      for (int i = 0; i < 9; i++) {
        gps_msg.position_covariance[i] = std::stod(line_data_vec[i + 4]) / 50;
      }
      gps_publisher.publish(gps_msg);
    }

    if (sensor_type == "imu") {
      if (time_imu_map.find(time_str) == time_imu_map.end()) {
        cerr << "[PublishData]: Failed to find imu data at time: " << time_str;
        return -1;
      }
      const string& imu_str = time_imu_map.at(time_str);
      stringstream imu_ss(imu_str);
      line_data_vec.clear();
      while (getline(imu_ss, value_str, ',')) {
        line_data_vec.push_back(value_str);
      }

      sensor_msgs::Imu imu_msg;
      ros::Time stamp(timestamp);
      imu_msg.header.stamp = stamp;
      imu_msg.header.frame_id = "imu_frame";
      imu_msg.orientation.x = std::stod(line_data_vec[1]);
      imu_msg.orientation.y = std::stod(line_data_vec[2]);
      imu_msg.orientation.z = std::stod(line_data_vec[3]);
      imu_msg.orientation.w = std::stod(line_data_vec[4]);
      imu_msg.orientation_covariance[0] = 99999.9;
      imu_msg.orientation_covariance[4] = 99999.9;
      imu_msg.orientation_covariance[8] = 99999.9;
      imu_msg.angular_velocity.x = std::stod(line_data_vec[8]);
      imu_msg.angular_velocity.y = std::stod(line_data_vec[9]);
      imu_msg.angular_velocity.z = std::stod(line_data_vec[10]);
      imu_msg.linear_acceleration.x = std::stod(line_data_vec[11]);
      imu_msg.linear_acceleration.y = std::stod(line_data_vec[12]);
      imu_msg.linear_acceleration.z = std::stod(line_data_vec[13]);
      imu_publisher.publish(imu_msg);
      usleep(10000 * nDelayTimes);  // usleep 1e-6, 5000*2 = 10000为10ms, 100hz
    }
  }
  ros::shutdown();

  return 0;
}
