#include <iostream>
#include <chrono>
#include <thread>

using namespace std;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sophus/se3.hpp>
#include "utils/data_process.h"
#include "utils/utils.h"


#include "neuroslam/visual_memory.h"
#include "neuroslam/posecell_network.h"
#include "neuroslam/experience_map.h"

#include <ratslam_ros/gaussianCells.h>


#define REGISTER_TIMES 1
shared_ptr<neuroslam::VisualMemory> vm = nullptr;
shared_ptr<neuroslam::PosecellNetwork> pc = nullptr;
shared_ptr<neuroslam::ExperienceMap> em = nullptr;

ros::Publisher pub_pose, pub_marker, pub_path;
nav_msgs::Path path;
string traj_outpath = "/home/daybeha/Documents/github/Hybird-NeuroSLAM/myratslam_ws/src/ratslam_ros/results/";

ros::Publisher pub_em_path;
ros::Publisher pub_gaussian;


double velocity;

bool pub_cell_status;
bool IS_VON_MISES;
float ACT_LEVEL_THRESHOLD;


void save_trajectory_tum() {
  std::ofstream file(traj_outpath + "trajectory_tum.txt");
  if (!file.is_open()) {
    std::cerr << "无法打开文件：" << traj_outpath << std::endl;
    return;
  }

  for (int i = 0; i < em->all_ids.size(); i++) {
//        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(em->get_experience(em->all_ids[i])->th_rad -M_PI_2);
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0,
                                                                          em->get_experience(em->all_ids[i])->th_rad -
                                                                          M_PI_2, 0);
    // TUM格式的时间戳使用秒为单位
    file << std::fixed << std::setprecision(9) << em->timestamps[i] << " "
         << em->get_experience(em->all_ids[i])->x_m << " "
         << em->get_experience(em->all_ids[i])->y_m << " "
         << 0 << " "
         << q.x << " " << q.y << " " << q.z << " " << q.w << std::endl;
  }

  file.close();
  std::cout << "tum格式轨迹已成功写入文件：" << traj_outpath + "trajectory_tum.txt" << std::endl;
}

void save_trajectory_kitti() {
  std::ofstream file(traj_outpath + "trajectory_kitti.txt");
  if (!file.is_open()) {
    std::cerr << "无法打开文件：" << traj_outpath << std::endl;
    return;
  }

  for (int i = 0; i < em->all_ids.size(); i++) {
    Eigen::Matrix3d R = Sophus::SO3d::exp(
        Eigen::Vector3d(0, em->get_experience(em->all_ids[i])->th_rad - M_PI_2, 0)).matrix();

    // TUM格式的时间戳使用秒为单位
    file << std::setprecision(9)
         << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " "
         << em->get_experience(em->all_ids[i])->x_m << " "
         << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " "
         << 0 << " "
         << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " "
         << em->get_experience(em->all_ids[i])->y_m
         << std::endl;
  }

  file.close();
  std::cout << "kitti格式轨迹已成功写入文件：" << traj_outpath + "trajectory_kitti.txt" << std::endl;
}

void publish_cells(unsigned int &dest_id) {
  ratslam_ros::gaussianCells gaussian_output;
  double coordinates[6];

  pc->get_curr_coordinates(coordinates);
  for (unsigned int i = 0; i < 6; i++) {
    gaussian_output.msg_curr_Coordinate.push_back(coordinates[i]);
  }
  pc->get_cali_coordinates(coordinates);
  for (unsigned int i = 0; i < 6; i++) {
    gaussian_output.msg_cali_Coordinate.push_back(coordinates[i]);
  }
  pc->get_inject_coordinates(coordinates);
  for (unsigned int i = 0; i < 6; i++) {
    gaussian_output.msg_inject_Coordinate.push_back(coordinates[i]);
  }
  pc->get_est_coordinates(coordinates);
  for (unsigned int i = 0; i < 6; i++) {
    gaussian_output.msg_est_Coordinate.push_back(coordinates[i]);
  }

  gaussian_output.msg_exp_id = dest_id;
  pub_gaussian.publish(gaussian_output);
}

void publish_cur_pose(const ros::Time &timestamp) {
  geometry_msgs::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.header.seq++;
  robot_pose.header.stamp = timestamp;

  robot_pose.pose.position.x = em->get_experience(em->get_current_id())->x_m;
  robot_pose.pose.position.y = em->get_experience(em->get_current_id())->y_m;
  robot_pose.pose.position.z = 0;
  robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(
      em->get_experience(em->get_current_id())->th_rad);;
  em->all_ids.emplace_back(em->get_current_id());
  em->timestamps.emplace_back(timestamp.toSec());

  path.poses.push_back(robot_pose);
  pub_pose.publish(robot_pose);
  pub_path.publish(path);
}

void pubish_em_path(const ros::Time &timestamp) {
  nav_msgs::Path em_path;
  em_path.header.stamp = timestamp;
  em_path.header.frame_id = "odom";

  for (int i = 0; i < em->all_ids.size(); i++) {
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header.frame_id = "odom";
    robot_pose.header.seq++;
    robot_pose.header.stamp = ros::Time(em->timestamps[i]);

    robot_pose.pose.position.x = em->get_experience(em->all_ids[i])->x_m;
    robot_pose.pose.position.y = em->get_experience(em->all_ids[i])->y_m;
    robot_pose.pose.position.z = 0;
    robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(em->get_experience(em->all_ids[i])->th_rad);

    em_path.poses.emplace_back(robot_pose);
  }
  pub_em_path.publish(em_path);
}


void odom_callback(const nav_msgs::OdometryConstPtr &msg) {
  chrono::steady_clock::time_point begin = chrono::steady_clock::now();

  velocity = msg->twist.twist.linear.x;

  /// Pose Cell
  static ros::Time prev_time(0);
  if (prev_time.toSec() > 0) {
    double time_diff = (msg->header.stamp - prev_time).toSec();

    pc->on_odom(msg->twist.twist.linear.x, msg->twist.twist.angular.z, time_diff, IS_VON_MISES);
    auto diff = std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - begin);

    neuroslam::PosecellNetwork::PosecellAction action = pc->get_action_opt();
    unsigned int dest_id = pc->get_current_exp_id();
    double relative_rad = pc->get_relative_rad();

    // 发布cells信息以供可视化
    if (pub_cell_status)
      publish_cells(dest_id);

    /// Experience Map
    if (pc->gaussianCells.curr_Coordinates.theta.act_level > ACT_LEVEL_THRESHOLD) {
      std::vector<double> delta_pos = pc->get_delta_pos(); // x,y,th
      em->on_odo(time_diff, delta_pos);
    } else {
      em->on_odo(msg->twist.twist.linear.x, msg->twist.twist.angular.z, time_diff);   // 路径积分
    }

    switch (action) {
      case neuroslam::PosecellNetwork::PosecellAction::CREATE_NODE:
        em->on_create_experience();
        em->on_set_experience(dest_id, 0);
        break;

      case neuroslam::PosecellNetwork::PosecellAction::CREATE_EDGE:
        em->on_create_experience_reducedgraph(pc->link_dest_id);
        em->on_create_link_reducedgraph(dest_id, pc->link_dest_id, relative_rad, true); // loop closure
        em->on_set_experience(dest_id, relative_rad);
        break;

      case neuroslam::PosecellNetwork::PosecellAction::SET_NODE:
        em->on_set_experience(dest_id, relative_rad);
        break;

      case neuroslam::PosecellNetwork::PosecellAction::NO_ACTION:
        break;
    }
    em->iterate();
    publish_cur_pose(msg->header.stamp);
    pubish_em_path(msg->header.stamp);
  }

  prev_time = msg->header.stamp;

}

void image_callback(const sensor_msgs::ImageConstPtr &msg) {
  static float last_time;
  static int cnt = 0;
  if (msg->header.stamp.toSec() == last_time) {
    cout << "time interval too short, throw image" << cnt << endl;
    return;
  }
  last_time = msg->header.stamp.toSec();

  cv::Mat image = cv_bridge::toCvShare(msg, "mono8")->image;

  chrono::steady_clock::time_point begin = chrono::steady_clock::now();

  if (velocity < 0.01) {   // TODO 这个阈值需要根据实际情况调整
    ROS_DEBUG("velocity is too small, robot may stoped");
    vm->reset_relative_pose();
  } else {
    vm->on_image(image, msg->header.stamp.toSec());
  }
  pc->on_view_template_opt(vm->get_current_vt(), vm->get_relative_pose());

  auto diff = std::chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - begin);
}

int main(int argc, char *argv[]) {
  cout << "On Image window" << endl;
  cout << "Press 'q' to quit" << endl;
  cout << "Press 'p' to pause" << endl;
  cout << "Press 's' to save trajectory\n" << endl;

  // 初始化ros
  ros::init(argc, argv, "RatSLAM");
  ros::NodeHandle node;


  int traj_format; // 0: kitti; 1:tum
  node.param<int>("/traj_format", traj_format, 1);
  node.param<string>("/traj_outpath", traj_outpath,
                     "/home/daybeha/Documents/github/Hybird-NeuroSLAM/myratslam_ws/src/ratslam_ros/results/");


  // 读取config参数
  string config_file, img_topic, odom_topic;
  node.param<string>("/config_file", config_file,
                     "/home/daybeha/Documents/github/Hybird-NeuroSLAM/myratslam_ws/src/ratslam_ros/config/kitti.yaml");
  node.param<string>("/img_topic", img_topic, "/camera/image_raw");
  node.param<string>("/odom_topic", odom_topic, "/vins/odom");
  cout << "Subscribe image topic: " << img_topic << endl;
  cout << "Subscribe odom topic: " << odom_topic << endl;


  // 读取 YAML 文件
  YAML::Node yaml_node = YAML::LoadFile(config_file);
  if (yaml_node.IsNull()) {
    cerr << "config file empty: " << config_file << endl;
    return 1;
  }
  YAML::Node ratslam_settings = yaml_node["ratslam"];
  YAML::Node camera_settings = yaml_node["camera"];
  YAML::Node draw_settings = yaml_node["draw"];
  int tmp = ratslam_settings["HD_distribution"].as<int>(0);
  if (tmp == 1)
    IS_VON_MISES = true;
  else
    IS_VON_MISES = false;
  cout << "use von mises distribution: " << IS_VON_MISES << endl;
  ACT_LEVEL_THRESHOLD = ratslam_settings["act_level_threshold"].as<float>(90);

  // 初始化各功能节点
  vm = make_shared<neuroslam::VisualMemory>(ratslam_settings, camera_settings);
  pc = make_shared<neuroslam::PosecellNetwork>(ratslam_settings);
  em = make_shared<neuroslam::ExperienceMap>(ratslam_settings);
  pub_cell_status = draw_settings["enable"].as<bool>(true);


  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe(img_topic, 5, image_callback);
  ros::Subscriber sub_odom = node.subscribe<nav_msgs::Odometry>(odom_topic, 5,
                                                                odom_callback,
                                                                ros::VoidConstPtr(),  // 回调队列实例，这里使用默认值
                                                                ros::TransportHints().tcpNoDelay());  // 传输选项，这里设置了TCP无延迟

  pub_pose = node.advertise<geometry_msgs::PoseStamped>("/ratslam/ExperienceMap/RobotPose", 1);
  pub_path = node.advertise<nav_msgs::Path>("/ratslam/path", 10, true);
  path.header.stamp = ros::Time(0);
  path.header.frame_id = "odom";

  pub_em_path = node.advertise<nav_msgs::Path>("/ratslam/em_path", 10, true);
  pub_gaussian = node.advertise<ratslam_ros::gaussianCells>("/gaussianCells", 0);


  ros::spin();

  if (traj_format == 0)
    save_trajectory_kitti();
  else
    save_trajectory_tum();

  return 0;
}