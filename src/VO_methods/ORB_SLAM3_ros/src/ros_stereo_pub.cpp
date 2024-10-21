
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"

#include<opencv2/opencv.hpp>

#include"System.h"

using namespace std;
nav_msgs::Path path;
ros::Publisher pub_pose, pub_path, pub_odom, pub_image;

string traj_outpath;
std::vector<double> timestamps;
std::vector<Sophus::SE3f> Twcs;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

void save_trajectory_kitti(){
    std::ofstream file(traj_outpath +"trajectory_kitti.txt");
    if (!file.is_open()) {
        std::cerr << "无法打开文件：" << traj_outpath << std::endl;
        return;
    }

    for (int i=0; i<Twcs.size(); i++){
        Eigen::Matrix3f Rwc = Twcs[i].rotationMatrix();
        Eigen::Vector3f twc = Twcs[i].translation();
        file << setprecision(9) << Rwc(0,0) << " " << Rwc(0,1)  << " " << Rwc(0,2) << " "  << twc(0) << " " <<
          Rwc(1,0) << " " << Rwc(1,1)  << " " << Rwc(1,2) << " "  << twc(1) << " " <<
          Rwc(2,0) << " " << Rwc(2,1)  << " " << Rwc(2,2) << " "  << twc(2) << endl;
    }

    file.close();
    std::cout << "kitti格式轨迹已成功写入文件：" << traj_outpath << std::endl;
}

void save_trajectory_tum(){
    std::ofstream file(traj_outpath +"trajectory_tum.txt");
    if (!file.is_open()) {
        std::cerr << "无法打开文件：" << traj_outpath << std::endl;
        return;
    }

    for (int i=0; i<Twcs.size(); i++){
        Eigen::Quaternionf quaternion(Twcs[i].rotationMatrix());
        Eigen::Vector3f twc = Twcs[i].translation();

        file << std::fixed << std::setprecision(9) << timestamps[i] << " "
             << twc(0) << " "
             << twc(1) << " "
             << twc(2) << " "
             << quaternion.x() << " " << quaternion.y() << " " <<  quaternion.z() << " " << quaternion.w() << std::endl;
    }

    file.close();
    std::cout << "tum格式轨迹已成功写入文件：" << traj_outpath << std::endl;
}

/// 发布轨迹
void publish_all(Sophus::SE3f &Tcw, const ros::Time &timestamp){
    static Sophus::SE3f prev_Tcw;
    Sophus::SE3f Twc = Tcw.inverse(); // Twc == Twc
    Twcs.emplace_back(Twc);
    timestamps.emplace_back(timestamp.toSec());

    static nav_msgs::Odometry odom_output;
    geometry_msgs::PoseStamped robot_pose;

    robot_pose.header.frame_id = "odom";
    robot_pose.header.seq++;
    robot_pose.header.stamp =  timestamp;
    odom_output.header.frame_id = "odom";
    odom_output.header.seq++;
    odom_output.header.stamp = timestamp;

    // https://zhuanlan.zhihu.com/p/342481675
    // ORB-SLAM 的坐标系为 X 轴向右，Y 轴向下，Z 轴向前。
    // 而 ROS 的坐标系为 X 轴向前，Y 轴向左，Z 轴向上。
    auto trans = Twc.translation();
    robot_pose.pose.position.x = trans[2];
    robot_pose.pose.position.y = -trans[0];
    robot_pose.pose.position.z = -trans[1];

    auto quat = Twc.unit_quaternion();
    robot_pose.pose.orientation.w = quat.w();
    robot_pose.pose.orientation.x = quat.z();
    robot_pose.pose.orientation.y = -quat.x();
    robot_pose.pose.orientation.z = -quat.y();
    path.poses.push_back(robot_pose);

    // 发布里程计
    if(path.poses.size()>1){
        ros::Time pre_time = path.poses[path.poses.size()-2].header.stamp;
        double dt = (timestamp-pre_time).toSec();
        odom_output.pose.pose = robot_pose.pose;    // 当前帧在世界坐标系下的位姿
        Sophus::SE3f Tc1c2 = prev_Tcw * Twc; // prev_Tcw 上一帧图像在世界坐标系下位姿的逆 * Twc 当前帧图像在世界坐标系下位姿 = 当前帧想对前一帧的位姿差

        odom_output.twist.twist.linear.x = Tc1c2.translation().z()/dt;
        odom_output.twist.twist.angular.z = (tf::getYaw(path.poses.back().pose.orientation)
                                             - tf::getYaw(path.poses[path.poses.size()-2].pose.orientation))/dt;
        pub_odom.publish(odom_output);

        prev_Tcw = Tcw;
//        prev_Twc = Twc;
    }
    //发布轨迹和实时位姿
    pub_path.publish(path);
//    pub_pose.publish(robot_pose); // odom内包含了pose

}



void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;

    sensor_msgs::CompressedImage compressed_msg;
    compressed_msg.header = msgLeft->header;
    compressed_msg.format = "jpeg";
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    std::vector<uchar> buffer;
    cv::imencode(".jpg", cv_ptrLeft->image, buffer);
    compressed_msg.data.resize(buffer.size());
    memcpy(&compressed_msg.data[0], &buffer[0], buffer.size());


    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw;
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Tcw = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    pub_image.publish(compressed_msg);
    publish_all(Tcw, msgLeft->header.stamp);    // 发布轨迹
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::NodeHandle node;

    string voc_file, config_file;
    node.param<string>("/Voc_file", voc_file, "/home/daybeha/Documents/github/ORB_SLAM3_detailed_comments-master/Vocabulary/ORBvoc.txt");
    node.param<string>("/config_file", config_file, "/home/daybeha/Documents/github/ORB_SLAM3/Examples/ROS/ORB_SLAM3/src/config/KITTI00-02.yaml");

    int traj_format; // 0: kitti; 1:tum
    node.param<int>("/traj_format", traj_format, 1);
    node.param<string>("/traj_outpath", traj_outpath, "/home/daybeha/Documents/github/ORB_SLAM3_detailed_comments-master/Examples/src/ORB_SLAM3/results/");


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(voc_file, config_file,ORB_SLAM3::System::STEREO,true);
    ImageGrabber igb(&SLAM);
    node.param<bool>("/do_rectify", igb.do_rectify, false);

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    string IMG_L_TOPIC, IMG_R_TOPIC;
    node.param<string>("/img_l", IMG_L_TOPIC, "/camera/left/image_raw");
    node.param<string>("/img_r", IMG_R_TOPIC, "/camera/right/image_raw");

    message_filters::Subscriber<sensor_msgs::Image> left_sub(node, IMG_L_TOPIC, 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(node, IMG_R_TOPIC, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    pub_image = node.advertise<sensor_msgs::CompressedImage>("/camera/image_raw/compressed", 1);
    pub_path = node.advertise<nav_msgs::Path>("/orb/path", 1);
    pub_odom = node.advertise<nav_msgs::Odometry>("/orb/odom", 1);
    path.header.stamp = ros::Time(0);
    path.header.frame_id="odom";

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (traj_format == 0)
        save_trajectory_kitti();
    else
        save_trajectory_tum();

    ros::shutdown();

    return 0;
}
