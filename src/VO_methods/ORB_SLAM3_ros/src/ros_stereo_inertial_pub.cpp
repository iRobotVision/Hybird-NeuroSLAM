#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"

#include<opencv2/core/core.hpp>

#include "System.h"
#include "ImuTypes.h"

using namespace std;
string dataset = "kitti";
nav_msgs::Path path;
ros::Publisher pub_pose, pub_path, pub_odom, pub_image;

string traj_outpath;
std::vector<double> timestamps;
std::vector<Sophus::SE3f> Twcs;

class ImuGrabber {
public:
    ImuGrabber() {};

    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber {
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, const bool bRect, const bool bClahe) : mpSLAM(pSLAM),
                                                                                                      mpImuGb(pImuGb),
                                                                                                      do_rectify(bRect),
                                                                                                      mbClahe(bClahe) {}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr &msg);

    void GrabImageRight(const sensor_msgs::ImageConstPtr &msg);

    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);

    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft, mBufMutexRight;

    ORB_SLAM3::System *mpSLAM;
    ImuGrabber *mpImuGb;

    const bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
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
    odom_output.header = robot_pose.header;

    // https://zhuanlan.zhihu.com/p/342481675
    // ORB-SLAM 的坐标系为 X 轴向右，Y 轴向下，Z 轴向前。
    // 而 ROS 的坐标系为 X 轴向前，Y 轴向左，Z 轴向上。
    auto trans = Twc.translation();
    auto quat = Twc.unit_quaternion();
    if (dataset == "kitti"){
        robot_pose.pose.position.x = trans[2];
        robot_pose.pose.position.y = -trans[0];
        robot_pose.pose.position.z = -trans[1];

        robot_pose.pose.orientation.w = quat.w();
        robot_pose.pose.orientation.x = quat.z();
        robot_pose.pose.orientation.y = -quat.x();
        robot_pose.pose.orientation.z = -quat.y();
    } else if (dataset == "urban"){
        robot_pose.pose.position.x = trans[1];
        robot_pose.pose.position.y = -trans[0];
        robot_pose.pose.position.z = trans[2];

        Eigen::Quaternionf quat2 = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()) *
                                    Eigen::AngleAxisf(0, ::Eigen::Vector3f::UnitY()) *
                                    Eigen::AngleAxisf(CV_PI/2, ::Eigen::Vector3f::UnitX());
        quat = quat * quat2;
        robot_pose.pose.orientation.w = quat.w();
        robot_pose.pose.orientation.x = quat.x();
        robot_pose.pose.orientation.y = quat.y();
        robot_pose.pose.orientation.z = quat.z();
    }
    path.poses.push_back(robot_pose);

    // 发布里程计
    if(path.poses.size()>1){
        ros::Time pre_time = path.poses[path.poses.size()-2].header.stamp;
        double dt = (timestamp-pre_time).toSec();
        odom_output.pose.pose = robot_pose.pose;    // 当前帧在世界坐标系下的位姿
        Sophus::SE3f Tc1c2 = prev_Tcw * Twc; // prev_Tcw 上一帧图像在世界坐标系下位姿的逆 * Twc 当前帧图像在世界坐标系下位姿 = 当前帧想对前一帧的位姿差

        if(dataset=="kitti"){
            odom_output.twist.twist.linear.x = Tc1c2.translation().y()/dt;
        }else if (dataset == "urban"){
            odom_output.twist.twist.linear.x = Tc1c2.translation().z()/dt;
        }

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "Stereo_Inertial");
    ros::NodeHandle node;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    bool bEqual = false;

    string voc_file, config_file;
    node.param<string>("/Voc_file", voc_file,
                       "/home/daybeha/Documents/github/ORB_SLAM3_detailed_comments-master/Vocabulary/ORBvoc.txt");
    node.param<string>("/config_file", config_file,
                       "/home/daybeha/Documents/github/ORB_SLAM3_detailed_comments-master/Examples/src/ORB_SLAM3/config/Urban.yaml");
    node.param<string>("/dataset", dataset,"urban");

    string IMG_L_TOPIC, IMG_R_TOPIC;
    node.param<string>("/img_l", IMG_L_TOPIC, "/stereo/left/image_mono");
    node.param<string>("/img_r", IMG_R_TOPIC, "/stereo/right/image_mono");

    int traj_format; // 0: kitti; 1:tum
    node.param<int>("/traj_format", traj_format, 1);
    node.param<string>("/traj_outpath", traj_outpath, "/home/daybeha/Documents/github/ORB_SLAM3_detailed_comments-master/Examples/src/ORB_SLAM3/results/");


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(voc_file, config_file, ORB_SLAM3::System::IMU_STEREO, true);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb, false, bEqual);

    if (igb.do_rectify) {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
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

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() ||
            D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F,
                                    igb.M1l, igb.M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F,
                                    igb.M1r, igb.M2r);
    }



    // Maximum delay, 5 seconds
    ros::Subscriber sub_imu = node.subscribe("/imu/data_raw", 1000, &ImuGrabber::GrabImu, &imugb);
    ros::Subscriber sub_img_left = node.subscribe(IMG_L_TOPIC, 100, &ImageGrabber::GrabImageLeft, &igb);
    ros::Subscriber sub_img_right = node.subscribe(IMG_R_TOPIC, 100, &ImageGrabber::GrabImageRight,
                                                   &igb);

    pub_image = node.advertise<sensor_msgs::CompressedImage>("/camera/image_raw/compressed", 1);
    pub_path = node.advertise<nav_msgs::Path>("/orb/path", 1);
    pub_odom = node.advertise<nav_msgs::Odometry>("/orb/odom", 1);
    path.header.stamp = ros::Time(0);
    path.header.frame_id="odom";

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    if (traj_format == 0)
        save_trajectory_kitti();
    else
        save_trajectory_tum();

    SLAM.SaveTrajectoryTUM(traj_outpath + "trajectory.txt");
//    SLAM.SaveKeyFrameTrajectoryTUM(traj_outpath + "trajectory_keyframe.txt");

    ros::shutdown();

    return 0;
}


void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg) {
    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg) {
    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0) {
        return cv_ptr->image.clone();
    } else {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void ImageGrabber::SyncWithImu() {
    const double maxTimeDiff = 0.01;
    while (1) {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;

        sensor_msgs::CompressedImage compressed_msg;
        compressed_msg.format = "jpeg";
        compressed_msg.header.frame_id = "camera_link";

        if (!imgLeftBuf.empty() && !imgRightBuf.empty() && !mpImuGb->imuBuf.empty()) {
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            tImRight = imgRightBuf.front()->header.stamp.toSec();

            this->mBufMutexRight.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1) {
                imgRightBuf.pop();
                tImRight = imgRightBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRight.unlock();

            this->mBufMutexLeft.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1) {
                imgLeftBuf.pop();
                tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexLeft.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff) {
                // std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImLeft > mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;

            this->mBufMutexLeft.lock();
            imLeft = GetImage(imgLeftBuf.front());
            imgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            std::vector<uchar> buffer;
            cv::imencode(".jpg", imLeft, buffer);
            compressed_msg.data.resize(buffer.size());
            memcpy(&compressed_msg.data[0], &buffer[0], buffer.size());


            this->mBufMutexRight.lock();
            imRight = GetImage(imgRightBuf.front());
            imgRightBuf.pop();
            this->mBufMutexRight.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty()) {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tImLeft) {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x,
                                    mpImuGb->imuBuf.front()->linear_acceleration.y,
                                    mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x,
                                    mpImuGb->imuBuf.front()->angular_velocity.y,
                                    mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();
            if (mbClahe) {
                mClahe->apply(imLeft, imLeft);
                mClahe->apply(imRight, imRight);
            }

            if (do_rectify) {
                cv::remap(imLeft, imLeft, M1l, M2l, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r, M2r, cv::INTER_LINEAR);
            }

            Sophus::SE3f Tcw = mpSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

            compressed_msg.header.stamp = ros::Time(tImLeft);
            publish_all(Tcw, ros::Time(tImLeft));    // 发布轨迹
            pub_image.publish(compressed_msg);


            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg) {
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    return;
}



