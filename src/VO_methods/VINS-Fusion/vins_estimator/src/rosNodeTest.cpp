#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>

#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"


Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

ros::Publisher pub_image;

string traj_outpath="/home/daybeha/Documents/github/myratslam_ws/src/ratslam_ros/results/";

void save_trajectory_tum(){
    std::ofstream file(traj_outpath +"trajectory_tum.txt");
    if (!file.is_open()) {
        std::cerr << "无法打开文件：" << traj_outpath << std::endl;
        return;
    }

    for (int i=0; i<estimator.Ps_all.size(); i++){
        Eigen::Quaterniond quaternion(estimator.Rs_all[i]);
        // TUM格式的时间戳使用秒为单位
        file << std::fixed << std::setprecision(9) << estimator.timestamps[i] << " "
             << estimator.Ps_all[i].x() << " "
                << estimator.Ps_all[i].y() << " "
                << estimator.Ps_all[i].z() << " "
                << quaternion.x() << " " << quaternion.y() << " " <<  quaternion.z() << " " << quaternion.w() << std::endl;
    }

    file.close();
    std::cout << "tum格式轨迹已成功写入文件：" << traj_outpath << std::endl;
}

void save_trajectory_kitti(){
    std::ofstream file(traj_outpath +"trajectory_kitti.txt");
    if (!file.is_open()) {
        std::cerr << "无法打开文件：" << traj_outpath << std::endl;
        return;
    }

    for (int i=0; i<estimator.Ps_all.size(); i++){
        // TUM格式的时间戳使用秒为单位
        file << std::setprecision(9)
                << estimator.Rs_all[i](0, 0) << " " << estimator.Rs_all[i](0, 1) << " " << estimator.Rs_all[i](0, 2) << " " << estimator.Ps_all[i].x() << " "
                << estimator.Rs_all[i](1, 0) << " " << estimator.Rs_all[i](1, 1) << " " << estimator.Rs_all[i](1, 2) << " " << estimator.Ps_all[i].y() << " "
                << estimator.Rs_all[i](2, 0) << " " << estimator.Rs_all[i](2, 1) << " " << estimator.Rs_all[i](2, 2) << " " << estimator.Ps_all[i].z()
                << std::endl;
    }

    file.close();
    std::cout << "kitti格式轨迹已成功写入文件：" << traj_outpath << std::endl;
}

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg, const bool &pub_compress)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();

    if(pub_compress){
        sensor_msgs::CompressedImage compressed_msg;
        compressed_msg.header = img_msg->header;
        compressed_msg.format = "jpeg";
        std::vector<uchar> buffer;
        cv::imencode(".jpg", ptr->image, buffer);
        compressed_msg.data.resize(buffer.size());
        memcpy(&compressed_msg.data[0], &buffer[0], buffer.size());
        pub_image.publish(compressed_msg);
    }
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front(), true);
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front(), false);
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty()){
//                // 去除地面部分
//                for (int i = image0.rows-150; i < image0.rows; ++i) {
//                    for (int j = 2*(image0.rows-i); j < image0.cols-2*(image0.rows-i); ++j) {
////                    for (int j = 0; j < image0.cols; ++j) {
//                        image0.at<uchar>(i, j) = 255;
//                        image1.at<uchar>(i, j) = 255;
//                    }
//                }
//                // 去除天空部分
//                for (int i = 0; i < 150; ++i) {
////                    for (int j = 2*i; j < image0.cols-2*i; ++j) {
//                    for (int j = 0; j < image0.cols; ++j) {
//                        image0.at<uchar>(i, j) = 255;
//                        image1.at<uchar>(i, j) = 255;
//                    }
//                }
//                // 去除右侧部分
//                for (int i = 0; i < image0.rows; ++i) {
//                    for (int j = image0.cols-200; j < image0.cols; ++j) {
//                        image0.at<uchar>(i, j) = 255;
//                        image1.at<uchar>(i, j) = 255;
//                    }
//                }

                estimator.inputImage(time, image0, image1);
//                cv::imshow("image0", image0);
//                cv::imshow("image1", image1);
//                cv::waitKey(1);
            }


        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front(), true);
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

string dataset_name(const std::string& filePath){
    string dataset;
    size_t pos = filePath.find_last_of('/'); // 找到最后一个\或/的位置
    if (pos != std::string::npos) // 如果找到了
    {
//        std::cout << img_name << std::endl; // 输出：MyFile.bat
        string filename = filePath.substr(pos + 1);
        size_t pos2 = filename.find_first_of('_'); // 找到最后一个\或/的位置
        dataset = filename.substr(0, pos2);
    }
    return dataset;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    int traj_format; // 0: kitti; 1:tum
    n.param<int>("/traj_format", traj_format, 1);
    n.param<string>("/traj_outpath", traj_outpath, "/home/daybeha/Documents/github/vins_ws/src/VINS-Fusion/results/");

    string config_file;
    n.param<string>("/config_path", config_file, "/home/daybeha/Documents/github/vins_ws/src/VINS-Fusion/config/kitti_odom/kitti_config04-12.yaml");
    cout << "config_file: " << config_file << endl;


    readParameters(config_file);
    estimator.setParameter();
    if(dataset_name(config_file) == "kitti"){
        estimator.setKitti();
    }else if(dataset_name(config_file) == "urban") {
        estimator.setUrban();
    }


#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu;
    if(USE_IMU)
    {
        sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    }
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1;
    if(STEREO)
    {
        sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    }
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);

    pub_image = n.advertise<sensor_msgs::CompressedImage>("/camera/image_raw/compressed", 1);

    std::thread sync_thread{sync_process};
    ros::spin();

    if (traj_format == 0)
        save_trajectory_kitti();
    else
        save_trajectory_tum();
    save_trajectory_tum();


    return 0;
}
