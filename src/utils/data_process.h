
#include <iostream>
#include <fstream>
#include <filesystem>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

struct ImuData {
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};

void get_imgs(std::vector<std::string>& imagePaths, const std::string& folderPath){
    cv::glob(folderPath + "*.png", imagePaths, false);  // false代表不会搜索子文件夹

    if (imagePaths.empty()) {
        std::cout << "No images found in the folder." << std::endl;
        return;
    }
}

string get_file_name(const std::string& filePath){
    string img_name;
    size_t pos = filePath.find_last_of('/'); // 找到最后一个\或/的位置
    if (pos != std::string::npos) // 如果找到了
    {
        img_name = filePath.substr(pos + 1);
//        std::cout << img_name << std::endl; // 输出：MyFile.bat
    }
    return img_name;
}

void show_image(const string& winname, const string& imagepath, shared_ptr<cv::Mat> &image){
    cv::Mat img_show = image->clone();
    string img_name = get_file_name(imagepath);
//    cout << "\nIn processing: " << img_name << endl;
    cv::putText(img_show, img_name, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    cv::imshow(winname, img_show);
}

void get_timestamp(std::vector<double>& timestamps, const std::string& timestampFile){
    ifstream fin(timestampFile);
    if (!fin.is_open()) {
        cout << "Error opening file" << endl;
        return;
    }
    float timestamp;
    while (fin >> timestamp) {
        timestamps.push_back(timestamp);
    }
    fin.close();
}

/// 读取kitti格式的时间戳
void get_timestamp_kitti(vector<double> &timestamps, string &timestamp_file){
    int hour, minute;
    double second;
    char delimiter;
    std::ifstream file(timestamp_file);
    if (file) {
        std::string timestamp;
        while (std::getline(file, timestamp)) {
            timestamp = timestamp.substr(11, timestamp.size() - 1); // 不要日期
            std::stringstream ss(timestamp);
            ss >> hour >> delimiter >> minute >> delimiter >> second;
//            std::cout << "Hour: " << hour << std::endl;
//            std::cout << "Minute: " << minute << std::endl;
//            std::cout << "Second: " << second << std::endl;
//            double time_in_s = hour * 3600 + minute * 60 + second;
//            std::cout << "Total seconds: " << time_in_s << std::endl;
            timestamps.emplace_back(hour * 3600 + minute * 60 + second);
        }
    } else {
        std::cout << "Invalid file path: " << timestamp_file << std::endl;
    }
}

void get_data_paths(vector<string> &dataPaths, string &data_path){
    cv::glob(data_path + "*.txt", dataPaths, false);  // false代表不会搜索子文件夹
    if (dataPaths.empty()) {
        std::cout << "No data found in folder: " << data_path << std::endl;
        return;
    }
}

void get_oxts_data(const string &datapath, ImuData &imu_data){
    double latitude, longitude, altitude, roll, pitch, yaw, velocity_north, velocity_east, velocity_forward,
            velocity_leftward, velocity_upward, ax, ay, az, af, al, au, wx, wy, wz, wf, wl, wu, pos_accuracy, vel_accuracy;
    int navstat, numsats, posmode, velmode, orimode;

    std::ifstream file(datapath);
    if (file) {
        std::string oxts_data;
        while (std::getline(file, oxts_data)) {
            std::stringstream ss(oxts_data);
            ss >> latitude >> longitude >> altitude >> roll >> pitch >> yaw >> velocity_north >> velocity_east >> velocity_forward
               >> velocity_leftward >> velocity_upward >> ax >> ay >> az >> af >> al >> au >> wx >> wy >> wz >> wf >> wl >> wu >> pos_accuracy >> vel_accuracy
               >> navstat >> numsats >> posmode >> velmode >> orimode;

            imu_data.acc << ax, ay, az;
            imu_data.gyro << wx, wy, wz;
        }
    } else {
        std::cout << "Invalid file path: " << datapath << std::endl;
    }
}


/**
 * @brief 基于Opencv窗口响应按键:
 * q: 退出程序
 * p: 暂停程序
 * */
bool key_response(){
    int key = cv::waitKey(1);
    if (key == 'q')
        return false;
    if (key == 'p')
        cv::waitKey();
    return true;
}