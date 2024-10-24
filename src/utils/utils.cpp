//
// Created by daybeha on 24-2-26.
//

#include "utils.h"

//template<typename T>
//void get_setting_from_ptree(T &var, boost::property_tree::ptree &settings, std::string name, T default_value) {
//    try {
//        var = settings.get<T>(name);
//    }
//    catch (boost::property_tree::ptree_bad_path pbp) {
//        var = default_value;
//        std::cout << "SETTINGS(warning): " << name << " not found so default (" << default_value << ") used."
//                  << std::endl;
//    }
//}

bool
get_setting_child(boost::property_tree::ptree &child, boost::property_tree::ptree &settings, std::string name,
                  bool pause_on_error) {
    try {
        child = settings.get_child(name);
    }
    catch (boost::property_tree::ptree_bad_path pbp) {
        std::cout << "SETTINGS(error): " << name << " child not found." << std::endl;
//		if (pause_on_error)
//			std::cin.get();
        return false;
    }
    return true;
}

// % Clip the input angle to between 0 and 2pi radians
double clip_rad_360(double angle) {
    while (angle < 0)
        angle += 2.0 * M_PI;

    while (angle >= 2.0 * M_PI)
        angle -= 2.0 * M_PI;

    return angle;
}

// % Clip the input angle to between -pi and pi radians
double clip_rad_180(double angle) {
    while (angle > M_PI)
        angle -= 2.0 * M_PI;

    while (angle <= -M_PI)
        angle += 2.0 * M_PI;

    return angle;
}

//% Get the signed delta angle from angle1 to angle2 handling the wrap from 2pi
//% to 0.
double get_signed_delta_rad(double angle1, double angle2) {
    double dir = clip_rad_180(angle2 - angle1);

    double delta_angle = clip_rad_360(angle1) - clip_rad_360(angle2);
    delta_angle = fabs(delta_angle);

    if (delta_angle < 2.0 * M_PI - delta_angle) {
        if (dir > 0)
            return delta_angle;
        else
            return -delta_angle;
    } else {
        if (dir > 0)
            return 2.0 * M_PI - delta_angle;
        else
            return -(2.0 * M_PI - delta_angle);
    }
}


////---------Math functions
//double mod(double x, double y) {
//    double x0 = x;
//
//    while (x0 < 0.)
//        x0 += y;
//
//    return fmod(x0, y);
//}
//
////comput the distance angle1-angle2,
////corrected into range[-PI,PI)
//double disAngle(double angle1, double angle2) {
//    return mod(angle1 - angle2 + M_PI, 2 * M_PI) - M_PI;
//}
//

// 计算两个von Mises分布的乘积，并返回新的von Mises分布参数
std::pair<double, double> multiplyVonMises(double mu1, double kappa1, double mu2, double kappa2) {
  // 第一步：计算新的集中度kappa
  double delta_mu = mu1 - mu2;
  double kappa_new = std::sqrt(kappa1 * kappa1 + kappa2 * kappa2 + 2 * kappa1 * kappa2 * std::cos(delta_mu));

  // 第二步：计算新的均值角mu
  std::complex<double> z1(kappa1 * std::cos(mu1), kappa1 * std::sin(mu1));
  std::complex<double> z2(kappa2 * std::cos(mu2), kappa2 * std::sin(mu2));
  std::complex<double> z_sum = z1 + z2;

  double mu_new = std::arg(z_sum);  // 计算复数的辐角，即新的均值角

  // 返回新的均值角和集中度
  return std::make_pair(mu_new, kappa_new);
}


// 将旋转矩阵转换为欧拉角 (Roll, Pitch, Yaw)
cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat& R) {
  // 确保 R 是 3x3 矩阵
  assert(R.rows == 3 && R.cols == 3);

  float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

  bool singular = sy < 1e-6; // 如果 sy 非常小，表示角度接近奇异解

  float x, y, z;
  if (!singular) {
    x = atan2(R.at<double>(2, 1), R.at<double>(2, 2)); // roll
    y = atan2(-R.at<double>(2, 0), sy);                 // pitch
    z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));  // yaw
  } else {
    x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1)); // roll
    y = atan2(-R.at<double>(2, 0), sy);                 // pitch
    z = 0;                                              // yaw
  }

  return cv::Vec3f(x, y, z); // 返回欧拉角 (roll, pitch, yaw)
}