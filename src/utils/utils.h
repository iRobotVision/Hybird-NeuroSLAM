/*
 * openRatSLAM
 *
 * utils - General purpose utility helper functions mainly for angles and readings settings
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _RATSLAM_UTILS_H
#define _RATSLAM_UTILS_H
#include <cmath>
#include <complex>
#include <boost/math/special_functions/bessel.hpp>
#include <boost/property_tree/ptree.hpp>
using boost::property_tree::ptree;

#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>
#include <float.h>

#include "opencv2/opencv.hpp"


template<typename T>
inline void get_setting_from_ptree(T & var, boost::property_tree::ptree & settings, std::string name, T default_value)
{
    try
    {
        var = settings.get<T>(name);
    }
    catch(boost::property_tree::ptree_bad_path pbp)
    {
        var = default_value;
        std::cout << "SETTINGS(warning): " << name << " not found so default (" << default_value << ") used." << std::endl;
    }
}


bool get_setting_child(boost::property_tree::ptree & child, boost::property_tree::ptree & settings, std::string name, bool pause_on_error = true);

// % Clip the input angle to between 0 and 2pi radians
double clip_rad_360(double angle);

// % Clip the input angle to between -pi and pi radians
double clip_rad_180(double angle);

//% Get the signed delta angle from angle1 to angle2 handling the wrap from 2pi
//% to 0.
double get_signed_delta_rad(double angle1, double angle2);



// 计算两个von Mises分布的乘积，并返回新的von Mises分布参数
std::pair<double, double> multiplyVonMises(double mu1, double kappa1, double mu2, double kappa2);

////---------Math functions
//double mod(double x, double y);
////comput the distance angle1-angle2,
////corrected into range[-PI,PI)
//double disAngle(double angle1, double angle2);

cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat& R);

#endif // _RATSLAM_UTILS_H
