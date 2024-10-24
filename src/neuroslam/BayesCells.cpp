#include "BayesCells.h"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>

using namespace std;

BayesCells::BayesCells() {
  act_level_sum_HD = 100.0;
  act_level_sum_Pos = 1.0;
  threshold = 1;
  box_size = 15;

  curr_Coordinates.theta.mu = 0;
  curr_Coordinates.theta.act_level = act_level_sum_HD;
  curr_Coordinates.x.mu = 0;
  curr_Coordinates.x.act_level = act_level_sum_Pos;
  curr_Coordinates.y.mu = 0;
  curr_Coordinates.y.act_level = act_level_sum_Pos;

  cali_Coordinates.theta.mu = 0;
  cali_Coordinates.theta.act_level = act_level_sum_HD * 0.1;
  cali_Coordinates.x.mu = 0;
  cali_Coordinates.x.act_level = act_level_sum_Pos * 0.1;
  cali_Coordinates.y.mu = 0;
  cali_Coordinates.y.act_level = act_level_sum_Pos * 0.1;

  est_Coordinates.theta.mu = 0;
  est_Coordinates.theta.act_level = act_level_sum_HD * 0.1;
  est_Coordinates.x.mu = 0;
  est_Coordinates.x.act_level = act_level_sum_Pos * 0.1;
  est_Coordinates.y.mu = 0;
  est_Coordinates.y.act_level = act_level_sum_Pos * 0.1;

  inject_Coordinates.theta.mu = 0;
  inject_Coordinates.theta.act_level = act_level_sum_HD * 0.1;
  inject_Coordinates.x.mu = 0;
  inject_Coordinates.x.act_level = act_level_sum_Pos * 0.1;
  inject_Coordinates.y.mu = 0;
  inject_Coordinates.y.act_level = act_level_sum_Pos * 0.1;

}


BayesCells::~BayesCells() {
  freeGaussianCells();
}

void BayesCells::freeGaussianCells() {

}

void BayesCells::VelocityInput(double vtrans, double vrot, double timeInterval) {

  double delta_theta = vrot * timeInterval;
  curr_Coordinates.theta.mu = curr_Coordinates.theta.mu + delta_theta;
  cali_Coordinates.theta.mu = cali_Coordinates.theta.mu + delta_theta;
  wrap(curr_Coordinates.theta.mu, 2.0 * M_PI);
  wrap(cali_Coordinates.theta.mu, 2.0 * M_PI);

//    double delta_x = cos(curr_Coordinates.theta.mu) * vtrans;
  double delta_x = cos(curr_Coordinates.theta.mu) * vtrans * timeInterval;
  curr_Coordinates.x.mu = curr_Coordinates.x.mu + delta_x;
  cali_Coordinates.x.mu = cali_Coordinates.x.mu + delta_x;
  wrap(curr_Coordinates.x.mu, box_size);
  wrap(cali_Coordinates.x.mu, box_size);

  double delta_y = sin(curr_Coordinates.theta.mu) * vtrans * timeInterval;
  curr_Coordinates.y.mu = curr_Coordinates.y.mu + delta_y;
  cali_Coordinates.y.mu = cali_Coordinates.y.mu + delta_y;
  wrap(curr_Coordinates.y.mu, box_size);
  wrap(cali_Coordinates.y.mu, box_size);

}

void BayesCells::IntrinsicDynamic(bool is_von_mises) {
  Coordinates temp_curr_Coordinates = curr_Coordinates;
  Coordinates temp_cali_Coordinates = cali_Coordinates;

  // inh 0.05; inj 0.4
//  double inhibition = 0.05; //0.05 - 0.01 - 0.02 - 0.01
  double inhibition = 0.2; //0.05 - 0.01 - 0.02 - 0.01

//    curr_Coordinates.theta.act_level = curr_Coordinates.theta.act_level - cali_Coordinates.theta.act_level * 0;
//    if(curr_Coordinates.theta.act_level <= 0) {curr_Coordinates.theta.act_level = 0.0001;}
//    curr_Coordinates.x.act_level = curr_Coordinates.x.act_level - cali_Coordinates.x.act_level * 0;
//    if(curr_Coordinates.x.act_level <= 0) {curr_Coordinates.x.act_level = 0.0001;}
//    curr_Coordinates.y.act_level = curr_Coordinates.y.act_level - cali_Coordinates.y.act_level * 0;
//    if(curr_Coordinates.y.act_level <= 0) {curr_Coordinates.y.act_level = 0.0001;}


  // 这里对应公式3  mutual inhibition
  cali_Coordinates.theta.act_level = cali_Coordinates.theta.act_level - curr_Coordinates.theta.act_level * inhibition;
  if (cali_Coordinates.theta.act_level <= 0) { cali_Coordinates.theta.act_level = 0.0001; }
  else if(cali_Coordinates.theta.act_level > act_level_sum_HD) {cali_Coordinates.theta.act_level = act_level_sum_HD;}
  cali_Coordinates.x.act_level = cali_Coordinates.x.act_level - curr_Coordinates.x.act_level * inhibition;
  if (cali_Coordinates.x.act_level <= 0) { cali_Coordinates.x.act_level = 0.0001; }
  else if(cali_Coordinates.x.act_level > act_level_sum_Pos) {cali_Coordinates.x.act_level = act_level_sum_Pos;}
  cali_Coordinates.y.act_level = cali_Coordinates.y.act_level - curr_Coordinates.y.act_level * inhibition;
  if (cali_Coordinates.y.act_level <= 0) { cali_Coordinates.y.act_level = 0.0001; }
  else if(cali_Coordinates.y.act_level > act_level_sum_Pos) {cali_Coordinates.y.act_level = act_level_sum_Pos;}

  // 这里的curr就是论文里的inte （公式2） global inhibition
  temp_curr_Coordinates.theta.act_level = act_level_sum_HD * curr_Coordinates.theta.act_level /
                                          (curr_Coordinates.theta.act_level + cali_Coordinates.theta.act_level);
  temp_curr_Coordinates.x.act_level = act_level_sum_Pos * curr_Coordinates.x.act_level /
                                      (curr_Coordinates.x.act_level + cali_Coordinates.x.act_level);
  temp_curr_Coordinates.y.act_level = act_level_sum_Pos * curr_Coordinates.y.act_level /
                                      (curr_Coordinates.y.act_level + cali_Coordinates.y.act_level);

  temp_cali_Coordinates.theta.act_level = act_level_sum_HD * cali_Coordinates.theta.act_level /
                                          (cali_Coordinates.theta.act_level + curr_Coordinates.theta.act_level);
  temp_cali_Coordinates.x.act_level = act_level_sum_Pos * cali_Coordinates.x.act_level /
                                      (cali_Coordinates.x.act_level + curr_Coordinates.x.act_level);
  temp_cali_Coordinates.y.act_level = act_level_sum_Pos * cali_Coordinates.y.act_level /
                                      (cali_Coordinates.y.act_level + curr_Coordinates.y.act_level);

  curr_Coordinates = temp_curr_Coordinates;
  cali_Coordinates = temp_cali_Coordinates;
}

void BayesCells::Estimation(bool is_von_mises) {
  // theta
  Coordinates temp_curr_Coordinates = curr_Coordinates;
  Coordinates temp_cali_Coordinates = cali_Coordinates;

  if (is_von_mises) {
    // von mises distribution
    auto result = multiplyVonMises(curr_Coordinates.theta.mu,
                                   curr_Coordinates.theta.act_level,
                                   cali_Coordinates.theta.mu,
                                   cali_Coordinates.theta.act_level);
    est_Coordinates.theta.mu = result.first;
    est_Coordinates.theta.act_level = result.second;
  } else {
    if (fabs(curr_Coordinates.theta.mu - cali_Coordinates.theta.mu) > M_PI) {
      if (curr_Coordinates.theta.mu > cali_Coordinates.theta.mu) {
        temp_curr_Coordinates.theta.mu = curr_Coordinates.theta.mu - 2.0 * M_PI;
      } else if (curr_Coordinates.theta.mu < cali_Coordinates.theta.mu) {
        temp_cali_Coordinates.theta.mu = cali_Coordinates.theta.mu - 2.0 * M_PI;
      }
    }
    // Gaussian distribution
    est_Coordinates.theta.act_level =
        curr_Coordinates.theta.act_level + cali_Coordinates.theta.act_level;
    est_Coordinates.theta.mu = (temp_curr_Coordinates.theta.mu * curr_Coordinates.theta.act_level
                                + temp_cali_Coordinates.theta.mu * cali_Coordinates.theta.act_level) /
                               est_Coordinates.theta.act_level;

    wrap(est_Coordinates.theta.mu, 2.0 * M_PI);
  }

  // x
  if (fabs(curr_Coordinates.x.mu - cali_Coordinates.x.mu) > box_size / 2.0) {
    if (curr_Coordinates.x.mu > cali_Coordinates.x.mu) {
      temp_curr_Coordinates.x.mu = curr_Coordinates.x.mu - box_size;
    } else if (curr_Coordinates.x.mu < cali_Coordinates.x.mu) {
      temp_cali_Coordinates.x.mu = cali_Coordinates.x.mu - box_size;
    }
  }
  est_Coordinates.x.act_level =
      curr_Coordinates.x.act_level + cali_Coordinates.x.act_level;
  est_Coordinates.x.mu = (temp_curr_Coordinates.x.mu * curr_Coordinates.x.act_level
                          + temp_cali_Coordinates.x.mu * cali_Coordinates.x.act_level) / est_Coordinates.x.act_level;
  wrap(est_Coordinates.x.mu, box_size);

  // y
  if (fabs(curr_Coordinates.y.mu - cali_Coordinates.y.mu) > box_size / 2.0) {
    if (curr_Coordinates.y.mu > cali_Coordinates.y.mu) {
      temp_curr_Coordinates.y.mu = curr_Coordinates.y.mu - box_size;
    } else if (curr_Coordinates.y.mu < cali_Coordinates.y.mu) {
      temp_cali_Coordinates.y.mu = cali_Coordinates.y.mu - box_size;
    }
  }
  est_Coordinates.y.act_level =
      curr_Coordinates.y.act_level + cali_Coordinates.y.act_level;
  est_Coordinates.y.mu = (temp_curr_Coordinates.y.mu * curr_Coordinates.y.act_level
                          + temp_cali_Coordinates.y.mu * cali_Coordinates.y.act_level) / est_Coordinates.y.act_level;

  wrap(est_Coordinates.y.mu, box_size);
}


void BayesCells::ShowCurrentCoords() {
  ROS_INFO_STREAM(
      "curr_Coordinates:position(theta,x,y) " << curr_Coordinates.theta.mu * 180 / M_PI << ", " << curr_Coordinates.x.mu
                                              << ", "
                                              << curr_Coordinates.y.mu);
  ROS_INFO_STREAM("curr_Coordinates:act_level(theta,x,y) " << curr_Coordinates.theta.act_level << ", "
                                                           << curr_Coordinates.x.act_level << ", "
                                                           << curr_Coordinates.y.act_level);
  ROS_INFO_STREAM(
      "cali_Coordinates:position(theta,x,y) " << cali_Coordinates.theta.mu * 180 / M_PI << ", " << cali_Coordinates.x.mu
                                              << ", "
                                              << cali_Coordinates.y.mu);
  ROS_INFO_STREAM("cali_Coordinates:act_level(theta,x,y) " << cali_Coordinates.theta.act_level << ", "
                                                           << cali_Coordinates.x.act_level << ", "
                                                           << cali_Coordinates.y.act_level);
  ROS_INFO_STREAM(
      "est_Coordinates:position(theta,x,y) " << est_Coordinates.theta.mu * 180 / M_PI << ", " << est_Coordinates.x.mu
                                             << ", "
                                             << est_Coordinates.y.mu);
  ROS_INFO_STREAM("est_Coordinates:act_level(theta,x,y) " << est_Coordinates.theta.act_level << ", "
                                                          << est_Coordinates.x.act_level << ", "
                                                          << est_Coordinates.y.act_level);
  ROS_INFO_STREAM("inject_Coordinates:position(theta,x,y) " << inject_Coordinates.theta.mu * 180 / M_PI << ", "
                                                            << inject_Coordinates.x.mu << ", "
                                                            << inject_Coordinates.y.mu);

//    cout <<"curr_Coordinates:position(theta,x,y) "<<curr_Coordinates.theta.mu<<", "<<curr_Coordinates.x.mu<<", "<<curr_Coordinates.y.mu<<", "<<endl;
////    cout <<"curr_Coordinates:act_level(theta,x,y) "<<curr_Coordinates.theta.act_level<<", "<<curr_Coordinates.x.act_level<<","<<curr_Coordinates.y.act_level<<","<<endl;
//    cout <<"cali_Coordinates:position(theta,x,y) "<<cali_Coordinates.theta.mu<<", "<<cali_Coordinates.x.mu<<", "<<cali_Coordinates.y.mu<<", "<<endl;
////    cout <<"cali_Coordinates:act_level(theta,x,y) "<<cali_Coordinates.theta.act_level<<", "<<cali_Coordinates.x.act_level<<","<<cali_Coordinates.y.act_level<<","<<endl;
//    cout <<"est_Coordinates:position(theta,x,y) "<<est_Coordinates.theta.mu<<", "<<est_Coordinates.x.mu<<", "<<est_Coordinates.y.mu<<", "<<endl;
//    cout <<"inject_Coordinates:position(theta,x,y) "<<inject_Coordinates.theta.mu<<", "<<inject_Coordinates.x.mu<<", "<<inject_Coordinates.y.mu<<", "<<endl;
////    cout <<"est_Coordinates:act_level(theta,x,y) "<<est_Coordinates.theta.act_level<<", "<<est_Coordinates.x.act_level<<","<<est_Coordinates.y.act_level<<","<<endl;

  cout << endl;
}


void BayesCells::PathIntegration(double vtrans, double vrot, double timeInterval, bool is_von_mises) {
  // velocity input
  VelocityInput(vtrans, vrot, timeInterval);

  // intrinsic dynamics
  IntrinsicDynamic(is_von_mises);

  // estimation
  Estimation(is_von_mises);

  // showCoords
//  ShowCurrentCoords();
}

void BayesCells::Calibration_GaussianCells(double theta, double x, double y, bool is_von_mises) {
  double inject = 0.4; //irat,stlucia 0.4, oxford 0.1
  // calibration
  inject_Coordinates.theta.mu = theta;
  inject_Coordinates.theta.act_level = act_level_sum_HD * inject;
  inject_Coordinates.x.mu = x;
  inject_Coordinates.x.act_level = act_level_sum_Pos * inject;
  inject_Coordinates.y.mu = y;
  inject_Coordinates.y.act_level = act_level_sum_Pos * inject;

  Coordinates temp_Coordinates;
  Coordinates temp_cali_Coordinates = cali_Coordinates;
  Coordinates temp_inject_Coordinates = inject_Coordinates;

  if (is_von_mises) {
    // von mises distribution
    auto result = multiplyVonMises(cali_Coordinates.theta.mu, cali_Coordinates.theta.act_level,
                                   inject_Coordinates.theta.mu, inject_Coordinates.theta.act_level);
    temp_Coordinates.theta.mu = result.first;
    temp_Coordinates.theta.act_level = result.second;
    if(temp_Coordinates.theta.act_level <= 0) {temp_Coordinates.theta.act_level = 0.0001;}
    else if(temp_Coordinates.theta.act_level > 100) {temp_Coordinates.theta.act_level = 100;}
  } else {
    // Gaussian distribution
    if (fabs(cali_Coordinates.theta.mu - inject_Coordinates.theta.mu) > M_PI) {
      if (cali_Coordinates.theta.mu > inject_Coordinates.theta.mu) {
        temp_cali_Coordinates.theta.mu = cali_Coordinates.theta.mu - 2.0 * M_PI;
      } else if (cali_Coordinates.theta.mu < inject_Coordinates.theta.mu) {
        temp_inject_Coordinates.theta.mu = inject_Coordinates.theta.mu - 2.0 * M_PI;
      }
    }
    temp_Coordinates.theta.act_level = cali_Coordinates.theta.act_level + inject_Coordinates.theta.act_level;
    temp_Coordinates.theta.mu = (temp_cali_Coordinates.theta.mu * cali_Coordinates.theta.act_level
                                 + temp_inject_Coordinates.theta.mu * inject_Coordinates.theta.act_level) /
                                temp_Coordinates.theta.act_level;
    wrap(temp_Coordinates.theta.mu, 2.0 * M_PI);
  }

  if (fabs(cali_Coordinates.x.mu - inject_Coordinates.x.mu) > box_size / 2.0) {
    if (cali_Coordinates.x.mu > inject_Coordinates.x.mu) {
      temp_cali_Coordinates.x.mu = cali_Coordinates.x.mu - box_size;
    } else if (cali_Coordinates.x.mu < inject_Coordinates.x.mu) {
      temp_inject_Coordinates.x.mu = inject_Coordinates.x.mu - box_size;
    }
  }
  temp_Coordinates.x.act_level = cali_Coordinates.x.act_level + inject_Coordinates.x.act_level;
  temp_Coordinates.x.mu = (temp_cali_Coordinates.x.mu * cali_Coordinates.x.act_level
                           + temp_inject_Coordinates.x.mu * inject_Coordinates.x.act_level) /
                          temp_Coordinates.x.act_level;
  wrap(temp_Coordinates.x.mu, box_size);

  if (fabs(cali_Coordinates.y.mu - inject_Coordinates.y.mu) > box_size / 2.0) {
    if (cali_Coordinates.y.mu > inject_Coordinates.y.mu) {
      temp_cali_Coordinates.y.mu = cali_Coordinates.y.mu - box_size;
    } else if (cali_Coordinates.y.mu < inject_Coordinates.y.mu) {
      temp_inject_Coordinates.y.mu = inject_Coordinates.y.mu - box_size;
    }
  }
  temp_Coordinates.y.act_level = cali_Coordinates.y.act_level + inject_Coordinates.y.act_level;
  temp_Coordinates.y.mu = (temp_cali_Coordinates.y.mu * cali_Coordinates.y.act_level
                           + temp_inject_Coordinates.y.mu * inject_Coordinates.y.act_level) /
                          temp_Coordinates.y.act_level;
  wrap(temp_Coordinates.y.mu, box_size);

  cali_Coordinates = temp_Coordinates;
}

void BayesCells::resetCali() {
  cali_Coordinates.theta.mu = curr_Coordinates.theta.mu;
  cali_Coordinates.theta.act_level = 0.0001;
  cali_Coordinates.x.mu = curr_Coordinates.x.mu;
  cali_Coordinates.x.act_level = 0.0001;
  cali_Coordinates.y.mu = curr_Coordinates.y.mu;
  cali_Coordinates.y.act_level = 0.0001;
}

void BayesCells::wrap(double &value, double period) {
  while (value >= period) { value = value - period; }
  while (value < 0) { value = value + period; }
}

double BayesCells::mod(double x, double y) {
  double x0 = x;

  while (x0 < 0.0)
    x0 += y;

  return fmod(x0, y);
}

double BayesCells::disPeriod(double angle1, double angle2, double t) {
  return mod(angle1 - angle2 + 0.5 * t, t) - 0.5 * t;
}





