#ifndef _GAUSSIANCELLS_H_
#define _GAUSSIANCELLS_H_

#include "utils/utils.h"


// Define parameters for the network
typedef struct {
  double mu;        //mu = mean
  //for Bayesian distribution: act_level = 1/sigma^2;
  //for Von Mises distribution: act_level = kappa;
  double act_level;
} Cells;

typedef struct {
  Cells x;
  Cells y;
  Cells theta;
} Coordinates;

class BayesCells {

public:
  BayesCells();

  ~BayesCells();

  void freeGaussianCells();

  /**
   * @param vtrans: translational velocity
   * @param vrot: rotational velocity
   * @param timeInterval: time interval
   * @param is_von_mises: whether to use von Mises distribution for HD cells
   * */
  void PathIntegration(double vtrans, double vrot, double timeInterval, bool is_von_mises = false);

  void Calibration_GaussianCells(double theta, double x, double y, bool is_von_mises= false);

  void ShowCurrentCoords();

  void resetCali();

  double get_box_size() const {
    return box_size;
  }

  Coordinates curr_Coordinates;
  Coordinates cali_Coordinates;
  Coordinates est_Coordinates;
  Coordinates inject_Coordinates;

  void set_box_size(double &size) {
    box_size = size;
  }

private:
  /// 路径积分
  void VelocityInput(double vtrans, double vrot, double timeInterval);

  /// mutual and global inhibition
  void IntrinsicDynamic(bool is_von_mises= false);

  /// estimate from curr and cali coordinates;
  void Estimation(bool is_von_mises= false);


  void wrap(double &value, double period);

  double act_level_sum_HD;
  double act_level_sum_Pos;

  double threshold;

  double box_size;

  double mod(double x, double y);

  double disAngle(double angle1, double angle2);

  double disPeriod(double angle1, double angle2, double t);

};

#endif 
