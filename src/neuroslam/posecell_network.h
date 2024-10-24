
#ifndef _POSE_CELL_NETWORK_HPP
#define _POSE_CELL_NETWORK_HPP

#define _USE_MATH_DEFINES

#include "math.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <ros/ros.h>
#include <math.h>
#include <float.h>

#include <vector>

#include <boost/property_tree/ini_parser.hpp>

using boost::property_tree::ptree;

typedef double Posecell;

#include <boost/serialization/access.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_member.hpp>

#include <yaml-cpp/yaml.h>

//#include "neuroslam/grid_src/src/NetworkUtility.h"
#include "utils/utils.h"

//#include "neuroslam/grid_src/period/Session.h"
//#include "neuroslam/grid_src/src/GridGeneration.h"
#include "BayesCells.h"

//define global variables - began
#ifndef SEEDLIST
unsigned int seedList[120] = {7922, 9594, 6557, 357, 8491, 9339, 6787, 7577,
                              7431, 3922, 6554, 1711, 7060, 318, 2769, 461, 971, 8234, 6948, 3170,
                              9502, 344, 4387, 3815, 7655, 7952, 1868, 4897, 4455, 6463, 7093, 7546,
                              2760, 6797, 6550, 1626, 1189, 4983, 9597, 3403, 5852, 2238, 7512, 2550,
                              5059, 6990, 8909, 9592, 5472, 1386, 1492, 2575, 8407, 2542, 8142, 2435,
                              9292, 3499, 1965, 2510, 6160, 4732, 3516, 8308, 5852, 5497, 9171, 2858,
                              7572, 7537, 3804, 5678, 758, 539, 5307, 7791, 9340, 1299, 5688, 4693,
                              119, 3371, 1621, 7942, 3112, 5285, 1656, 6019, 2629, 6540, 6892, 7481,
                              4505, 838, 2289, 9133, 1523, 8258, 5383, 9961, 781, 4426, 1066, 9618,
                              46, 7749, 8173, 8686, 844, 3997, 2598, 8000, 4314, 9106, 1818, 2638,
                              1455, 1360, 8692, 5797};

FILE *logFile = NULL; //define the output log file
FILE *debugFile = NULL; //define the debug info file
//define global variables -end
#endif

namespace neuroslam {
struct PosecellVisualTemplate {
  unsigned int id;
  double pc_x, pc_y, pc_th;
  double decay;
  std::vector<unsigned int> exps;

  template<typename Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar & id;
    ar & pc_x & pc_y & pc_th;
    ar & decay;
  }

};

struct PosecellExperience {
  double x_pc, y_pc, th_pc;
  int vt_id;
};

class PosecellNetwork {

public:
  enum PosecellAction {
    NO_ACTION = 0, CREATE_NODE, CREATE_EDGE, SET_NODE
  };

  PosecellNetwork(YAML::Node &settings);

  ~PosecellNetwork();

  void on_odom(double vtrans, double vrot, double time_diff_s, bool is_von_mises = false);

  void on_view_template_opt(unsigned int vt, double *vt_relative_pose = NULL, bool is_von_mises = false);

  PosecellAction get_action_opt();

  void ResetCells();

  void get_curr_coordinates(double *Coordinates);

  void get_cali_coordinates(double *Coordinates);

  void get_inject_coordinates(double *Coordinates);

  void get_est_coordinates(double *Coordinates);

  std::vector<double> get_delta_pos() {
    std::vector<double> delta_pos(3);
    delta_pos[0] = delta_x;
    delta_pos[1] = delta_y;
    delta_pos[2] = delta_th;
    return delta_pos;
  }

  double get_delta_gc(double x, double y);

  unsigned int get_current_exp_id() {
    return current_exp;
  }

  double get_relative_rad() {
    return vt_delta_pc_th;
  }


  template<typename Archive>
  void save(Archive &ar, const unsigned int version) const {
    ar & PC_DIM_XY;
    ar & PC_DIM_TH;

    ar & VT_ACTIVE_DECAY;
    ar & PC_VT_RESTORE;

    ar & best_x;
    ar & best_y;
    ar & best_th;

    int i, j, k;
    for (k = 0; k < PC_DIM_TH; k++)
      for (j = 0; j < PC_DIM_XY; j++)
        for (i = 0; i < PC_DIM_XY; i++)
          ar & posecells[k][j][i];

  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()
private  :
  friend class boost::serialization::access;

  void create_experience();

  void create_view_template();

  PosecellNetwork(const PosecellNetwork &other);

  const PosecellNetwork &operator=(const PosecellNetwork &other);

  int rot90_square(double **array, int dim, int rot);

  double get_min_delta(double d1, double d2, double max);

  void reformat_delta(double &delta, const double &max);

  int PC_DIM_XY;
  int PC_DIM_TH;

  double VT_ACTIVE_DECAY;
  double PC_VT_RESTORE;

  double EXP_DELTA_PC_THRESHOLD;

  double BOX_SIZE; // Grid Cell网络的大小：BOX_SIZE * BOX_SIZE
  double SCALE_FACTOR; // trans的缩放因子，决定每个Grid Cell的感受野

  int CALI_MODE;

  double best_x;
  double best_y;
  double best_th;
  double best_theta_rad;

  double vt_delta_pc_th;


  Posecell ***posecells;
  Posecell *posecells_memory;

  int *PC_AVG_XY_WRAP;
  int *PC_AVG_TH_WRAP;

  std::vector<PosecellVisualTemplate> visual_templates;
  std::vector<PosecellExperience> experiences;

  unsigned int current_vt, prev_vt;
  unsigned int prev_odom_vt;
  unsigned int current_exp, prev_exp;

  double pre_x, pre_y, pre_th;
  double delta_th, delta_x, delta_y;

  int check_cnt;
  //-----add a member class-grid network session
public:
  float *weight_m_grid_scale;
  float *weight_m_HD_scale;
  float min_delta_threshold;
  int link_dest_id;  // the destination exp id of the link

  //gaussian cells
  BayesCells gaussianCells;


private:
  float *grid_estSpatialPhase;
  float *HD_estSpatialPhase;
  float *spatialUnitState;
  float *gridInjectUnitState;
  std::string topic_root;

};

}

#endif // _POSE_CELL_NETWORK_HPP
