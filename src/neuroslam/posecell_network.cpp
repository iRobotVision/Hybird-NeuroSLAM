
#include "posecell_network.h"

using namespace std;

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

namespace neuroslam {
PosecellNetwork::PosecellNetwork(YAML::Node &settings) {
  // after read topic_root,start to read gaussianslam_settings
  PC_DIM_XY = settings["pc_dim_xy"].as<int>(21);
  PC_DIM_TH = settings["pc_dim_th"].as<int>(36);

  VT_ACTIVE_DECAY = settings["vt_active_decay"].as<double>(1.0);
  EXP_DELTA_PC_THRESHOLD = settings["exp_delta_pc_threshold"].as<double>(2.0);

  PC_VT_RESTORE = settings["pc_vt_restore"].as<double>(0.05);

  SCALE_FACTOR = settings["scale_factor"].as<double>(10.0);
  BOX_SIZE = settings["box_size"].as<double>(15);
  gaussianCells.set_box_size(BOX_SIZE);  //设置grid cell网络尺寸

  CALI_MODE = settings["cali_mode"].as<int>(2);

  pre_x = 0;
  pre_y = 0;
  pre_th = 0;
  best_x = 0;
  best_y = 0;
  best_th = 0;

  current_exp = 0;
  current_vt = 0;
  prev_odom_vt = 0;

  check_cnt=0;

  min_delta_threshold = settings["min_delta_threshold"].as<float>(10.0); //urban
}

//free the memory pointer
template<class T>
void freeArray(T *&ptr) {
  if (ptr == NULL)
    return;

  delete[] ptr;

  ptr = NULL; //clear the pointer.
}

PosecellNetwork::~PosecellNetwork() {
  int i;
  for (i = 0; i < PC_DIM_TH; i++) {
    free(posecells[i]);
  }
  free(posecells);
  free(PC_AVG_XY_WRAP);
  free(PC_AVG_TH_WRAP);
  free(posecells_memory);

  freeArray(gridInjectUnitState);
  freeArray(spatialUnitState);
  freeArray(grid_estSpatialPhase);
  freeArray(HD_estSpatialPhase);
  freeArray(weight_m_grid_scale);
  freeArray(weight_m_HD_scale);
//	freeArray(actionArrayValue);
}

void PosecellNetwork::on_odom(double vtrans, double vrot, double time_diff_s, bool is_von_mises) {

  //----start path integration ----//
  gaussianCells.PathIntegration(vtrans / SCALE_FACTOR, vrot, time_diff_s,
                                is_von_mises);

//  // estimation
//  estPos[0] = gaussianCells.est_Coordinates.x.mu;
//  estPos[1] = gaussianCells.est_Coordinates.y.mu;
//  estPos[4] = gaussianCells.est_Coordinates.theta.mu;

  // return best value
  best_theta_rad = gaussianCells.est_Coordinates.theta.mu;
  best_th = gaussianCells.est_Coordinates.theta.mu / 2 * M_PI * static_cast<double>(PC_DIM_TH);
  best_x = gaussianCells.est_Coordinates.x.mu;
  best_y = gaussianCells.est_Coordinates.y.mu;


  // update pos transition
  delta_x = gaussianCells.est_Coordinates.x.mu - pre_x;
  delta_y = gaussianCells.est_Coordinates.y.mu - pre_y;
  delta_th = gaussianCells.est_Coordinates.theta.mu - pre_th;
  reformat_delta(delta_x, BOX_SIZE);
  reformat_delta(delta_y, BOX_SIZE);
  reformat_delta(delta_th, 2 * M_PI);
  delta_x = delta_x * SCALE_FACTOR;
  delta_y = delta_y * SCALE_FACTOR;

  pre_x = best_x;
  pre_y = best_y;
  pre_th = best_theta_rad;
}


double PosecellNetwork::get_delta_gc(double x, double y) {
  return sqrt(
      pow(get_min_delta(best_x, x, gaussianCells.get_box_size()), 2)
      + pow(get_min_delta(best_y, y, gaussianCells.get_box_size()), 2)
  );
}

double PosecellNetwork::get_min_delta(double d1, double d2, double max) {
  double absval = abs(d1 - d2);
  return min(absval, max - absval);
}

int PosecellNetwork::rot90_square(double **array, int dim, int rot) {
  double centre = (double) (dim - 1) / 2.0f;

  double a, b, c, d;

  double tmp_new, tmp_old;

  int i, j, quad, id, jd, is1, js1;

  if (rot < 0) {
    rot += 4;
  }

  switch (rot % 4) {
    case 0:
      return 1;
    case 1:
      a = 0;
      b = -1;
      c = 1;
      d = 0;
      break;
    case 2:
      a = -1;
      b = 0;
      c = 0;
      d = -1;
      break;
    case 3:
      a = 0;
      b = 1;
      c = -1;
      d = 0;
      break;
    default:
      return 1;
  }

  if (rot % 2 == 1) {
    for (j = 0; j < (int) (centre) + (1 - dim % 2); j++) {
      for (i = 0; i < (int) centre + 1; i++) {
        id = i;
        jd = j;
        tmp_old = array[j][i];
        for (quad = 0; quad < 4; quad++) {
          is1 = id;
          js1 = jd;
          id = (int) (a * ((float) (is1) - centre)
                      + b * ((float) (js1) - centre) + centre);
          jd = (int) (c * ((float) (is1) - centre)
                      + d * ((float) (js1) - centre) + centre);
          tmp_new = array[jd][id];
          array[jd][id] = tmp_old;
          tmp_old = tmp_new;
        }
      }
    }
  } else {
    rot90_square(array, dim, 1);
    rot90_square(array, dim, 1);
  }
  return true;
}

void PosecellNetwork::create_experience() {
  PosecellVisualTemplate *pcvt = &visual_templates[current_vt];
  if(pcvt == NULL){
    return;
  }
  experiences.resize(experiences.size() + 1);
  current_exp = experiences.size() - 1;
  PosecellExperience *exp = &experiences[current_exp];
  pcvt->pc_x = best_x;
  pcvt->pc_y = best_y;
//  pcvt->pc_th = best_th;
  pcvt->pc_th = best_theta_rad;
  exp->vt_id = current_vt;
  pcvt->exps.push_back(current_exp);
}

/// get gaussian action
PosecellNetwork::PosecellAction PosecellNetwork::get_action_opt() {
  PosecellAction action = NO_ACTION;

  PosecellVisualTemplate *pcvt = &visual_templates[current_vt];

  if (experiences.empty() || pcvt->exps.empty() || current_vt == prev_odom_vt) {
    create_experience();
    action = CREATE_NODE;
    return action;
  }
  prev_odom_vt = current_vt;

  PosecellExperience *experience = &experiences[current_exp];
  double delta_pc = get_delta_gc(experience->x_pc, experience->y_pc);

  if (delta_pc > EXP_DELTA_PC_THRESHOLD ||
      std::abs((int) current_vt - (int) prev_vt) > 50) // current_vt != prev_vt 代表场景有变化
  {
    if (delta_pc > EXP_DELTA_PC_THRESHOLD) {
      cout << RED << "delta_pc: " << delta_pc << RESET << endl;
    }

    //EXP_DELTA_PC_THRESHOLD means previous experience and current experience are not too close
    // go through all the exps associated with the current view and find the one with the closest delta_pc
    int min_delta_id = -1;
    auto min_delta = DBL_MAX;
    // find the closest experience in pose cell space
    for (unsigned int exp_id: pcvt->exps) {
      // make sure we aren't comparing to the current experience
      if (current_exp == exp_id)
        continue;
      experience = &experiences[exp_id];
      delta_pc = get_delta_gc(experience->x_pc, experience->y_pc);
      if (delta_pc < min_delta) {
        min_delta = delta_pc;
        min_delta_id = exp_id;
      }
    }

    cout << BLUE << "min_delta: " << min_delta << ", min_delta_id: " << min_delta_id << RESET << endl;
    // if an experience is closer than the thres create a link
    if (min_delta < min_delta_threshold && min_delta_id > -1) {
      if (current_exp != min_delta_id) {
//        current_exp = min_delta_id;
        link_dest_id = min_delta_id;
        create_experience();
      }
      action = CREATE_EDGE;
      ResetCells();
//      ROS_WARN("GREATE_EDGE!");
      cout << RED << "GREATE_EDGE!" << RESET << endl;
    }
    else if(min_delta < min_delta_threshold*2 && min_delta_id > -1){
      check_cnt ++;
      if(check_cnt >5){
        if (current_exp != min_delta_id) {
//        current_exp = min_delta_id;
          link_dest_id = min_delta_id;
          create_experience();
        }
        action = CREATE_EDGE;
        ResetCells();
        //        ROS_WARN("check_cnt: %d, GREATE_EDGE!", check_cnt);
        cout << RED << "check_cnt: " << check_cnt << ", GREATE_EDGE!" << RESET << endl;
        check_cnt = 0;
      }
    }
    else {
      create_experience();
      action = CREATE_NODE;
    }
  }
  return action;
}

void PosecellNetwork::ResetCells(){
  gaussianCells.ShowCurrentCoords();
  // curr = est
  gaussianCells.curr_Coordinates = gaussianCells.est_Coordinates;
  // reset cali
  gaussianCells.resetCali();

}

void PosecellNetwork::create_view_template() {
  PosecellVisualTemplate *pcvt;
  visual_templates.resize(visual_templates.size() + 1);
  pcvt = &visual_templates[visual_templates.size() - 1];
  pcvt->pc_x = best_x;
  pcvt->pc_y = best_y;
//  pcvt->pc_th = best_th;
  pcvt->pc_th = best_theta_rad;
  pcvt->decay = VT_ACTIVE_DECAY;
}

// view template optimized
void PosecellNetwork::on_view_template_opt(unsigned int vt, double *vt_relative_pose, bool is_von_mises) {
  //Correct the error - injection
  //1, the theta plane position of the grid - projection
  //2, grid network I is proportional to the current velocity
  static size_t pre_vt = 1;
  if (pre_vt < 100) { pre_vt = pre_vt + 1; }

  if (vt >= visual_templates.size()) {
    // For new view templates the id is associated with the centroid of
    // the current peak activity packet in the pose cell network.

    // must be a new template
    create_view_template();
    assert(vt == visual_templates.size() - 1);
  } else {
    // this prevents energy injected in recently created vt's
    if ((vt < (visual_templates.size() - 10)) && (pre_vt > 50)) { // default:10, set 10 no injection
      // the template must exist
      PosecellVisualTemplate *pcvt = &visual_templates[vt];
      // saturation process
      if (vt == current_vt) {
        pcvt->decay += VT_ACTIVE_DECAY;
      }
      double energy = (30.0 - exp(1.2 * pcvt->decay)) / 30.0;
      if (energy > 0) {

        if(CALI_MODE ==0){
          gaussianCells.Calibration_GaussianCells(pcvt->pc_th, pcvt->pc_x, pcvt->pc_y, is_von_mises);
        } else{
          double theta = pcvt->pc_th + vt_relative_pose[2];
          cout << "pcvt->pc_th: " << pcvt->pc_th *180/M_PI << ", vt_relative_theta: " << vt_relative_pose[2]*180/M_PI << ", theta: " << theta*180/M_PI << endl;
          double x_new, y_new;

          if(CALI_MODE == 1){
            x_new = pcvt->pc_x;
            y_new = pcvt->pc_y;
          }else if(CALI_MODE == 2){
            x_new = vt_relative_pose[0] * cos(theta) - vt_relative_pose[1] * sin(theta);
            y_new = vt_relative_pose[0] * sin(theta) + vt_relative_pose[1] * cos(theta);
            cout << "pcvt->(x,y): " << pcvt->pc_x << ", " << pcvt->pc_y << ", vt_relative_pose(x,y): "
                 << vt_relative_pose[0] << ", " << vt_relative_pose[1] << ", (x_new,y_new): " << x_new << ", " << y_new << endl;
            x_new = x_new + pcvt->pc_x;
            y_new = y_new + pcvt->pc_y;
          }else{
            ROS_ERROR_STREAM("CALI_MODE ERROR! (0:直接拉回原细胞位置 1:校准方向 2:校准方向和位置)");
          }
          gaussianCells.Calibration_GaussianCells(theta,x_new, y_new, is_von_mises);
        }
        gaussianCells.ShowCurrentCoords();
      }

    }
  }

  //#pragma omp parallel for num_threads(5)
  for (auto &visual_template: visual_templates) {
    visual_template.decay -= PC_VT_RESTORE;
    if (visual_template.decay < VT_ACTIVE_DECAY)
      visual_template.decay = VT_ACTIVE_DECAY;
  }
  prev_vt = current_vt;
  current_vt = vt;
}

void PosecellNetwork::get_curr_coordinates(double *Coordinates) {

  Coordinates[0] = gaussianCells.curr_Coordinates.theta.mu;
  Coordinates[1] = gaussianCells.curr_Coordinates.theta.act_level;
  Coordinates[2] = gaussianCells.curr_Coordinates.x.mu;
  Coordinates[3] = gaussianCells.curr_Coordinates.x.act_level;
  Coordinates[4] = gaussianCells.curr_Coordinates.y.mu;
  Coordinates[5] = gaussianCells.curr_Coordinates.y.act_level;

}

void PosecellNetwork::get_cali_coordinates(double *Coordinates) {

  Coordinates[0] = gaussianCells.cali_Coordinates.theta.mu;
  Coordinates[1] = gaussianCells.cali_Coordinates.theta.act_level;
  Coordinates[2] = gaussianCells.cali_Coordinates.x.mu;
  Coordinates[3] = gaussianCells.cali_Coordinates.x.act_level;
  Coordinates[4] = gaussianCells.cali_Coordinates.y.mu;
  Coordinates[5] = gaussianCells.cali_Coordinates.y.act_level;

}

void PosecellNetwork::get_inject_coordinates(double *Coordinates) {

  Coordinates[0] = gaussianCells.inject_Coordinates.theta.mu;
  Coordinates[1] = gaussianCells.inject_Coordinates.theta.act_level;
  Coordinates[2] = gaussianCells.inject_Coordinates.x.mu;
  Coordinates[3] = gaussianCells.inject_Coordinates.x.act_level;
  Coordinates[4] = gaussianCells.inject_Coordinates.y.mu;
  Coordinates[5] = gaussianCells.inject_Coordinates.y.act_level;

}

void PosecellNetwork::get_est_coordinates(double *Coordinates) {

  Coordinates[0] = gaussianCells.est_Coordinates.theta.mu;
  Coordinates[1] = gaussianCells.est_Coordinates.theta.act_level;
  Coordinates[2] = gaussianCells.est_Coordinates.x.mu;
  Coordinates[3] = gaussianCells.est_Coordinates.x.act_level;
  Coordinates[4] = gaussianCells.est_Coordinates.y.mu;
  Coordinates[5] = gaussianCells.est_Coordinates.y.act_level;

}

void PosecellNetwork::reformat_delta(double &delta, const double &max) {
  if (delta > 0) {
    delta = min(delta, max - delta);
  } else {
    delta = -min(-delta, max + delta);
  }
}

} // namespace gaussianslam

