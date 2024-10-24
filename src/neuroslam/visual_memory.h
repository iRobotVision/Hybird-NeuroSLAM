#ifndef _VISUAL_MEMORY_H_
#define _VISUAL_MEMORY_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

using namespace std;

#include "utils/utils.h"

#include <cmath>
#include <yaml-cpp/yaml.h>

#include "DBoW3/DBoW3.h"
#include <opencv2/opencv.hpp>

using namespace cv;


namespace neuroslam {

class VisualMemory {
public:
  VisualMemory(YAML::Node &settings);

  VisualMemory(YAML::Node &settings, YAML::Node &camera_settings);

  void on_image(cv::Mat &image, double timestamp);

  void cmp_relative_pose(int source_id, int target_id);

  int get_current_vt() const {
    return current_vt;
  }

  double* get_relative_pose(){
    return vt_relative_pose;
  }

  void reset_relative_pose(){
    vt_relative_pose[0] = 0;
    vt_relative_pose[1] = 0;
    vt_relative_pose[2] = 0;
  }

  std::mutex m_buf;
  queue<pair<double, int>> img_buf;

private:
  Ptr<Feature2D> detector;
  Ptr<DescriptorMatcher> matcher;
  Mat K;

  DBoW3::Vocabulary *vocab;
  DBoW3::Database db;

  vector<int> template_ids;


  int IMAGE_WIDTH;
  int IMAGE_HEIGHT;

  int current_vt;
  int prev_vt;
  int vt_num;
  int find_loop_cnt;
  double vt_relative_pose[3]; //[rad, x, y]

  string vocab_file;
  int min_loop_frame_dis;
  int loop_cnt_thres;
  double loop_score_thres;


  map<int, vector<KeyPoint>> keypoints_;
  map<int, Mat> descriptors_;
};

} // namespace gaussianslam

#endif // _VISUAL_MEMORY_H_
