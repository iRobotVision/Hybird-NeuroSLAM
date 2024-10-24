#include "visual_memory.h"

// Colors
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"

namespace neuroslam {
/**
 * @brief Constructor for LocalViewMatch class
 * */
VisualMemory::VisualMemory(YAML::Node &settings) {
  vocab_file = settings["vocab_file"].as<string>(
      "/home/daybeha/Documents/github/Hybird-NeuroSLAM/to_release_ws/src/ratslam_ros/src/neuroslam/orbvoc.dbow3");
  min_loop_frame_dis = settings["min_loop_frame_dis"].as<int>(300); // kitti 50
  loop_cnt_thres = settings["loop_cnt_thres"].as<int>(1);
  loop_score_thres = settings["loop_score_thres"].as<double>(0.05);
  cout << "min_loop_frame_dis: " << min_loop_frame_dis << endl; // "min_loop_frame_dis: 300
  cout << "loop_cnt_thres: " << loop_cnt_thres << endl;
  cout << "loop_score_thres: " << loop_score_thres << endl;

  current_vt = 0;
  prev_vt = -1;
  vt_num = 0;
  find_loop_cnt = 0;

  detector = ORB::create();
  vocab = new DBoW3::Vocabulary(vocab_file);
  db.setVocabulary(*vocab, false, 0);
}


/**
 * @brief Constructor for LocalViewMatch class
 * */
VisualMemory::VisualMemory(YAML::Node &settings, YAML::Node &camera_settings) {
  vocab_file = settings["vocab_file"].as<string>(
      "/home/daybeha/Documents/github/Hybird-NeuroSLAM/to_release_ws/src/ratslam_ros/src/neuroslam/orbvoc.dbow3");
  min_loop_frame_dis = settings["min_loop_frame_dis"].as<int>(300); // kitti 50
  loop_cnt_thres = settings["loop_cnt_thres"].as<int>(1);
  loop_score_thres = settings["loop_score_thres"].as<double>(0.05);

  cout << "min_loop_frame_dis: " << min_loop_frame_dis << endl; // "min_loop_frame_dis: 300
  cout << "loop_cnt_thres: " << loop_cnt_thres << endl;
  cout << "loop_score_thres: " << loop_score_thres << endl;

  current_vt = 0;
  prev_vt = -1;
  vt_num = 0;
  find_loop_cnt = 0;

  detector = ORB::create();
  vocab = new DBoW3::Vocabulary(vocab_file);
  db.setVocabulary(*vocab, false, 0);


  /// Load camera settings
  matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
  K = (Mat_<double>(3, 3) << camera_settings["fx"].as<double>(816.402214740649), 0, camera_settings["cx"].as<double>(608.826584275826),
      0, camera_settings["fy"].as<double>(817.383885628095), camera_settings["cy"].as<double>(266.688656524394), 0, 0, 1);

}


void VisualMemory::cmp_relative_pose(int source_id, int target_id) {
  vector<KeyPoint> keypoints_src = keypoints_[source_id];
  vector<KeyPoint> keypoints_tgt = keypoints_[target_id];
  Mat descriptor_src = descriptors_.at(source_id);
  Mat descriptor_tgt = descriptors_.at(target_id);

  if (descriptor_src.type() != CV_32F) {
    descriptor_src.convertTo(descriptor_src, CV_32F);
    descriptor_tgt.convertTo(descriptor_tgt, CV_32F);
  }

  vector<vector<DMatch>> knn_matches;
  matcher->knnMatch(descriptor_src, descriptor_tgt, knn_matches, 2);

  const float ratio_thresh = 0.7f;
  vector<DMatch> good_matches;
  for (size_t i = 0; i < knn_matches.size(); i++) {
    if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
      good_matches.push_back(knn_matches[i][0]);
    }
  }

  if(good_matches.size() < 10) {
    cout << RED <<  "good_matches.size() < 10" << RESET << endl;
    reset_relative_pose();
    return;
  }

  vector<Point2f> pts1, pts2;
  for (size_t i = 0; i < good_matches.size(); i++) {
    pts1.push_back(keypoints_src[good_matches[i].queryIdx].pt);
    pts2.push_back(keypoints_tgt[good_matches[i].trainIdx].pt);
  }

  Mat F = findFundamentalMat(pts1, pts2, FM_RANSAC);

  if(!F.empty()){
    Mat E = K.t() * F * K;

    Mat R, t;
    recoverPose(E, pts1, pts2, K, R, t);
    cv::Vec3f rpy = rotationMatrixToEulerAngles(R);

    vt_relative_pose[0] = t.at<double>(2);
    vt_relative_pose[1] = -t.at<double>(0);
    vt_relative_pose[2] = rpy[2];
  }else{
    reset_relative_pose();
  }
}

void VisualMemory::on_image(cv::Mat &image, double timestamp) {
  if (image.empty()) return;

  IMAGE_WIDTH = image.cols;
  IMAGE_HEIGHT = image.rows;


  cv::Mat img_in;
  cv::resize(image, img_in, cv::Size(int(IMAGE_WIDTH / 2), int(IMAGE_HEIGHT / 2)));

  vector<KeyPoint> keypoints;
  Mat descriptor;
  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  detector->detectAndCompute(img_in, Mat(), keypoints, descriptor);
  keypoints_.insert(pair<int, vector<KeyPoint>>(vt_num, keypoints));
  descriptors_.insert(pair<int, Mat>(vt_num, descriptor));

  auto diff = chrono::duration_cast<std::chrono::milliseconds>(chrono::steady_clock::now() - begin);
//        cout << "extract feature time " << diff.count() << "ms" << endl;

  DBoW3::QueryResults ret;
  begin = chrono::steady_clock::now();
  if (template_ids.size() > min_loop_frame_dis) {
    db.query(descriptor, ret, 4, template_ids.size() - min_loop_frame_dis);  // max result=4, kitti
  }

  bool find_loop = false;
  if (!ret.empty() && ret[0].Score > loop_score_thres)
  {
    for (unsigned int i = 1; i < ret.size(); i++) {
      if (ret[i].Score > 0.015 && abs(ret[0].Id - ret[i].Id) < 4)
      {
        find_loop_cnt++;
        if (find_loop_cnt > loop_cnt_thres)
          find_loop = true;
        break;
      }
    }
  }

  if (find_loop) {
    // 找到成功匹配的最早模板id
    int min_index = -1;
    for (auto &r: ret) {
      if (min_index == -1 || (r.Id < min_index && r.Score > 0.015))
        min_index = r.Id;
    }
//            set_current_vt((int) template_ids[min_index]);
    current_vt = template_ids[min_index];
    std::cout << GREEN << "View Template " << get_current_vt() << " matches with " << vt_num << " !" << RESET << std::endl;

    cmp_relative_pose(vt_num, get_current_vt());

  } else { // 匹配失败
    vt_relative_pose[0] = 0;
    vt_relative_pose[1] = 0;
    vt_relative_pose[2] = 0;

    if (find_loop_cnt > loop_cnt_thres) // 非关键帧
      find_loop_cnt = 0;
    current_vt = vt_num;
    vt_num++;
  }
  db.add(descriptor);

  template_ids.push_back(current_vt);

  m_buf.lock();
  img_buf.emplace(timestamp, current_vt);
  m_buf.unlock();
}
}
