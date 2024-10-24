
#include "experience_map.h"

using namespace std;

namespace neuroslam {

ExperienceMap::ExperienceMap(YAML::Node &settings) {

  EXP_CORRECTION = settings["exp_correction"].as<double>(0.5);
  EXP_LOOPS = settings["exp_loops"].as<int>(10);
  EXP_INITIAL_EM_DEG = settings["exp_initial_em_deg"].as<double>(0);

  MAX_GOALS = 10;

  experiences.reserve(10000);
  links.reserve(10000);


  current_exp_id = 0;
  prev_exp_id = 0;
  waypoint_exp_id = -1;
  goal_timeout_s = 0;
  goal_success = false;

  accum_delta_facing = EXP_INITIAL_EM_DEG * M_PI / 180;
  accum_delta_x = 0;
  accum_delta_y = 0;
  accum_delta_time_s = 0;

  relative_rad = 0;

  // cluster loops
  newCluster = true;

  // reduced graph
//  LINK_D = 0.1; //irat
//  LINK_D = 5.0; //stlucia,10,5
  LINK_D = 0.0; //kitti,

  LINK_HEADING_RAD = M_PI / 180 * 5;
//    link_id_count = 0;

  links_count = 0;//stlucia
}

ExperienceMap::~ExperienceMap() {
  links.clear();
  experiences.clear();
}

// create a new experience for a given position
int ExperienceMap::on_create_experience() {
  experiences.resize(experiences.size() + 1);
  Experience *new_exp = &(*(experiences.end() - 1));

  if (experiences.empty()) {
    new_exp->x_m = 0;
    new_exp->y_m = 0;
    new_exp->th_rad = 0;
  } else {
    new_exp->x_m = experiences[current_exp_id].x_m + accum_delta_x;
    new_exp->y_m = experiences[current_exp_id].y_m + accum_delta_y;
    new_exp->th_rad = clip_rad_180(accum_delta_facing);
  }
  new_exp->id = experiences.size() - 1;

  if (experiences.size() != 1)
    on_create_link_reducedgraph(current_exp_id, experiences.size() - 1, 0, false); // no loop closure

  return experiences.size() - 1;
}

// create a new experience for a given position
int ExperienceMap::on_create_experience_reducedgraph(unsigned int link_dest_id) {
  experiences.resize(experiences.size() + 1);
  Experience *new_exp = &(*(experiences.end() - 1));

  if (experiences.empty()) {
    new_exp->x_m = 0;
    new_exp->y_m = 0;
    new_exp->th_rad = 0;
  } else {
    new_exp->x_m = experiences[current_exp_id].x_m + accum_delta_x;
    new_exp->y_m = experiences[current_exp_id].y_m + accum_delta_y;

    if (experiences[link_dest_id].links_to.empty()) {
      new_exp->th_rad = clip_rad_180(accum_delta_facing);
    } else {
      new_exp->th_rad = experiences[link_dest_id].th_rad;  // 纠正朝向
    }
//	ROS_INFO_STREAM("on_create_experience_reducedgraph th_rad:"<<new_exp->th_rad);
  }
  new_exp->id = experiences.size() - 1;

  if (experiences.size() != 1)
    on_create_link_reducedgraph(current_exp_id, experiences.size() - 1, 0, false); // no loop closure

  return experiences.size() - 1;
}

bool ExperienceMap::on_create_link_reducedgraph(int exp_id_from, int exp_id_to, double rel_rad, bool loop) {
  Experience *current_exp = &experiences[exp_id_from];
  // check if the link already exists
  for (unsigned int link_from: experiences[exp_id_from].links_from) {
//        if (links[link_from].exp_to_id == exp_id_to || links[link_from].exp_to_id == exp_id_from)
    if (links[link_from].exp_to_id == exp_id_to)
      return false;
  }
  for (unsigned int link_to: experiences[exp_id_to].links_to) {
    if (links[link_to].exp_from_id == exp_id_from)
      return false;
  }

  if (!loop && links.size() > 5000) //sequential links
  {
    links_count = 5000;
    // reduce or not
    if (current_exp->links_from.empty()) {

      Link *current_link = &(*(links.end() - 1));

      double new_link_d = sqrt(accum_delta_x * accum_delta_x + accum_delta_y * accum_delta_y);
      double new_link_heading_rad = get_signed_delta_rad(current_exp->th_rad,
                                                         atan2(accum_delta_y, accum_delta_x));

//			ROS_WARN_STREAM("current_link->d:"<<current_link->d<<",new_link_d:"<<new_link_d);
//			ROS_WARN_STREAM("new_link_heading_rad:"<<new_link_heading_rad<<",LINK_HEADING_RAD:"<<LINK_HEADING_RAD);

      if ((current_link->d < LINK_D) && (new_link_d < LINK_D)) {
        if (fabs(new_link_heading_rad) < LINK_HEADING_RAD) {
          if (!current_exp->links_to.empty()) {
//						  ROS_WARN_STREAM("seq_reduce true...");
//						  ROS_INFO_STREAM("current_exp id:"<<current_exp->id);

//					      ROS_INFO_STREAM("current_exp->links_to.back():"<<current_exp->links_to.back());
//					      ROS_INFO_STREAM("get_links_index_from_id:"<<get_links_index_from_id(current_exp->links_to.back()));

            unsigned int link_index_from_id = get_links_index_from_id(current_exp->links_to.back());
            int new_exp_id_from = links[link_index_from_id].exp_from_id;
//						  int new_exp_id_from = links[current_exp->links_to.back()].exp_from_id;

            experiences[new_exp_id_from].links_from.pop_back(); // cut old links from exp
            current_exp->links_to.pop_back(); //cut old links to exp

            // new links
            Link new_link;

            new_link.exp_to_id = exp_id_to;
            new_link.exp_from_id = new_exp_id_from;
            new_link.heading_rad = clip_rad_180(current_link->heading_rad + new_link_heading_rad);

//						  new_link.d = sqrt(pow(current_link->d + new_link_d*cos(fabs(new_link_heading_rad)),2)
//								  + pow(new_link_d*sin(fabs(new_link_heading_rad)),2)
//						  	  	  );
            new_link.d = sqrt(pow(current_link->d + new_link_d * cos(new_link_heading_rad), 2)
                              + pow(new_link_d * sin(new_link_heading_rad), 2)
            );

            new_link.facing_rad = clip_rad_180(
                current_link->facing_rad + get_signed_delta_rad(current_exp->th_rad, clip_rad_180(
                    accum_delta_facing + rel_rad))
            );
            new_link.delta_time_s = current_link->delta_time_s + accum_delta_time_s;
            // add loop ref
//                        new_link.loop_ref = loop_ref;
//                        new_link.loop_var = loop_ref; // initial value is ref
            new_link.id = current_link->id;

//						  ROS_INFO_STREAM("new_link.id:"<<new_link.id);

            links.pop_back();
            links.push_back(new_link);
            // add this link to the 'to exp' so we can go backwards through the em
            experiences[new_exp_id_from].links_from.push_back(new_link.id);
            experiences[exp_id_to].links_to.push_back(new_link.id);

            ROS_INFO_STREAM("sequential reduce has been executed...");
            ROS_INFO_STREAM("Original exp_id_from:" << exp_id_from << " to exp_id_to:" << exp_id_to);
            ROS_INFO_STREAM(
                "Replace new_exp_id_from:" << new_exp_id_from << " to exp_id_to:" << exp_id_to);
            ROS_INFO_STREAM("new_link.d:" << new_link.d);
            return true;
          }
        }
      }
    }
  }

  links.resize(links.size() + 1);
  Link *new_link = &(*(links.end() - 1));

  new_link->exp_to_id = exp_id_to;
  new_link->exp_from_id = exp_id_from;
  if (loop) {
    new_link->d = 0;
    new_link->heading_rad = 0;
    new_link->facing_rad = rel_rad;
    new_link->delta_time_s = 0;
    cluster_loop_link_id.push_back(links.size() - 1);
  } else {
    new_link->d = sqrt(accum_delta_x * accum_delta_x + accum_delta_y * accum_delta_y);
    new_link->heading_rad = get_signed_delta_rad(current_exp->th_rad, atan2(accum_delta_y, accum_delta_x));
    new_link->facing_rad = get_signed_delta_rad(current_exp->th_rad, clip_rad_180(accum_delta_facing + rel_rad));
    new_link->delta_time_s = accum_delta_time_s;
  }
  new_link->id = links.size() - 1;

  // add this link to the 'to exp' so we can go backwards through the em
  experiences[exp_id_from].links_from.push_back(new_link->id);
  experiences[exp_id_to].links_to.push_back(new_link->id);

  //----count for clustering-----//
  clustering_timing(loop);

  return true;
}

unsigned int ExperienceMap::get_links_index_from_id(unsigned int links_id) {
  unsigned int links_index;

  std::vector<Link>::iterator it =
      std::find_if(links.begin(), links.end(), boost::bind(&Link::id, _1) == links_id);
  links_index = it - links.begin();

  return links_index;
}


bool ExperienceMap::clustering_timing(bool loop) {
  // for cluster
  if (loop) // if loop closure
  {
    t_loop_end = ros::Time::now(); // stamp loop links time
    hasLoop = true;

    if (newCluster) {
      newCluster = false;
      t_loop_start = ros::Time::now();
    }
  }
  double t_links_d = (ros::Time::now() - t_loop_end).toSec();
  double t_loop_d = (ros::Time::now() - t_loop_start).toSec();

  if (hasLoop) { // has not been looped
    if ((t_links_d > 5.0) || (t_loop_d > 30.0)) //kitti, duration between two loop links
//		if((t_links_d > 5.0) || ( t_loop_d > 200.0) ) //stlucia, duration between two loop links
//		if((t_links_d > 2.0) || ( t_loop_d > 100.0) ) //irat,duration between two loop links
    {
      newCluster = true;
      hasLoop = false;
      OnceLoop = true;
    }
  }
  return true;
}

/// update the current position of the experience map based on odometry
void ExperienceMap::on_odo(double vtrans, double vrot, double time_diff_s) {
  vtrans = vtrans * time_diff_s;
  vrot = vrot * time_diff_s;
  accum_delta_facing = clip_rad_180(accum_delta_facing + vrot);
  accum_delta_x = accum_delta_x + vtrans * cos(accum_delta_facing);
  accum_delta_y = accum_delta_y + vtrans * sin(accum_delta_facing);
  accum_delta_time_s += time_diff_s;
}

/// update the current position of the experience map based on the cells phase dynamic
void ExperienceMap::on_odo(double &time_diff_s, std::vector<double> &delta_pos) {
  accum_delta_x = accum_delta_x + delta_pos[0];
  accum_delta_y = accum_delta_y + delta_pos[1];
  accum_delta_facing = clip_rad_180(accum_delta_facing + delta_pos[2]);
  accum_delta_time_s += time_diff_s;
}

// iterate the experience map.
// Perform a graph relaxing algorithm to allow the map to partially converge.
void ExperienceMap::iterate() {
  Experience *link_from, *link_to;
  Link *link;
  double lx, ly, df;

  for (int i = 0; i < EXP_LOOPS; i++) {
    for (auto &experience: experiences) {
      link_from = &experience;
      for (unsigned int link_id = 0; link_id < link_from->links_from.size(); link_id++) {
        //% experience 0 has a link to experience 1
        link = &links[link_from->links_from[link_id]];
        link_to = &experiences[link->exp_to_id];

        //% work out where e0 thinks e1 (x,y) should be based on the stored
        //% link information
        lx = link_from->x_m + link->d * cos(link_from->th_rad + link->heading_rad);
        ly = link_from->y_m + link->d * sin(link_from->th_rad + link->heading_rad);

        //% correct e0 and e1 (x,y) by equal but opposite amounts
        //% a 0.5 correction parameter means that e0 and e1 will be fully
        //% corrected based on e0's link information
        link_from->x_m += (link_to->x_m - lx) * EXP_CORRECTION; // EXP_CORRECTION=0.5
        link_from->y_m += (link_to->y_m - ly) * EXP_CORRECTION;
        link_to->x_m -= (link_to->x_m - lx) * EXP_CORRECTION;
        link_to->y_m -= (link_to->y_m - ly) * EXP_CORRECTION;

        //% determine the angle between where e0 thinks e1's facing
        //% should be based on the link information
        df = get_signed_delta_rad(link_from->th_rad + link->facing_rad, link_to->th_rad);

        //% correct e0 and e1 facing by equal but opposite amounts
        //% a 0.5 correction parameter means that e0 and e1 will be fully
        //% corrected based on e0's link information
        link_from->th_rad = clip_rad_180(link_from->th_rad + df * EXP_CORRECTION);
        link_to->th_rad = clip_rad_180(link_to->th_rad - df * EXP_CORRECTION);
      }
    }
  }
}


// change the current experience
void ExperienceMap::on_set_experience(int new_exp_id, double rel_rad) {
  if (new_exp_id > experiences.size() - 1)
    return;

  if (new_exp_id == current_exp_id) {
    return;
  }

  prev_exp_id = current_exp_id;
  current_exp_id = new_exp_id;
  accum_delta_x = 0;
  accum_delta_y = 0;
  accum_delta_facing = clip_rad_180(experiences[current_exp_id].th_rad + rel_rad);

  relative_rad = rel_rad;
}

struct compare {
  bool operator()(const Experience *exp1, const Experience *exp2) {
    return exp1->time_from_current_s > exp2->time_from_current_s;
  }
};

} // namespace gaussianslam

