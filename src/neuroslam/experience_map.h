
#ifndef _EXPERIENCE_MAP_H_
#define _EXPERIENCE_MAP_H_

#define _USE_MATH_DEFINES

#include "math.h"

#include <stdio.h>
#include <iostream>

#include <fstream>

#include <map>
#include <string>
#include <vector>
#include <deque>
#include <queue>

#include <float.h>
#include <ros/ros.h>
#include <boost/property_tree/ini_parser.hpp>
#include <yaml-cpp/yaml.h>

#include "utils/utils.h"
#include "Eigen/Core"

using boost::property_tree::ptree;

#include <boost/serialization/access.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/deque.hpp>


namespace neuroslam {

/*
 * The Link structure describes a link
 * between two experiences.
 */

    struct Link {
        double d;
        double heading_rad;
        double facing_rad;
        int exp_to_id;
        int exp_from_id;
        double delta_time_s;
        unsigned int id;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & d;
            ar & heading_rad;
            ar & facing_rad;
            ar & exp_to_id;
            ar & exp_from_id;
            ar & delta_time_s;
        }

    };

/*
 * The Experience structure describes
 * a node in the Experience_Map.
 */
    struct Experience {
        int id; // its own id

        double x_m, y_m, th_rad;
        int vt_id;

        std::vector<unsigned int> links_from; // links from this experience
        std::vector<unsigned int> links_to; // links to this experience


        // goal navigation
        double time_from_current_s;
        unsigned int goal_to_current, current_to_goal;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & id;
            ar & x_m & y_m & th_rad;
            ar & vt_id;
            ar & links_from & links_to;
            ar & time_from_current_s;
            ar & goal_to_current & current_to_goal;
        }

    };

    class ExperienceMapScene;

    class ExperienceMap {

    public:
        friend class ExperienceMapScene;

        ExperienceMap(YAML::Node &settings);

        ~ExperienceMap();

        // create a new experience for a given position
        int on_create_experience();

        // reduced graph
        bool on_create_link_reducedgraph(int exp_id_from, int exp_id_to, double rel_rad, bool loop);
        int on_create_experience_reducedgraph(unsigned int link_dest_id);

        Experience *get_experience(int id) {
            return &experiences[id];
        }

        Link *get_link(int id) {
            return &links[id];
        }

        // update the current position of the experience map
        // since the last experience
        void on_odo(double vtrans, double vrot, double time_diff_s);
        void on_odo(double &time_diff_s, std::vector<double>& delta_pos);

        // update the map by relaxing the graph
        void iterate();

        bool clustering_timing(bool loop);

        unsigned int get_links_index_from_id(unsigned int links_id);

        // change the current experience
        void on_set_experience(int new_exp_id, double rel_rad);

        int get_num_experiences() {
            return experiences.size();
        }

        int get_num_links() {
            return links.size();
        }

        int get_current_id() {
            return current_exp_id;
        }

        // functions for setting and handling goals.
        void add_goal(double x_m, double y_m);

        void add_goal(int id) {
            goal_list.push_back(id);
        }

        bool calculate_path_to_goal(double time_s);

        bool get_goal_waypoint();

        void clear_goal_list() {
            goal_list.clear();
        }

        int get_current_goal_id() {
            return (goal_list.size() == 0) ? -1 : (int) goal_list.front();
        }

        void delete_current_goal() {
            goal_list.pop_front();
        }

        bool get_goal_success() {
            return goal_success;
        }

        double get_subgoal_m() const;

        double get_subgoal_rad() const;

        const std::deque<int> &get_goals() const {
            return goal_list;
        }

        unsigned int get_goal_path_final_exp() {
            return goal_path_final_exp_id;
        }


        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & EXP_LOOPS;
            ar & EXP_CORRECTION;
            ar & MAX_GOALS;
            ar & EXP_INITIAL_EM_DEG;

            ar & experiences;
            ar & links;
            ar & goal_list;

            ar & current_exp_id & prev_exp_id;

            ar & accum_delta_facing;
            ar & accum_delta_x;
            ar & accum_delta_y;
            ar & accum_delta_time_s;

            ar & waypoint_exp_id;
            ar & goal_success;
            ar & goal_timeout_s;
            ar & goal_path_final_exp_id;

            ar & relative_rad;

        }

    private:
        friend class boost::serialization::access;

        ExperienceMap() {
            ;
        }

        // calculate distance between two experiences using djikstras algorithm
        // can be very slow for many experiences
        double dijkstra_distance_between_experiences(int id1, int id2);


        int EXP_LOOPS;
        double EXP_CORRECTION;
        unsigned int MAX_GOALS;
        double EXP_INITIAL_EM_DEG;

        std::vector<Experience> experiences;
        std::vector<Link> links;
        std::deque<int> goal_list;

//        ClusterLoops clusterloops;

        int current_exp_id, prev_exp_id;

        double accum_delta_facing;
        double accum_delta_x;
        double accum_delta_y;
        double accum_delta_time_s;

        double relative_rad;

        int waypoint_exp_id;
        bool goal_success;
        double goal_timeout_s;
        unsigned int goal_path_final_exp_id;

        // clustering loops
        bool newCluster;
        ros::Time t_loop_start;
        ros::Time t_loop_end;
        bool hasLoop; // present whether comparing with t_loop_end

        // reduced graph
        double LINK_D;
        double LINK_HEADING_RAD;
        std::vector<unsigned int> cluster_loop_link_id;

        unsigned int links_count; // stlucia

    public:
        bool OnceLoop;

        std::vector<int> all_ids; // 汇总所有current_exp_id;
        std::vector<double> timestamps; // all_ids对应的时间戳
    };

}

#endif // _EXPERIENCE_MAP_H_
