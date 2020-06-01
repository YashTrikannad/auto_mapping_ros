#ifndef TRAJP_H
#define TRAJP_H
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "auto_mapping_ros/transforms.hpp"
#include "auto_mapping_ros/occgrid.hpp"
#include "auto_mapping_ros/visualizer.hpp"
#include "auto_mapping_ros/state.hpp"
#include "auto_mapping_ros/trajectory.hpp"

#include <experimental/filesystem>
#include <fstream>
#include <tf2/transform_datatypes.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

// Plans best local path from preset paths for the global paths 
// maximizing progress, and preventing collision and reversal

class TrajectoryPlanner
{
public:
    TrajectoryPlanner();
    // Initializes various parameters from yaml as well as private members
    TrajectoryPlanner(ros::NodeHandle &nh);
    virtual ~TrajectoryPlanner();
    // Reads the set of pre calculated trajectories
    void ReadTrajectories(string path);
    // Transforms a given point from car frame to world/map frame
    pair<float,float> CarPoint2World(float x, float y, const geometry_msgs::Pose &current_pose);
    // Transforms trajectory from car frame to occupancy grid frame
    void Trajectory2miniworld(const geometry_msgs::Pose &current_pose);
    // Transforms trajectory from car frame to world/map frame
    void Trajectory2world(const geometry_msgs::Pose &current_pose);
    vector<pair<float,float>> trajectories_world;
    // In any given situation, selects the best trajectory 
    // (close to global path + farthest from car + non-reversing) 
    int BestTrajectory(OccGrid &occ_grid,const geometry_msgs::Pose &current_pose, std::pair<float,float> &goal_to_track);
    // Visualization of local trajectory planned with the aimed goal point
    void Visualize();
    // Lane selection and setting of the best trajectory 
    void Update(const geometry_msgs::Pose &pose_msg, OccGrid &occ_grid, std::pair<float,float>& goal_to_track);
    vector<State> best_minipath();
    int best_trajectory_index();

private:
    int best_trajectory_index_;
    float close_weight;
    int max_horizon_;
    int num_traj_;
    float rev_threshold_;
    bool prev_rev_;
    State best_cmaes_point_;
    vector<State> best_minipath_;
    unsigned int current_lane_ = 0;
    std::vector<Trajectory> lanes_;
    double distance_from_switch_;
    double switch_distance_threshold_;
    // Selects lane based on opponent's position to prevent 
    // collision and promote over-taking
    void SelectLane(const geometry_msgs::Pose pose, OccGrid &occ_grid);
    int horizon_;
    vector<pair<float,float>> trajectories_;
    geometry_msgs::Pose last_pose_;

    ros::Publisher traj_pub_;
    std::vector<geometry_msgs::Point> points_;
    std::vector<std_msgs::ColorRGBA> colors_;
    bool cmaes_point_pushed_ = false;
    bool cmaes_pushed_ = false;
    bool best_traj_pushed_ = false;

};

#endif
