#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "transforms.hpp"
#include "input.hpp"
#include "state.hpp"
#include "transforms.hpp"
#include "input.hpp"
#include "occgrid.hpp"
#include <ros/package.h>

#include <fstream>
#include <limits>

using namespace std;

class Trajectory
{
public:
    Trajectory(float lookahead1, float lookahead2);
    ~Trajectory();
    // Converts trajectory of State objects to pairs of X,Y coordinates
    vector<pair<float,float>> GetPairPoints();
    // Loads CSV of CMA-ES trajectories
    bool ReadCMAES(string filename);
    // Checks if path is collision free till lookahead distance
    bool IsPathCollisionFree(const geometry_msgs::Pose pose, OccGrid &occ_grid);
    // Finds the closest point in the CMA-ES trajectory
    pair<pair<float,float>,int> FindClosest(pair<float,float> &globalPoint);
private:
    vector<State> waypoints_;
    float lookahead_1_;
    float lookahead_2_;
    // Returns distances of waypoints in CMA-ES trajectory relative to
    // car's position
    vector<float> GetWaypointDistances(const geometry_msgs::Pose &pose, bool inFront);
    // Helper function for isPathCollisionFree(..)
    int GetClosestIdx(const geometry_msgs::Pose pose, float lookahead);
};
#endif