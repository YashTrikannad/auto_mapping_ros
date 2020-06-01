#ifndef OCC_H
#define OCC_H
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

// Manages occupany grid using a boolean Eigen Matrix
// Includes helper functions for converting coordinates and checking collisions

class OccGrid
{
public:
    OccGrid();
    OccGrid(ros::NodeHandle &nh);
    virtual ~OccGrid();
    // Coordinate transform functions
    // First element of pair is x and second element is y
    std::pair<int, int> WorldToOccupancy(std::pair<float, float> point);
    std::pair<int, int> WorldToOccupancy(float x, float y);
    std::pair<float,float> OccupancyToWorld(int row, int col);
    std::pair<float,float> OccupancyToWorld(std::pair<int,int> grid_point);
    std::pair<float, float> PolarToCartesian(float range, float angle);

    // Marks cells as occupied in occupancy grid given the current position
    // of the vehicle and the scan message
    void FillOccGrid(const geometry_msgs::Pose &pose_msg, const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    // Overloaded sanity check for checking if the queried point is in the
    // bounds of the grid
    bool InGrid(int col, int row);
    bool InGrid(std::pair<int, int> grid_point);
    bool CartesianInGrid(float x, float y);
    bool CartesianInGrid(std::pair<float, float> cart_point);
    // Overloaded function to check if the line joining a pair of points in
    // the grid intersects with an obstacles. Implemented using Bresenham's
    // 2D line drawing algorithm using only integer operations
    bool CheckCollision(float x1, float y1, float x2, float y2);
    bool CheckCollision(std::pair<float, float> first_point, std::pair<float, float> second_point);
    // Draws pretty grid blocks
    void Visualize();
    int size();
private:
    int size_;
    float discrete_;
    int grid_blocks_;
    float dilation_;
    std::pair<float, float> occ_offset_;
    Eigen::MatrixXf grid_;
    ros::Publisher occ_pub_;
};
#endif