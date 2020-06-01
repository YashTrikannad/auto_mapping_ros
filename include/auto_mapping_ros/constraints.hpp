#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include "auto_mapping_ros/state.hpp"
#include <OsqpEigen/OsqpEigen.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include "auto_mapping_ros/visualizer.hpp"

// Stores and manages constraints for running MPC

class Constraints
{
public:
    Constraints(ros::NodeHandle &nh);
    virtual ~Constraints();

    // Getters and setters for various constraint parameters
    Eigen::VectorXd x_max();
    Eigen::VectorXd u_max();
    Eigen::VectorXd x_min();
    Eigen::VectorXd u_min();

    Eigen::MatrixXd slip_constraint();
    Eigen::MatrixXd slip_upper_bound();
    Eigen::MatrixXd slip_lower_bound();

    Eigen::VectorXd l1();
    Eigen::VectorXd l2();

    void set_x_max(Eigen::VectorXd xmax);
    void set_u_max(Eigen::VectorXd umax);
    void set_x_min(Eigen::VectorXd xmin);
    void set_u_min(Eigen::VectorXd umin);
    void set_state(State &state);
    void SetXLims(State x);

    // Finds half spaces using a conical model with a preset FOV
    void FindHalfSpaces(State &state,sensor_msgs::LaserScan &scan_msg_);

private:
    Eigen::VectorXd x_max_;
    Eigen::VectorXd u_max_;
    Eigen::VectorXd x_min_;
    Eigen::VectorXd u_min_;
    Eigen::VectorXd l1_;
    Eigen::VectorXd l2_;
    Eigen::MatrixXd slip_constraint_;
    Eigen::MatrixXd slip_upper_bound_;
    Eigen::MatrixXd slip_lower_bound_;
    State state_;
    float d_;
    float ftg_thresh_;
    std::pair <float,float> p1_;
    std::pair <float,float> p2_;
    std::pair <float,float> p_;
    sensor_msgs::LaserScan scan_msg_;
    ros::Publisher points_pub_;
    float umax_val_;
    float umin_val_;
    float divider_;
    float buffer_;
};

#endif