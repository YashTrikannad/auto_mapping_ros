#ifndef MODEL_H
#define MODEL_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include "auto_mapping_ros/state.hpp"
#include "auto_mapping_ros/input.hpp"

// Linearizes the dynamics given the current state and its input using
// Forward Euler discretization of the kinematic model. Accurate upto dt=200ms.
// Equation: x_(t+1) = Ax_(t)+Bu(t)+C

class Model
{
public:
    Model();
    virtual ~Model();
    Eigen::MatrixXd a();
    Eigen::MatrixXd b();
    Eigen::MatrixXd c();
    // Updates a_, b_, c_ about current input and state given dt
    void Linearize(State &S, Input &I, double dt);

private:
    double time_step_;
    Eigen::MatrixXd a_;
    Eigen::MatrixXd b_;
    Eigen::MatrixXd c_;
};

#endif