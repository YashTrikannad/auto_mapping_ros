#ifndef COST_H
#define COST_H
#include <ros/ros.h>
#include <Eigen/Geometry>

// Stores the Q and R cost matrices using Eigen matrices

class Cost
{
public:
    Cost();
    Cost(Eigen::MatrixXd q, Eigen::MatrixXd r);
    virtual ~Cost();

    Eigen::MatrixXd q();
    Eigen::MatrixXd r();
private:
    Eigen::MatrixXd q_;
    Eigen::MatrixXd r_;
    // mode
};
#endif