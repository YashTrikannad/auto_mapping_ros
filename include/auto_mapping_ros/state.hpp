#ifndef STATE_H
#define STATE_H

#include <ros/ros.h>
#include <Eigen/Geometry>

// Defines format of State used for MPC
// Stores x,y coordinates and orientation of the car

class State
{
public:
    State();
    State(double x, double y, double ori);
    virtual ~State();

    // Converts from state to Eigen vector
    Eigen::VectorXd ToVector();

    // setters
    void set_x(double x);
    void set_y(double y);
    void set_ori(double ori);
    std::pair<float,float> GetPair();
    // getters
    double x();
    double y();
    double ori();
    int size();
private:
    double x_;
    double y_;
    double ori_;
    int size_;
};

#endif