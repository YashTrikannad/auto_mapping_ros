#include "auto_mapping_ros/input.hpp"

Input::Input(): v_(0), steer_ang_(0), size_(2)
{
}

Input::Input(double v, double steer_ang): v_(v), steer_ang_(steer_ang), size_(2)
{
}

Input::~Input()
{
}

Eigen::VectorXd Input::ToVector()
{
    Eigen::VectorXd input_vector;
    input_vector.resize(size_);
    input_vector << v_, steer_ang_;
    return input_vector;
}

// Mutators
void Input::set_v(double v)
{
    v_ = v;
}

void Input::set_steer_ang(double steer_ang)
{
    steer_ang_ = steer_ang;
}

// Accessors
double Input::v()
{
    return v_;
}

double Input::steer_ang()
{
    return steer_ang_;
}