#include "auto_mapping_ros/model.hpp"

Model::Model()
{
    // ROS_INFO("model created");
}

Model::~Model()
{
    // ROS_INFO("killing the model");
}

Eigen::MatrixXd Model::a()
{
    return a_;
}


Eigen::MatrixXd Model::b()
{
    return b_;
}


Eigen::MatrixXd Model::c()
{
    return c_;
}

void Model::Linearize(State &S, Input &I, double dt)
{
    float L = 0.3302f;
    // a_.resize(3,3);
    a_ = Eigen::MatrixXd::Zero(3, 3);

    // b_.resize(3,2);
    b_ = Eigen::MatrixXd::Zero(3, 2);

    // c_.resize(3,1);
    c_ = Eigen::MatrixXd::Zero(3, 1);

    a_(0,2) = -1*I.v()*sin(S.ori())*dt;
    a_(1,2) = I.v()*cos(S.ori())*dt;
    a_(0,0) = 1;
    a_(1,1) = 1;
    a_(2,2) = 1;

    b_(0,0) = cos(S.ori())*dt;
    b_(1,0) = sin(S.ori())*dt;
    b_(2,0) = tan(I.steer_ang())*dt/L;
    b_(2,1) = I.v()*pow(cos(I.steer_ang()),-2)*dt/L;

    c_(0,0) = I.v()*S.ori()*sin(S.ori())*dt;
    c_(1,0) = -1*I.v()*S.ori()*cos(S.ori())*dt;
    c_(2,0) = -1*I.steer_ang()*I.v()*pow(cos(I.steer_ang()),-2)*dt/L;

}
