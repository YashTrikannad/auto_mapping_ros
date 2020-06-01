#include "auto_mapping_ros/cost.hpp"

Cost::Cost()
{
}
Cost::Cost(Eigen::MatrixXd q, Eigen::MatrixXd r): q_(q), r_(r)
{
}

Cost::~Cost()
{
}

Eigen::MatrixXd Cost::q()
{
    return q_;
}
Eigen::MatrixXd Cost::r()
{
    return r_;
}