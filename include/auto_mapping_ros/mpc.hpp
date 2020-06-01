#ifndef MPC_H
#define MPC_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>


#include "auto_mapping_ros/constraints.hpp"
#include "auto_mapping_ros/state.hpp"
#include "auto_mapping_ros/model.hpp"
#include "auto_mapping_ros/cost.hpp"
#include "auto_mapping_ros/visualizer.hpp"

// Implements Model Predictive Control by minimizing a quadratic programming
// problem given constraints. Uses OSQP for optimization.

class MPC
{
public:
    MPC();
    MPC(ros::NodeHandle &nh);
    virtual ~MPC();
    // Runs one iteration of MPC given the latest state from callback and last MPC input
    // Uses desired_state_trajectory for tracking reference
    void Update(State current_state, Input input, std::vector<State> &desired_state_trajectory);
    // Generates pretty lines
    void Visualize();
    // Updates scan_msg content
    void UpdateScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    // accessor functions
    Constraints constraints();
    float dt();
    int horizon();
    std::vector<Input> solved_trajectory();
private:
    int horizon_;
    int input_size_;
    int state_size_;
    int num_states_;
    int num_inputs_;
    int num_variables_;
    int num_constraints_;
    bool solver_init_ = false;
    float dt_;

    Eigen::SparseMatrix<double> hessian_;
    Eigen::VectorXd gradient_;
    Eigen::SparseMatrix<double> linear_matrix_;
    Eigen::VectorXd lower_bound_;
    Eigen::VectorXd upper_bound_;
    Constraints constraints_;
    Model model_;
    State current_state_;
    Input desired_input_;
    std::vector<State> desired_state_trajectory_;
    Cost cost_;
    sensor_msgs::LaserScan scan_msg_;
    Eigen::VectorXd full_solution_;
    OsqpEigen::Solver solver_;
    std::vector<Input> solved_trajectory_;

    // ros vis stuff
    ros::NodeHandle nh_;
    ros::Publisher mpc_pub_;
    std::vector<geometry_msgs::Point> points_;
    std::vector<std_msgs::ColorRGBA> colors_;
    // Initializes the hessian matrix for costs (which is constant)
    void CreateHessianMatrix();
    // Updates the gradient vector for costs. The gradient vector is fully updated each time.
    void CreateGradientVector();
    // Initializes the linear constraint matrix. 
    void CreateLinearConstraintMatrix();
    // Updated the non-constant parts of the linear constraint matrix
    void UpdateLinearConstraintMatrix();
    // Creates the upper and lower bound constraints for states and inputs
    void CreateLowerBound();
    void CreateUpperBound();
    // Updates the non-constant parts of the upper and lower bound constraint vectors
    void UpdateLowerBound();
    void UpdateUpperBound();
    // Initiliaze a block in a sparse matrix
    void SparseBlockInit(Eigen::SparseMatrix<double> &modify, const Eigen::MatrixXd &block, int row_start, int col_start);
    // Sets an already initialized block in a sparse matrix
    void SparseBlockSet(Eigen::SparseMatrix<double> &modify, const Eigen::MatrixXd &block, int row_start, int col_start);
    // Sets just the ones of an identity block in a sparse matrix
    void SparseBlockEye(Eigen::SparseMatrix<double> &modify, int size, int row_start, int col_start, int number);
    // Pushes lines that create a state and input representation of the car for visualization
    void DrawCar(State &state, Input &input);
    // Converts output of osqp vector into vector of inputs
    void UpdateSolvedTrajectory();
};
#endif