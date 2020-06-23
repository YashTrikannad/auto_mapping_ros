#include "auto_mapping_ros/mpc.hpp"

MPC::MPC(ros::NodeHandle &nh):
    input_size_(2),
    state_size_(3),
    constraints_(nh),
    nh_(nh)
{
    nh.getParam("/horizon", horizon_);
    nh.getParam("/dt", dt_);
    double desired_vel, desired_steer, q0, q1, q2, r0, r1;
    nh.getParam("des_vel", desired_vel);
    nh.getParam("des_steer", desired_steer);
    nh_.getParam("/q0", q0);
    nh_.getParam("/q1", q1);
    nh_.getParam("/q2", q2);
    nh_.getParam("/r0", r0);
    nh_.getParam("/r1", r1);
    std::cout<<r1<<std::endl;
    desired_input_.set_v(desired_vel);
    desired_input_.set_steer_ang(desired_steer);
    Eigen::DiagonalMatrix<double, 3> q;
    Eigen::DiagonalMatrix<double, 2> r;
    q.diagonal() << q0, q1, q2;
    r.diagonal() << r0, r1;
    cost_ = Cost(q, r);
    num_inputs_=(input_size_ * horizon_);
    num_states_=(state_size_ * (horizon_ + 1));
    num_variables_=(num_states_ + num_inputs_);
    num_constraints_=(num_states_ + 2 * (horizon_ + 1) + num_inputs_ + 2 * horizon_); // dynamics + follow the gap + max/min input + skid limit

    hessian_.resize(num_variables_, num_variables_);
    gradient_.resize(num_variables_);
    linear_matrix_.resize(num_constraints_, num_variables_);
    lower_bound_.resize(num_constraints_);
    upper_bound_.resize(num_constraints_);

    CreateHessianMatrix();
    CreateLinearConstraintMatrix();
    CreateUpperBound();
    CreateLowerBound();

    full_solution_ = Eigen::VectorXd::Zero(num_variables_);

    mpc_pub_ = nh.advertise<visualization_msgs::Marker>("mpc", 1);
    ROS_INFO("mpc created");
}


MPC::~MPC()
{
    ROS_INFO("killing the mpc");
}

float MPC::dt()
{
    return dt_;
}

int MPC::horizon()
{
    return horizon_;
}

std::vector<Input> MPC::solved_trajectory()
{
    return solved_trajectory_;
}

void MPC::UpdateScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    scan_msg_ = *scan_msg;
}

void MPC::Update(State current_state, Input input, std::vector<State> &desired_state_trajectory)
{
    current_state_ = current_state;
    desired_state_trajectory_ = desired_state_trajectory;
    // std::cout<<desired_state_trajectory.size()<<std::endl;
    model_.Linearize(current_state_, input, dt_);
    constraints_.set_state(current_state_);
    constraints_.FindHalfSpaces(current_state_,scan_msg_);

    CreateGradientVector();
    UpdateLinearConstraintMatrix();
    UpdateLowerBound();
    UpdateUpperBound();
    if (solver_init_)
    {
        if (!solver_.updateGradient(gradient_))
        {
            std::cout << "gradient failed" << std::endl;
        }
        if (!solver_.updateLinearConstraintsMatrix(linear_matrix_))
        {
            std::cout << "linear failed" << std::endl;
        }
        if (!solver_.updateBounds(lower_bound_, upper_bound_))
        {
            std::cout << "bounds failed" << std::endl;
        }
    }
    else
    {
        solver_.settings()->setWarmStart(true);
        solver_.settings()->setVerbosity(false);
        solver_.data()->setNumberOfVariables(num_variables_);
        solver_.data()->setNumberOfConstraints(num_constraints_);
        if (!solver_.data()->setHessianMatrix(hessian_))
        {
            std::cout << "hessian failed" << std::endl;
        }
        if (!solver_.data()->setGradient(gradient_))
        {
            std::cout << "gradient failed" << std::endl;
        }
        if (!solver_.data()->setLinearConstraintsMatrix(linear_matrix_))
        {
            std::cout << "linear failed" << std::endl;
        }
        if (!solver_.data()->setLowerBound(lower_bound_))
        {
            std::cout << "lower failed" << std::endl;
        }
        if (!solver_.data()->setUpperBound(upper_bound_))
        {
            std::cout << "upper failed" << std::endl;
        }
        if (!solver_.initSolver())
        {
            std::cout << "solver failed" << std::endl;
            
        } else
        {
            solver_init_ = true;
        }
        
    }
    if (!solver_.solve())
    {
        ROS_ERROR("solve failed");
    }
    else
    {
        full_solution_ = solver_.getSolution();
        UpdateSolvedTrajectory();
    }
}

void MPC::UpdateSolvedTrajectory()
{
    solved_trajectory_.clear();//full_solution_.size()-1
    for (int i = num_states_; i < num_states_+10; i+=2)
    {
        double v = full_solution_(i);
        double angle = full_solution_(i+1);
        if (std::isnan(v) || std::isnan(angle))
        {
            return;
        }
        Input input(v, angle);
        solved_trajectory_.push_back(input);
    }
}

void MPC::Visualize()
{
    State predicted_state;
    Input predicted_input;
    for (int ii = 0; ii < horizon_; ++ii)
    {
        predicted_state.set_x(full_solution_(ii * state_size_));
        predicted_state.set_y(full_solution_(ii * state_size_ + 1));
        predicted_state.set_ori(full_solution_(ii * state_size_ + 2));

        predicted_input.set_v(full_solution_(num_states_ + ii * input_size_));
        predicted_input.set_steer_ang(full_solution_(num_states_ + ii * input_size_ + 1));

        DrawCar(predicted_state, predicted_input);
    }

    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;

    // for (unsigned int ii = 0; ii < desired_state_trajectory_.size(); ++ii)
    // {
    //     point.x = desired_state_trajectory_[ii].x();
    //     point.y = desired_state_trajectory_[ii].y();
    //     point.z = 0.2;
    //     points_.push_back(point);
    //     point.x = desired_state_trajectory_[ii].x() + 0.4 * cos(desired_state_trajectory_[ii].ori());
    //     point.y = desired_state_trajectory_[ii].y() + 0.4 * sin(desired_state_trajectory_[ii].ori());
    //     point.z = 0.2;
    //     points_.push_back(point);
    //     color.r = (float)ii / horizon_;
    //     color.g = 0;
    //     color.b = 0;
    //     color.a = 1;
    //     colors_.push_back(color);
    //     color.r = 0;
    //     color.g = (float)ii / horizon_;
    //     color.b = 0;
    //     color.a = 1;
    //     colors_.push_back(color);
    // }

    // mpc_pub_.publish(Visualizer::GenerateList(points_, colors_, visualization_msgs::Marker::LINE_LIST, 0.05, 0.0, 0.0));
    points_.clear();
    colors_.clear();
}

void MPC::CreateHessianMatrix()
{
    hessian_.setZero();
    for (int ii = 0; ii < horizon_ + 1; ++ii) // change for terminal cost
    {
        SparseBlockInit(hessian_, cost_.q(), ii * state_size_, ii * state_size_);
    }
    for (int ii = 0; ii < horizon_; ++ii) // change for terminal cost
    {
        SparseBlockInit(hessian_, cost_.r(), num_states_ + ii * input_size_, num_states_ + ii * input_size_);
    }
}

void MPC::CreateGradientVector()
{
    for (int ii  = 0; ii < horizon_; ++ii)
    {
        gradient_.block(ii * state_size_, 0, state_size_, 1) = -1 * cost_.q() * desired_state_trajectory_[ii].ToVector();
        gradient_.block(num_states_ + ii * input_size_, 0, input_size_, 1) = -1 * cost_.r() * desired_input_.ToVector();
    }
    gradient_.block(horizon_ * state_size_, 0, 3, 1) = -1 * cost_.q() * desired_state_trajectory_[horizon_ - 1].ToVector();
}

void MPC::CreateLinearConstraintMatrix()
{
    linear_matrix_.setZero();

    Eigen::MatrixXd a_eye(state_size_, 2 * state_size_);
    a_eye << Eigen::MatrixXd::Identity(state_size_, state_size_), -Eigen::MatrixXd::Identity(state_size_, state_size_);
    Eigen::MatrixXd gap_con(2, state_size_);
    gap_con = Eigen::MatrixXd::Ones(2,state_size_);

    // prediction equality constraint
    SparseBlockInit(linear_matrix_, gap_con, num_states_, 0);

    // prediction equality constraint
    SparseBlockEye(linear_matrix_, num_states_, 0, 0, -1);
    for (int ii = 1; ii < horizon_ + 1; ++ii)
    {
        SparseBlockInit(linear_matrix_, Eigen::MatrixXd::Identity(state_size_, state_size_), ii*state_size_, (ii - 1) * state_size_);
        SparseBlockInit(linear_matrix_, Eigen::MatrixXd::Identity(state_size_, input_size_), ii*state_size_, num_states_ + (ii - 1) * input_size_);
        SparseBlockInit(linear_matrix_, gap_con, num_states_ + 2 * (ii), (ii)*state_size_);
        SparseBlockInit(linear_matrix_, constraints_.slip_constraint(), num_states_ + 2 * (horizon_ + 1) + num_inputs_ + ii - 1, num_states_ + (ii - 1) * input_size_);
    }

    SparseBlockEye(linear_matrix_, num_inputs_, num_states_ + 2 * (horizon_ + 1), num_states_, 1);
}

void MPC::UpdateLinearConstraintMatrix()
{
    Eigen::MatrixXd a_eye(state_size_, 2 * state_size_);
    a_eye << model_.a(), -Eigen::MatrixXd::Identity(state_size_, state_size_);
    Eigen::MatrixXd gap_con(2, state_size_);
    gap_con(0, 0) = constraints_.l1()(0);
    gap_con(0, 1) = constraints_.l1()(1);
    gap_con(0, 2) = 0;
    gap_con(1, 0) = constraints_.l2()(0);
    gap_con(1, 1) = constraints_.l2()(1);
    gap_con(1, 2) = 0;
    for (int ii = 1; ii < horizon_ + 1; ++ii)
    {
        SparseBlockSet(linear_matrix_, model_.a(), ii*state_size_, (ii - 1) * state_size_);
        SparseBlockSet(linear_matrix_, model_.b(), ii*state_size_, num_states_ + (ii - 1) * input_size_);
        SparseBlockSet(linear_matrix_, gap_con, num_states_ + 2 * (ii), (ii)*state_size_);
    }
}

void MPC::CreateLowerBound()
{
    lower_bound_.resize(num_constraints_);
    Eigen::VectorXd gap_con(2);
    gap_con(0) = -OsqpEigen::INFTY;
    gap_con(1) = -OsqpEigen::INFTY;
    lower_bound_ << Eigen::VectorXd::Zero((horizon_ + 1) * state_size_), gap_con.replicate(horizon_ + 1, 1), constraints_.u_min().replicate(horizon_, 1), constraints_.slip_lower_bound().replicate(horizon_, 1);
}

void MPC::CreateUpperBound()
{
    upper_bound_.resize(num_constraints_);
    Eigen::VectorXd gap_con(2);
    gap_con(0) = OsqpEigen::INFTY;
    gap_con(1) = OsqpEigen::INFTY;
    upper_bound_ << Eigen::VectorXd::Zero((horizon_ + 1) * state_size_), gap_con.replicate(horizon_ + 1, 1), constraints_.u_max().replicate(horizon_, 1), constraints_.slip_upper_bound().replicate(horizon_, 1);
}

void MPC::UpdateLowerBound()
{
    lower_bound_.resize(num_constraints_);
    Eigen::VectorXd gap_con(2);
    gap_con(0) = -OsqpEigen::INFTY;//-constraints_.l1()(2);
    gap_con(1) = -OsqpEigen::INFTY;//-constraints_.l2()(2);
    lower_bound_.head(num_states_) << -current_state_.ToVector(), -model_.c().replicate(horizon_, 1);
    lower_bound_.block(num_states_, 0, (horizon_ + 1) * 2, 1) = gap_con.replicate(horizon_ + 1, 1);
}

void MPC::UpdateUpperBound()
{
    upper_bound_.head(num_states_) << -current_state_.ToVector(), -model_.c().replicate(horizon_, 1);
}

void MPC::SparseBlockInit(Eigen::SparseMatrix<double> &modify, const Eigen::MatrixXd &block, int row_start, int col_start)
{
    int row_size = block.rows();
    int col_size = block.cols();
    for (int row = 0; row < row_size; ++row)
    {
        for (int col = 0; col < col_size; ++col)
        {
            modify.insert(row_start + row, col_start + col) = block(row, col);
        }
    }
}

void MPC::SparseBlockSet(Eigen::SparseMatrix<double> &modify, const Eigen::MatrixXd &block, int row_start, int col_start)
{
    int row_size = block.rows();
    int col_size = block.cols();
    for (int row = 0; row < row_size; ++row)
    {
        for (int col = 0; col < col_size; ++col)
        {
            modify.coeffRef(row_start + row, col_start + col) = block(row, col);
        }
    }
}

void MPC::SparseBlockEye(Eigen::SparseMatrix<double> &modify, int size, int row_start, int col_start, int number)
{
    for (int row = 0; row < size; ++row)
    {
        modify.insert(row_start + row, col_start + row) = number;
    }
}

void MPC::DrawCar(State &state, Input &input)
{
    float L = 0.3302f;
    float wheel_size = 0.2;
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;

    point.x = state.x();
    point.y = state.y();
    point.z = 0.1;
    points_.push_back(point);

    point.x = state.x() + L * cos(state.ori());
    point.y = state.y() + L * sin(state.ori());
    point.z = 0.1;
    points_.push_back(point);

    point.x = state.x() + L * cos(state.ori()) - 0.5 * wheel_size * cos(state.ori() + input.steer_ang());
    point.y = state.y() + L * sin(state.ori()) - 0.5 * wheel_size * sin(state.ori() + input.steer_ang());
    point.z = 0.125;
    points_.push_back(point);

    point.x = state.x() + L * cos(state.ori()) + 0.5 * wheel_size * cos(state.ori() + input.steer_ang());
    point.y = state.y() + L * sin(state.ori()) + 0.5 * wheel_size * sin(state.ori() + input.steer_ang());

    point.z = 0.125;
    points_.push_back(point);

    point.x = state.x() + L * 0.5 * cos(state.ori());
    point.y = state.y() + L * 0.5 * sin(state.ori());
    point.z = 0.15;
    points_.push_back(point);
    point.x = state.x() + L * 0.5 * cos(state.ori()) + L * 0.5 * (input.v() / constraints_.u_max()(0)) * cos(state.ori());
    point.y = state.y() + L * 0.5 * sin(state.ori()) + L * 0.5 * (input.v() / constraints_.u_max()(0)) * sin(state.ori());
    point.z = 0.15;
    points_.push_back(point);

    color.r = 0;
    color.g = 0;
    color.b = 1;
    color.a = 1;
    colors_.push_back(color);
    colors_.push_back(color);

    color.r = 0;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    colors_.push_back(color);
    colors_.push_back(color);

    color.r = 1;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    colors_.push_back(color);
    colors_.push_back(color);
}