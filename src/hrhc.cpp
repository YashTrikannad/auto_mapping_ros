#include "auto_mapping_ros/hrhc.hpp"

HRHC::~HRHC()
{
    ROS_INFO("Killing HRHC");
}
HRHC:: HRHC(ros::NodeHandle &nh):  occ_grid_(nh), trajp_(nh), mpc_(nh)
{
    std::string pose_topic, scan_topic, drive_topic;

    nh_.getParam("/pose_topic", pose_topic);
    nh_.getParam("/scan_topic", scan_topic);
    nh_.getParam("/drive_topic", drive_topic);

    ros::NodeHandle nh_(nh);
    odom_sub_ = nh_.subscribe(pose_topic, 1, &HRHC::OdomCallback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 1, &HRHC::ScanCallback, this);
    current_pose_.position.x = 0;
    current_pose_.position.y = 0;
    current_pose_.position.z = 0;
    current_pose_.orientation.x = 0;
    current_pose_.orientation.y = 0;
    current_pose_.orientation.z = 0;
    current_pose_.orientation.w = 1;
    // trajp_ = TrajectoryPlanner(nh,10);
    trajp_.ReadTrajectories();
    trajp_.Trajectory2world(current_pose_);

    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    std::thread t(&HRHC::DriveLoop, this);
    t.detach();

    ROS_INFO("Created HRHC");
}

void HRHC::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    current_pose_ = odom_msg->pose.pose;
    if (!first_pose_estimate_)
    {
        first_pose_estimate_ = true;
    }
    float current_angle = Transforms::GetCarOrientation(current_pose_);
    State current_state(current_pose_.position.x, current_pose_.position.y, current_angle);

    trajp_.Update(current_pose_, occ_grid_);
    trajp_.Visualize();
    ackermann_msgs::AckermannDriveStamped drive_msg;
    if (first_scan_estimate_)
    {
        Input input_to_pass = GetNextInput();
        input_to_pass.set_v(0);
        if (trajp_.best_trajectory_index() > -1)
        {
            vector<State> bestMiniPath = trajp_.best_minipath();
            mpc_.Update(current_state,input_to_pass,bestMiniPath);
            current_inputs_ = mpc_.solved_trajectory();
            mpc_.Visualize();
            inputs_idx_ = 0;
        }
    }
}

void HRHC::DriveLoop()
{
    while (true)
    {
        if (first_pose_estimate_ && first_scan_estimate_)
        {
            ackermann_msgs::AckermannDriveStamped drive_msg;
            Input input = GetNextInput();
            if (trajp_.best_trajectory_index() < 0)
            {
                input.set_v(0.2);
            }
            drive_msg.header.stamp = ros::Time::now();
            drive_msg.drive.speed = input.v();
            drive_msg.drive.steering_angle = input.steer_ang();
            drive_pub_.publish(drive_msg);
            int dt_ms = 2*mpc_.dt()*1000;
            inputs_idx_++;
            std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
        }
    }
}

Input HRHC::GetNextInput()
{
    if (inputs_idx_ >= current_inputs_.size())
    {
        // ROS_ERROR("Trajectory complete!");
        return Input(0.2,-0.05);
    }
    return current_inputs_[inputs_idx_];
}

void HRHC::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    if (first_pose_estimate_)
    {
        if (!first_scan_estimate_)
        {
            first_scan_estimate_ = true;
        }

        occ_grid_.FillOccGrid(current_pose_, scan_msg);
        occ_grid_.Visualize();
        mpc_.UpdateScan(scan_msg);

    }
}

