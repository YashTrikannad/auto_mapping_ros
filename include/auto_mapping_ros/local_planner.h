#ifndef FMT_STAR_LOCAL_PLANNER_H
#define FMT_STAR_LOCAL_PLANNER_H

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include <utility>

#include "auto_mapping_ros/global_planner.h"
#include "auto_mapping_ros/utils.h"

static const auto localized_pose_topic = "gt_pose";
static const auto drive_topic = "nav";
static const auto csv_filepath = "/home/yash/yasht_ws/src/auto_mapping_ros/csv/ideal_sequence.csv";

namespace amr
{

class LocalController
{
public:
    LocalController(std::shared_ptr<ros::NodeHandle> node_handle) :
            node_handle_(std::move(node_handle)),
            pose_sub_(node_handle_->subscribe(localized_pose_topic, 5, &LocalController::pose_callback, this)),
            drive_pub_(node_handle_->advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1)),
            tf_listener_(tf_buffer_),
            global_planner_(node_handle_)
    {
        node_handle_->getParam("/lookahead_distance", lookahead_distance_);
        node_handle_->getParam("/resolution", resolution_);
        ROS_INFO("Starting auto_mapping_ros node ...");
        std::vector<std::array<int, 2>> coverage_sequence_non_ros_map;
        amr::read_sequence_from_csv(&coverage_sequence_non_ros_map, csv_filepath);
        global_planner_.translate_and_init(coverage_sequence_non_ros_map, resolution_);
        ros::Duration(1.0).sleep();
        ROS_INFO("auto_mapping_ros node is now running!");
    }

private:
    std::shared_ptr<ros::NodeHandle> node_handle_;
    ros::Subscriber pose_sub_;
    ros::Publisher drive_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double lookahead_distance_{};
    geometry_msgs::PoseStamped last_updated_pose_;
    double resolution_{};

    GlobalPlanner global_planner_;
    std::vector<std::array<double, 2>> coverage_sequence_;

    /// Updates the current pose of the agent
    /// @param pose_msg
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        last_updated_pose_ = *pose_msg;
    }

    /// Updates the sequence in the Local Planner that is to be tracked by the global planner
    /// @param coverage_sequence_non_ros_map
    void update_sequence(std::vector<std::array<int, 2>> *coverage_sequence_non_ros_map)
    {
        amr::read_sequence_from_csv(coverage_sequence_non_ros_map, csv_filepath);
        coverage_sequence_ = amr::translate_vector_of_indices_to_xy(*coverage_sequence_non_ros_map, resolution_);
    }

    /// Main loop of the local planner that executes while the conditions are satisfied
    void loop();
};

} // namespace amr

#endif //FMT_STAR_LOCAL_PLANNER_H
