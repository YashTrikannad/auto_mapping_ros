#ifndef AUTO_MAPPING_ROS_LOCAL_PLANNER_H
#define AUTO_MAPPING_ROS_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include <utility>

#include "auto_mapping_ros/global_planner.h"
#include "auto_mapping_ros/utils.h"

namespace amr
{

class LocalPlanner
{
public:
    LocalPlanner():
            node_handle_(std::make_shared<ros::NodeHandle>(ros::NodeHandle())),
            tf_listener_(tf_buffer_),
            global_planner_(node_handle_),
            coverage_sequence_(),
            last_updated_pose_(),
            current_plan_()
    {
        std::string package_name, csv_relative_filepath;
        node_handle_->getParam("package_name", package_name);
        node_handle_->getParam("csv_filepath", csv_relative_filepath);

        std::string pose_topic, drive_topic;
        node_handle_->getParam("pose_topic", pose_topic);
        node_handle_->getParam("drive_topic", drive_topic);

        node_handle_->getParam("base_frame", base_frame_);
        node_handle_->getParam("map_frame", map_frame_);

        pose_sub_ = node_handle_->subscribe(pose_topic, 5, &LocalPlanner::pose_callback, this);
        drive_pub_ = node_handle_->advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);

        node_handle_->getParam("lookahead_distance", lookahead_distance_);
        node_handle_->getParam("resolution", resolution_);
        node_handle_->getParam("distance_threshold", distance_threshold_);

        ROS_INFO("Starting auto_mapping_ros node ...");
        std::vector<std::array<int, 2>> coverage_sequence_non_ros_map;
        const auto csv_filepath = ros::package::getPath(package_name) + csv_relative_filepath;
        amr::read_sequence_from_csv(&coverage_sequence_non_ros_map, csv_filepath);
        coverage_sequence_ = global_planner_.translate_and_init(coverage_sequence_non_ros_map, resolution_, distance_threshold_);
        ROS_INFO("auto_mapping_ros node is now running!");
    }

    /// Updates the current pose of the agent
    /// @param pose_msg
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        last_updated_pose_[0] = pose_msg->pose.position.x;
        last_updated_pose_[1] = pose_msg->pose.position.y;
        const auto new_plan = global_planner_.update_current_position(last_updated_pose_);
        if(!new_plan.empty())
        {
            current_plan_ = new_plan;
        }
        run_pure_pursuit(current_plan_, last_updated_pose_);
    }

private:
    std::shared_ptr<ros::NodeHandle> node_handle_;
    ros::Subscriber pose_sub_;
    ros::Publisher drive_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double distance_threshold_;
    double lookahead_distance_;
    double resolution_;

    GlobalPlanner global_planner_;
    std::vector<PlannerNode> coverage_sequence_;
    PlannerNode last_updated_pose_;
    std::vector<PlannerNode> current_plan_;

    std::string base_frame_;
    std::string map_frame_;

    std::vector<PlannerNode> transform(const std::vector<PlannerNode>& reference_way_points,
                                       const PlannerNode& current_way_point)
    {
        geometry_msgs::TransformStamped map_to_base_link;
        map_to_base_link = tf_buffer_.lookupTransform(base_frame_, map_frame_, ros::Time(0));

        std::vector<PlannerNode> transformed_way_points;
        for(const auto& reference_way_point: reference_way_points)
        {
            geometry_msgs::Pose map_way_point;
            map_way_point.position.x = reference_way_point[0];
            map_way_point.position.y = reference_way_point[1];
            map_way_point.position.z = 0;
            map_way_point.orientation.x = 0;
            map_way_point.orientation.y = 0;
            map_way_point.orientation.z = 0;
            map_way_point.orientation.w = 1;

            tf2::doTransform(map_way_point, map_way_point, map_to_base_link);
            transformed_way_points.emplace_back(PlannerNode{map_way_point.position.x, map_way_point.position.y});
        }
        return transformed_way_points;
    }


    PlannerNode get_best_track_point(const std::vector<PlannerNode>& way_point_data)
    {
        double closest_distance = std::numeric_limits<double>::max();
        PlannerNode best_node{};

        for(const auto& way_point: way_point_data)
        {
            if(way_point[0] < 0) continue;
            double distance = sqrt(way_point[0]*way_point[0] + way_point[1]*way_point[1]);
            double lookahead_diff = std::abs(distance - lookahead_distance_);
            if(lookahead_diff < closest_distance)
            {
                closest_distance = lookahead_diff;
                best_node = way_point;
            }
        }

        ROS_DEBUG("closest_way_point: %f, %f", static_cast<double>(best_node[0]), static_cast<double>(best_node[1]));
        return best_node;
    }

    void run_pure_pursuit(const std::vector<PlannerNode>& reference_way_points,
                          const PlannerNode& current_way_point)
    {
        const auto transformed_way_points = transform(reference_way_points, current_way_point);
        const auto goal_way_point = get_best_track_point(transformed_way_points);
        const auto steering_angle = 2*(goal_way_point[1])/(lookahead_distance_*lookahead_distance_);

        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = base_frame_;
        drive_msg.drive.steering_angle = steering_angle > 0.4? steering_angle: ((steering_angle<-0.4)? -0.4: steering_angle);
        ROS_DEBUG("steering angle: %f", steering_angle);
        drive_msg.drive.speed = 0.3;
        drive_pub_.publish(drive_msg);
    }
};

} // namespace amr

#endif //FMT_STAR_LOCAL_PLANNER_H
