#ifndef AUTO_MAPPING_ROS_LOCAL_PLANNER_H
#define AUTO_MAPPING_ROS_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include <utility>
#include <std_msgs/Bool.h>

#include "auto_mapping_ros/global_planner.h"
#include "auto_mapping_ros/maneuvers.h"
#include "auto_mapping_ros/utils.h"
#include "fmt_star/planner.h"

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
        local_planner_id_ = ++local_planner_counter_;

        std::string package_name, csv_relative_filepath;
        node_handle_->getParam("package_name", package_name);
        node_handle_->getParam("csv_filepath", csv_relative_filepath);
        csv_relative_filepath = csv_relative_filepath + "_" + std::to_string(local_planner_id_) + ".csv";

        std::string pose_topic, drive_topic, brake_topic;
        node_handle_->getParam("pose_topic", pose_topic);
        pose_topic = pose_topic + "_" + std::to_string(local_planner_id_);
        node_handle_->getParam("drive_topic", drive_topic);
        drive_topic = drive_topic + "_" + std::to_string(local_planner_id_);
        node_handle_->getParam("brake_topic", brake_topic);
        brake_topic = brake_topic + "_" + std::to_string(local_planner_id_);

        node_handle_->getParam("base_frame", base_frame_);
        base_frame_ = "racecar" + std::to_string(local_planner_id_)  + "/" + base_frame_;
        node_handle_->getParam("map_frame", map_frame_);

        pose_sub_ = node_handle_->subscribe(pose_topic, 5, &LocalPlanner::pose_callback, this);
        drive_pub_ = node_handle_->advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
        brake_pub_ = node_handle_->advertise<std_msgs::Bool>(brake_topic, 1);

        node_handle_->getParam("lookahead_distance", lookahead_distance_);

        // Get ROS Map
        auto input_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(0.1));
        while(input_map == nullptr || input_map->data.empty())
        {
            ROS_WARN("Waiting for Map Message on topic: %s", "map");
            input_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(1));
        }
        ROS_INFO("Map Received");

        // Update ROS Map Parameters
        resolution_ = input_map->info.resolution;
        double origin_x = input_map->info.origin.position.x;
        double origin_y = input_map->info.origin.position.y;
        int ros_map_width_cells = input_map->info.width;
        int ros_map_height_cells = input_map->info.height;

        node_handle_->getParam("distance_threshold", distance_threshold_);
        node_handle_->getParam("velocity", velocity_);

        // Update Non ROS Map Params
        double non_ros_map_width;
        node_handle_->getParam("non_ros_map_width", non_ros_map_width);
        double non_ros_map_height;
        node_handle_->getParam("non_ros_map_height", non_ros_map_height);
        bool switch_xy;
        node_handle_->getParam("switch_xy", switch_xy);

        std::vector<std::array<int, 2>> coverage_sequence_non_ros_map;
        const auto csv_filepath = ros::package::getPath(package_name) + csv_relative_filepath;
        amr::read_sequence_from_csv(&coverage_sequence_non_ros_map, csv_filepath);

        // Translate non ros sequence to ros
        const auto coverage_sequence_ros_map = fmt_star::translate_sequence_to_ros_coords(coverage_sequence_non_ros_map,
                non_ros_map_width,
                non_ros_map_height,
                switch_xy,
                ros_map_width_cells*resolution_,
                ros_map_height_cells*resolution_,
                origin_x,
                origin_y);

        coverage_sequence_ = global_planner_.init(coverage_sequence_ros_map, distance_threshold_);

        std::thread global_planning_thread(&GlobalPlanner::start_global_planner, &global_planner_);
        global_planning_thread.detach();
        ROS_INFO("auto_mapping_ros node is now running!");
    }

    /// Updates the current pose of the agent
    /// @param pose_msg
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        last_updated_pose_[0] = pose_msg->pose.position.x;
        last_updated_pose_[1] = pose_msg->pose.position.y;
        global_planner_.update_current_position(last_updated_pose_);
        const auto new_plan = global_planner_.get_new_plan();
        if(new_plan)
        {
            ROS_INFO("Updated Global Plan for Car %i", local_planner_id_);
            current_plan_ = new_plan.value();
        }
        run_pure_pursuit(current_plan_, last_updated_pose_);
    }

private:
    int local_planner_id_;
    static int local_planner_counter_;

    std::shared_ptr<ros::NodeHandle> node_handle_;
    ros::Subscriber pose_sub_;
    ros::Publisher drive_pub_;
    ros::Publisher brake_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double distance_threshold_;
    double lookahead_distance_;
    double resolution_;
    double velocity_;

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
        PlannerNode best_node{-1, -1};

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

        if(best_node == PlannerNode{-1, -1})
        {
            // TODO: Execute Reverse
            ROS_INFO("Pure Pursuit Failed to Find a Point in Front. Executing Stop.");
            stop_vehicle(&brake_pub_);
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
        drive_msg.drive.speed = velocity_;
        drive_pub_.publish(drive_msg);
    }
};

int LocalPlanner::local_planner_counter_ = 0;

} // namespace amr

#endif //FMT_STAR_LOCAL_PLANNER_H
