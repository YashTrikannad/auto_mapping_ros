#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>

#include <ros/ros.h>
#include <ros/package.h>

#include "auto_mapping_ros/global_planner.h"

static constexpr auto csv_filepath = "/home/yash/yasht_ws/src/auto_mapping_ros/csv/sequence.csv";

int main(int argc, char **argv)
{
    const auto csv_filepath = ros::package::getPath("auto_mapping_ros") + "/csv/sequence.csv";

    ros::init(argc, argv, "global_planner");

    auto nh = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
    ros::Publisher vis_pub = nh->advertise<visualization_msgs::MarkerArray>("coverage_points", 0);

    const double resolution = 0.05;

    std::vector<std::array<int, 2>> coverage_sequence_non_ros_map;
    amr::read_sequence_from_csv(&coverage_sequence_non_ros_map, csv_filepath);

    const auto coverage_sequence = amr::translate_vector_of_indices_to_xy(
            coverage_sequence_non_ros_map, resolution);

    int i = 0;
    visualization_msgs::MarkerArray marker_array;
    for(const auto& node: coverage_sequence)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "coverage_points";
        marker.id = i++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = node[0];
        marker.pose.position.y = node[1];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
    vis_pub.publish(marker_array);

    amr::GlobalPlanner planner(coverage_sequence, nh);

    for(auto j=0; j<coverage_sequence.size()-1; j++)
    {
        planner.get_next_plan(coverage_sequence[j]);
    }

    return 0;
}