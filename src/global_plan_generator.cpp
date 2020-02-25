#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "fmt_star/plan_srv.h"
#include "auto_mapping_ros/utils.h"

static constexpr auto csv_filepath = "/home/yash/yasht_ws/src/auto_mapping_ros/csv/ideal_sequence.csv";
static constexpr auto global_plan_filepath = "/home/yash/yasht_ws/src/auto_mapping_ros/csv/global_plan.csv";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_plan_generator");
    ros::NodeHandle nh;
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "coverage_points", 0 );

    double resolution = 0.05;

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<fmt_star::plan_srv>("FMTstar_search");

    fmt_star::plan_srv srv_message;

    std::vector<std::array<int, 2>> coverage_sequence_non_ros_map;
    amr::read_sequence_from_csv(&coverage_sequence_non_ros_map, csv_filepath);

    const auto coverage_sequence = amr::translate_vector_of_indices_to_xy(
            coverage_sequence_non_ros_map, resolution);

    amr::print_vector_of_nodes(coverage_sequence);
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

    std::vector<Plan> global_coverage_plans{};
    for(int i=coverage_sequence.size(); i!=1; i--)
    {
        geometry_msgs::PoseStamped start;
        start.pose.position.x = coverage_sequence[i-1][0];
        start.pose.position.y = coverage_sequence[i-1][1];
        start.pose.position.z = 0;
        start.pose.orientation.w = 0;
        start.pose.orientation.x = 0;
        start.pose.orientation.y = 0;
        start.pose.orientation.z = 1;

        geometry_msgs::PoseStamped end;
        end.pose.position.x = coverage_sequence[i-2][0];
        end.pose.position.y = coverage_sequence[i-2][1];
        end.pose.position.z = 0;
        end.pose.orientation.w = 0;
        end.pose.orientation.x = 0;
        end.pose.orientation.y = 0;
        end.pose.orientation.z = 1;

        srv_message.request.start_position = start;
        srv_message.request.end_position = end;
        srv_message.request.update_map = true;

        Plan current_plan;
        current_plan.start = coverage_sequence[i-1];
        current_plan.end = coverage_sequence[i-2];

        if (client.call(srv_message))
        {
            auto path = srv_message.response.path.poses;
            for(const auto& node: path)
            {
                std::array<double, 2> global_path_node{};
                global_path_node[0] = node.pose.position.x;
                global_path_node[1] = node.pose.position.y;
                current_plan.plan.emplace_back(global_path_node);
            }
            global_coverage_plans.emplace_back(current_plan);
            ROS_INFO("Plan Recieved");
        }
        else
        {
            ROS_ERROR("No Plan Recieved");
        }
    }

    amr::write_plans_to_csv(global_coverage_plans, global_plan_filepath);

    return 0;
}
