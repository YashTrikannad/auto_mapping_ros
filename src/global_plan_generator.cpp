#include <ros/ros.h>

#include "fmt_star/plan_srv.h"
#include "auto_mapping_ros/utils.h"

static constexpr auto csv_filepath = "/home/yash/yasht_ws/src/auto_mapping_ros/csv/sequence.csv";
static constexpr auto global_plan_filepath = "/home/yash/yasht_ws/src/auto_mapping_ros/csv/global_plan.csv";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_plan_generator");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<fmt_star::plan_srv>("FMTstar_search");

    fmt_star::plan_srv srv_message;

    std::vector<std::array<int, 2>> coverage_sequence_non_ros_map;
    std::vector<std::array<double, 2>> coverage_sequence;
    amr::read_sequence_from_csv(&coverage_sequence_non_ros_map, csv_filepath);

    std::vector<std::array<double, 2>> global_coverage_plan;

    for(const auto& node:coverage_sequence_non_ros_map)
    {
        coverage_sequence.emplace_back(amr::translate_node_from_previous_map_to_ros_map(node));
    }

    for(int i=1; i<coverage_sequence.size(); i++)
    {
        geometry_msgs::PoseStamped start;
        start.pose.position.x = coverage_sequence[i-1][0];
        start.pose.position.y = coverage_sequence[i-1][1];
        start.pose.position.z = 0;
        start.pose.orientation.w = 0;
        start.pose.orientation.x = 0;
        start.pose.orientation.y = 0;
        start.pose.orientation.z = 0;

        geometry_msgs::PoseStamped end;
        start.pose.position.x = coverage_sequence[i][0];
        start.pose.position.y = coverage_sequence[i][1];
        start.pose.position.z = 0;
        start.pose.orientation.w = 0;
        start.pose.orientation.x = 0;
        start.pose.orientation.y = 0;
        start.pose.orientation.z = 0;

        srv_message.request.start_position = start;
        srv_message.request.end_position = end;
        srv_message.request.update_map = true;

        if (client.call(srv_message))
        {
            const auto path = srv_message.response.path.poses;
            for(const auto& node: path)
            {
                std::array<double, 2> global_path_node{};
                global_path_node[0] = node.pose.position.x;
                global_path_node[1] = node.pose.position.y;
                global_coverage_plan.emplace_back(global_path_node);
            }
            ROS_INFO("Plan Recieved");
        }
        else
        {
            ROS_ERROR("No Plan Recieved");
        }
    }

    amr::write_sequence_to_csv(global_coverage_plan, global_plan_filepath);

    return 0;
}
