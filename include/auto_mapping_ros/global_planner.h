#ifndef FMT_STAR_GLOBAL_PLANNER_H
#define FMT_STAR_GLOBAL_PLANNER_H

#include <memory>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "auto_mapping_ros/utils.h"
#include "fmt_star/plan_srv.h"

namespace amr
{

using PlannerNode = std::array<double, 2>;

class GlobalPlanner
{
public:
    /// Constructor when sequence is already expressed in ros map-coordinates
    /// @param sequence
    /// @param node_handle
    /// @param resolution
    GlobalPlanner(const std::vector<std::array<int, 2>>& sequence,
            std::unique_ptr<ros::NodeHandle>&& node_handle, double resolution):
            current_goal_index_(0),
            sequence_(amr::translate_vector_of_indices_to_xy(sequence, resolution)),
            node_handle_(std::move(node_handle)),
            client_(node_handle_->serviceClient<fmt_star::plan_srv>("FMTstar_search"))
    {
    };

    /// Constructor when sequence is not expressed in ros map-coordinates
    /// @param sequence
    /// @param node_handle
    GlobalPlanner(const std::vector<std::array<double, 2>>& sequence,
                  std::unique_ptr<ros::NodeHandle> node_handle):
            current_goal_index_(0),
            sequence_(sequence),
            node_handle_(std::move(node_handle)),
            client_(node_handle_->serviceClient<fmt_star::plan_srv>("FMTstar_search"))
    {
    };

    /// Find Path between current position and the next position in the sequence
    /// @param current_position - current position (x, y) in the map
    /// @return
    std::vector<PlannerNode> get_next_plan(const PlannerNode &current_position)
    {
        std::cout<< "HI1" << std::endl;
        update_start(current_position);
        std::cout<< "HI1" << std::endl;
        update_end();
        std::cout<< "HI1" << std::endl;
        return find_plan();
    }

    /// Clears the current sequence
    void refresh_sequence()
    {
        sequence_.clear();
    }

private:
    size_t current_goal_index_;
    geometry_msgs::PoseStamped start_;
    geometry_msgs::PoseStamped end_;
    std::vector<PlannerNode> sequence_;
    fmt_star::plan_srv srv_message_;

    std::unique_ptr<ros::NodeHandle> node_handle_;
    ros::ServiceClient client_;

    void update_start(const PlannerNode &current_position)
    {
        start_.pose.position.x = current_position[0];
        start_.pose.position.y = current_position[1];
        start_.pose.position.z = 0;
        start_.pose.orientation.w = 0;
        start_.pose.orientation.x = 0;
        start_.pose.orientation.y = 0;
        start_.pose.orientation.z = 1;
    }

    void update_end()
    {
        current_goal_index_++;
        end_.pose.position.x = sequence_[current_goal_index_][0];
        end_.pose.position.y = sequence_[current_goal_index_][1];
        end_.pose.position.z = 0;
        end_.pose.orientation.w = 0;
        end_.pose.orientation.x = 0;
        end_.pose.orientation.y = 0;
        end_.pose.orientation.z = 1;
    }

    std::vector<PlannerNode> find_plan()
    {
        srv_message_.request.start_position = start_;
        srv_message_.request.end_position = end_;
        srv_message_.request.update_map = true;

        std::cout << "HI" << std::endl;

        std::vector<PlannerNode> current_plan{};
        if(client_.call(srv_message_))
        {
            ROS_INFO("Plan Recieved");
            auto path = srv_message_.response.path.poses;
            for(const auto& node: path)
            {
                std::array<double, 2> global_path_node{};
                global_path_node[0] = node.pose.position.x;
                global_path_node[1] = node.pose.position.y;
                current_plan.emplace_back(global_path_node);
            }
        }
        else
        {
            ROS_ERROR("No Plan Recieved");
        }
        return current_plan;
    }
};

}

#endif //FMT_STAR_GLOBAL_PLANNER_H
