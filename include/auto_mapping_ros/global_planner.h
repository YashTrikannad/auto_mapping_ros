#ifndef AUTO_MAPPING_ROS_GLOBAL_PLANNER_H
#define AUTO_MAPPING_ROS_GLOBAL_PLANNER_H

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
    /// Constructs GlobalPlanner (Does not initialize the sequence)
    /// @param node_handle
    GlobalPlanner(std::shared_ptr<ros::NodeHandle> node_handle):
            current_tracking_node_index_(0),
            current_goal_index_(0),
            node_handle_(std::move(node_handle)),
            first_plan_(true)
    {
        std::string planner_name;
        node_handle_->getParam("planner_name", planner_name);
        client_ = node_handle_->serviceClient<fmt_star::plan_srv>(planner_name);
    };

    /// Constructor when sequence is already expressed in ros map-coordinates
    /// @param sequence
    /// @param node_handle
    /// @param resolution
    GlobalPlanner(const std::vector<std::array<int, 2>>& sequence,
                  std::shared_ptr<ros::NodeHandle> node_handle, double resolution):
            current_tracking_node_index_(0),
            current_goal_index_(0),
            sequence_(amr::translate_vector_of_indices_to_xy(sequence, resolution)),
            node_handle_(std::move(node_handle)),
            client_(node_handle_->serviceClient<fmt_star::plan_srv>("FMTstar_search")),
            first_plan_(true)
    {
    };

    /// Constructor when sequence is not expressed in ros map-coordinates
    /// @param sequence
    /// @param node_handle
    GlobalPlanner(std::vector<std::array<double, 2>>  sequence,
                  std::shared_ptr<ros::NodeHandle> node_handle):
            current_tracking_node_index_(0),
            current_goal_index_(0),
            sequence_(std::move(sequence)),
            node_handle_(std::move(node_handle)),
            client_(node_handle_->serviceClient<fmt_star::plan_srv>("FMTstar_search")),
            first_plan_(true)
    {
    };

    /// Find Path between current position and the next position in the sequence
    /// @param current_position - current position (x, y) in the map
    /// @return
    std::vector<PlannerNode> get_next_plan(const PlannerNode &current_position)
    {
        if(sequence_.empty())
        {
            ROS_ERROR("The coverage sequence is empty. It needs to be set before calling get_next_plan.");
        }
        update_start(current_position);
        update_end();
        return find_plan();
    }

    /// Initializes the Global Planner with the sequence (coordinates of sequence already same as in ROS Map)
    /// @param sequence
    std::vector<PlannerNode> init(const std::vector<PlannerNode>& sequence)
    {
        sequence_.clear();
        sequence_ = sequence;
        ROS_INFO("Global Planner Initialized");
        return sequence_;
    }

    /// Initializes the Global Planner by first translating non ros sequence in compatible format and then initializes
    /// the sequence
    /// @param sequence
    /// @param resolution
    std::vector<PlannerNode> translate_and_init(const std::vector<std::array<int, 2>>& sequence, double resolution, double distance_threshold)
    {
        sequence_.clear();
        sequence_ = amr::translate_vector_of_indices_to_xy(sequence, resolution);
        distance_threshold_ = distance_threshold;
        ROS_INFO("Global Planner Initialized");
        return sequence_;
    }

    std::vector<PlannerNode> update_current_position(const PlannerNode &current_position)
    {
        current_position_ = current_position;
        std::vector<PlannerNode> new_plan{};
        if(distance(current_position_, sequence_[current_tracking_node_index_]) < distance_threshold_ || first_plan_)
        {
            ROS_INFO("Getting New Plan.");
            if(first_plan_)
            {
                first_plan_ = false;
            }
            else
            {
                current_tracking_node_index_++;
            }
            new_plan = get_next_plan(current_position_);
            if(current_tracking_node_index_ == sequence_.size()-1)
            {
                ROS_INFO("Sequence Explored!");
            }
        }
        return new_plan;
    }

private:
    size_t current_tracking_node_index_;
    size_t current_goal_index_;
    geometry_msgs::PoseStamped start_;
    geometry_msgs::PoseStamped end_;
    PlannerNode current_position_;
    std::vector<PlannerNode> sequence_;
    fmt_star::plan_srv srv_message_;

    std::shared_ptr<ros::NodeHandle> node_handle_;
    ros::ServiceClient client_;

    bool first_plan_;
    double distance_threshold_;

    /// Update the start position as the current position
    /// @param current_position
    void update_start(const PlannerNode &current_position)
    {
        ROS_INFO("Start Position");
        std::cout << current_position[0] << " " << current_position[1] << "\n";
        start_.pose.position.x = current_position[0];
        start_.pose.position.y = current_position[1];
        start_.pose.position.z = 0;
        start_.pose.orientation.w = 0;
        start_.pose.orientation.x = 0;
        start_.pose.orientation.y = 0;
        start_.pose.orientation.z = 1;
    }

    /// Update the end position to the next index in the sequence
    void update_end()
    {
        ROS_INFO("End Position");
        std::cout << sequence_[current_goal_index_][0] << " " << sequence_[current_goal_index_][1] << "\n";
        end_.pose.position.x = sequence_[current_goal_index_][0];
        end_.pose.position.y = sequence_[current_goal_index_][1];
        end_.pose.position.z = 0;
        end_.pose.orientation.w = 0;
        end_.pose.orientation.x = 0;
        end_.pose.orientation.y = 0;
        end_.pose.orientation.z = 1;
        current_goal_index_++;
    }

    /// Finds and returns the plan between start and end by calling the planner service
    std::vector<PlannerNode> find_plan()
    {
        srv_message_.request.start_position = start_;
        srv_message_.request.end_position = end_;
        srv_message_.request.update_samples = true;
        srv_message_.request.update_map = false;

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
