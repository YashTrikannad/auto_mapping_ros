#ifndef AUTO_MAPPING_ROS_COVERAGE_PLANNER_H
#define AUTO_MAPPING_ROS_COVERAGE_PLANNER_H

#include "graph_builder.h"

#ifndef DEBUG
#define DEBUG 1
#endif

namespace amr
{

using Sequence = std::vector<std::array<int, 2>>;

class CoveragePlanner
{
public:
    CoveragePlanner(Graph graph): graph_(std::move(graph))
    {}

    //TODO: Implement Coverage Planning algorithm to solve TSP
    void compute_sequence();

    Sequence get_sequence()
    {
        if(sequence_.empty()) std::__throw_logic_error("Use compute_sequence before returning.");
        return sequence_;
    }

private:
    Graph graph_;
    Sequence sequence_;
};

}

#endif //AUTO_MAPPING_ROS_COVERAGE_PLANNER_H
