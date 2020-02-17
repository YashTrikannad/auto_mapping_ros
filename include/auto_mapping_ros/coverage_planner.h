#ifndef AUTO_MAPPING_ROS_COVERAGE_PLANNER_H
#define AUTO_MAPPING_ROS_COVERAGE_PLANNER_H

#include "graph_builder.h"
#include <queue>

namespace amr
{
    struct MSTNode
    {
        explicit MSTNode(const Node &n) : node(&n), key(INT_MAX)
        {}

        const Node* node;
        int key;
        std::vector<MSTNode*> mst_child;

    };

    struct MST_key_greater_than {
        bool operator()(MSTNode const& a, MSTNode const& b) const {
            return a.key > b.key;
        }
    };

using Sequence = std::vector<std::array<int, 2>>;

class CoveragePlanner
{
public:
    CoveragePlanner(Graph graph): graph_(std::move(graph))
    {}

    //TODO: Implement Coverage Planning algorithm to solve TSP
    void compute_sequence()
    {
        compute_MST();
    }

    Sequence get_sequence()
    {
        if(sequence_.empty()) std::__throw_logic_error("Use compute_sequence before returning.");
        return sequence_;
    }

private:
    Graph graph_;
    Sequence sequence_;

    void compute_MST()
    {
        std::vector<MSTNode*> open_set;
        std::vector<MSTNode*> MST;

        for(auto& node: graph_)
        {
            MSTNode mstnode(node);
            open_set.emplace_back(&mstnode);
        }

        open_set[0]->key = 0;
        std::make_heap(open_set.begin(), open_set.end(), MST_key_greater_than());



        while(!open_set.empty())
        {
            MSTNode* min_key_node = open_set.front();
            std::pop_heap (open_set.begin(),open_set.end());
            open_set.pop_back();


        }

    }
};

}

#endif //AUTO_MAPPING_ROS_COVERAGE_PLANNER_H
