#ifndef AUTO_MAPPING_ROS_COVERAGE_PLANNER_H
#define AUTO_MAPPING_ROS_COVERAGE_PLANNER_H

#include "graph_builder.h"
#include <queue>
# include<unordered_set>
#ifndef DEBUG
#define DEBUG 1
#endif

namespace amr
{

using Sequence = std::vector<std::array<int, 2>>;

class CoveragePlanner
{
public:
    CoveragePlanner(Graph* graph): graph_(graph)
    {}

    //TODO: Implement Coverage Planning algorithm to solve TSP
    void compute_sequence()
    {
        construct_MST();
        sequence_ = preorder_traverse(MST_root);
    }

    Sequence get_sequence()
    {
        if(sequence_.empty()) std::__throw_logic_error("Use compute_sequence before returning.");
        return sequence_;
    }

private:
    Graph* graph_;
    Node* MST_root;
    Sequence sequence_;

    void construct_MST()
    {
        // stores nodes belonging to mst
        std::unordered_set<Node*> mst_set;

        // priority queue
        auto less = [](const Node *left, const Node *right) {
            return (left->mst_key) > (right->mst_key);
        };
        std::priority_queue<Node*, std::vector<Node*>, decltype(less)> open_queue(less);


        bool first = true;
        for(auto &node: *graph_)
        {
            // make root node key = 0
            if (first)
            {
                MST_root = &node;
                node.mst_key = 0;
                first = false;
            }
            // insert all nodes into queue
            open_queue.push(&node);
        }

        while(!open_queue.empty())
        {
            // get minimum key node from queue and add to mst
            Node* curr_node = open_queue.top();
            open_queue.pop();
            mst_set.insert(curr_node);

            std::cout<<"\n\nMin key "<<curr_node->mst_key;
            print_node(curr_node);


            // loop through all neighbours of current node
            for(int i =0; i< curr_node->neighbors.size(); i++)
            {
                // if neighbour not already in mst
                if(mst_set.find(curr_node->neighbors[i]) == mst_set.end())
                {


                    // neighbours key = cost b/w curr and neighbour
                    curr_node->neighbors[i]->mst_key = curr_node->neighbors_cost[i];

                    // make neighbour as child of current in mst
                    curr_node->mst_child.emplace_back(curr_node->neighbors[i]);

                    std::cout<<"\n\nparent";
                    print_node(curr_node);
                    std::cout<<"\nchild ,key: "<<curr_node->neighbors[i]->mst_key;
                    print_node(curr_node->neighbors[i]);
                }
            }
        }
    }

    void print_node(Node* node)
    {
        std::cout<<"\nx: "<<node->x<<" "<<"y: "<<node->y;
    }
    Sequence preorder_traverse(Node* root)
    {
        Sequence res;

        if(root==nullptr)
            return res;

        std::array<int, 2> root_coods{root->x,root->y};
        res.push_back(root_coods);

        for(auto &child: root->mst_child)
        {
            Sequence child_sequence = preorder_traverse(child);
            res.insert(res.end(), child_sequence.begin(), child_sequence.end());
        }

        return res;
    }

};

}

#endif AUTO_MAPPING_ROS_COVERAGE_PLANNER_H
