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

    void compute_sequence_maximum()
    {
        construct_MaxST();
        sequence_ = preorder_traverse(MST_root);
    }

    void compute_sequence_edge()
    {
        explore_edges();
    }

    Sequence get_sequence()
    {
        if(sequence_.empty()) std::__throw_logic_error("Use compute_sequence before returning.");
        return sequence_;
    }

    void save_sequence()
    {
    }

private:

    Graph* graph_;
    Node* MST_root;
    Sequence sequence_;

    void construct_MST()
    {
        // stores nodes belonging to mst
        std::unordered_set<Node, NodeHasher> mst_set;
        std::unordered_set<Node*> open_set;

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
            open_set.insert(&node);
        }

        while(!open_set.empty())
        {
            // get minimum key node from queue and add to mst
            Node* min_node = nullptr;
            double min_node_distance = std::numeric_limits<double>::max();
            for(const auto& node: open_set)
            {
                if(node->mst_key < min_node_distance)
                {
                    min_node = node;
                    min_node_distance = node->mst_key;
                }
            }

            if(min_node == nullptr)
            {
                std::__throw_logic_error("Wrong Minimum from Container");
            }

            Node* curr_node = min_node;
            mst_set.insert(*curr_node);
            open_set.erase(min_node);

            // loop through all neighbours of current node
            for(auto i =0; i< curr_node->neighbors.size(); i++)
            {
                // if neighbour not already in mst
                if(mst_set.find(*curr_node->neighbors[i]) == mst_set.end())
                {
                    // neighbours key = cost b/w curr and neighbour
                    curr_node->neighbors[i]->mst_key = curr_node->neighbors_cost[i];

                    // make neighbour as child of current in mst
                    curr_node->mst_child.emplace_back(curr_node->neighbors[i]);
                }
            }
        }
    }

    void construct_MaxST() {
        // stores nodes belonging to mst
        std::unordered_set <Node, NodeHasher> mst_set;
        std::unordered_set <Node *> open_set;

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
            open_set.insert(&node);
        }

        while(!open_set.empty())
        {

            Node* min_node = nullptr;
            double min_node_distance = std::numeric_limits<int>::min();
            for(const auto& node: open_set)
            {
                if(node->mst_key > min_node_distance && node->mst_key < std::numeric_limits<int>::max())
                {
                    min_node = node;
                    min_node_distance = node->mst_key;
                }
            }

            if(min_node == nullptr)
            {
                std::__throw_logic_error("Wrong Minimum from Container");
            }

            Node* curr_node = min_node;
            mst_set.insert(*curr_node);
            open_set.erase(min_node);

            // loop through all neighbours of current node
            for(auto i = 0; i < curr_node->neighbors.size(); i++)
            {
                // if neighbour not already in mst
                if(mst_set.find(*curr_node->neighbors[i]) == mst_set.end())
                {
                    // neighbours key = cost b/w curr and neighbour
                    curr_node->neighbors[i]->mst_key = curr_node->neighbors_cost[i];

                    // make neighbour as child of current in mst
                    curr_node->mst_child.emplace_back(curr_node->neighbors[i]);
                }
            }
        }
    }

    void explore_edges()
    {
        bool first = true;
        for(auto &node: *graph_)
        {
            if (first)
            {
                MST_root = &node;
                first = false;
            }
        }

        if(MST_root == nullptr) { return; }

        explore_edges_recursive(nullptr, MST_root);
    }

    void explore_edges_recursive(Node* prev, Node* curr)
    {

        std::cout << "RECURSIVE CALL" << std::endl;

        if (curr == nullptr) { return; }
        std::array<int, 2> coords {curr->x, curr->y};
        sequence_.push_back(coords);

        if (prev != nullptr)
        {
            for (int i = 0; i < curr->neighbors.size(); i++)
            {
                if (curr->neighbors[i] == prev)
                {
                    curr->visited_neighbor[i] = true;
                }
            }
        }

        for (int i = 0; i < curr->neighbors.size(); i++)
        {

            if (!(curr->visited_neighbor[i]))
            {
                curr->visited_neighbor[i] = true;
                explore_edges_recursive(curr, curr->neighbors[i]);
            }

        }

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

#endif
