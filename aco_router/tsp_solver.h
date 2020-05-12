#ifndef ACO_TSP_TSP_SOLVER_H
#define ACO_TSP_TSP_SOLVER_H

#include <eigen3/Eigen/Dense>
#include <unordered_map>

#include "types.h"

namespace aco
{
    /**
     * Parameters for solving the Ant Colony Optimzation Problem for TSP
     */
    struct AcoParams
    {
        int n_ants = -1;
        int max_iters;
        double alpha;
        double beta;
        double rho;
    };

    /**
     * Function to solve the traveling salesman problem using the ant colony optimization
     * @param graph
     * @param params
     * @return
     */
    std::pair<std::vector<aco::Node>, double> solve_tsp(const Graph& graph, const AcoParams& params, int initial_node_id = -1);
}

#endif //ACO_TSP_TSP_SOLVER_H
