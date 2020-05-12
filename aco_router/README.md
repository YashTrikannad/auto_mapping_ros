# aco_router
Solves routing problems for coverage planning using Ant Colony Optimization based algorithms (tested on Google OR Tools dataset):
1. Traveling Salesman Problem
2. Vehicle Routing Problem

The input graph needs to be in this format: 

```
struct Graph
    {
    private:
        std::vector<Node> graph_;
        .
        .

```

You must use the two public helper functions to create the graph.
```
    public:
        /**
         * Create a new node in Graph with co-ordinates x and y
         * @param x - x coordinate
         * @param y - y coordinate
         * @return node id created
         */
        int create_node_in_graph(double x, double y);
        
        /**
         * Add edge (onw way) from node_id_from to node_id_to in the graph
         * @param node_id_from
         * @param node_id_to
         */
        void add_edge(int node_id_from, int node_id_to);
        .
        .
    };
```

The test example shows an example of how a graph is created. After creating the graph, you need to set the parameters of the algorithm and then call tsp_solver.
```
    // Set Parameters of the Ant Colony Optimization Problem
    aco::AcoParams params{.n_ants = 2, .max_iters = 5, .alpha=1, .beta=1, .rho=0.05};

    // Solve the TSP using Ant Colony Optimization
    std::vector<aco::Node> best_route = aco::solve_tsp(graph, params);
```


Parameters of Ant Colony Optimization that the user needs to set:
```
    struct AcoParams
    {
        int n_ants; // Number of ants in each iteration 
        int max_iters; // Max number of iterations
        double alpha; // Exponential for weighting the pheromone value 
        double beta; // Exponential for weighting the desirability of paths
        double rho; // Evaporation Rate
    };
```
