#ifndef AUTO_MAPPING_ROS_GRAPH_BUILDER_H
#define AUTO_MAPPING_ROS_GRAPH_BUILDER_H

#include <array>
#include <iostream>
#include <libconfig.h++>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <set>
#include <unordered_set>
#include <utility>

#include "auto_mapping_ros/landmarks.h"
#include "auto_mapping_ros/types.h"
#include "auto_mapping_ros/utils.h"

#ifndef DEBUG
#define DEBUG 1
#endif

namespace amr
{

void visualize_graph(const cv::Mat& map, const Graph& graph);

/// Class for constructing the graph from skeleton image and blueprint
class GraphBuilder
{
public:
    /// Constructs GraphBuilder class
    /// @param skeletonized_image - image obtained from skeletonization and subtracting the blueprint
    /// @param map - Original blueprint of the map
    GraphBuilder(cv::Mat skeletonized_image, cv::Mat map) :
            skeletonized_image_(std::move(skeletonized_image)), map_(std::move(map))
    {
        init_config();
    }

    // Initialize the parameters of the graph builder
    void init_config();

    /// Builds the graph
    void build_graph();

    /// Get the graph if it is constructed
    /// @return Built graph
    Graph get_graph();

    static std::vector<std::array<int, 2>> boundary_corners_;
    static void CallBackFunc(int event, int x, int y, int flags, void* userdata);

private:
    cv::Mat skeletonized_image_;
    cv::Mat map_;
    std::vector<Node> graph_;
    int unique_id_gen_ = 0;

    int dilation_size_;
    int block_size_;
    int aperture_size_;
    double k_;
    double distance_threshold_;
    int obstacle_threshold_;

    /// Register boundary around the map where the graph is built
    void register_boundary();

    /// Performs morphological operation of dilation on the input image
    /// @param img - input image to be diluted
    /// @return
    cv::Mat dilate(const cv::Mat& img) const;

    /// Computes the centers of multiple blobs in a binary image
    /// @param binary_image Input Image (Binary/GrayScale)
    /// @return vector of centroids of the white blobs in the input image
    std::vector<std::array<int, 2>> compute_blob_centers(const cv::Mat& binary_image) const;

    /// Filters the corners vector by finding sparse matrices using morphological techniques
    /// @param corners - vector of features detected
    /// @return sparse vector of features
    std::vector<std::array<int, 2>> find_sparse_centroid(
            const std::vector<std::array<int, 2>>& corners) const;

    static bool is_within_specified_region(int x, int y);

    /// Finds all the important features/corners in the blueprint which can be used as nodes
    /// @return vector of corners/features
    std::vector<std::array<int, 2>> find_corners() const;

    /// Finds the distance between a point and line segment
    /// @param x_point - x co-ordinate of point
    /// @param y_point - y co-ordinate of point
    /// @param x_line_segment_start - x co-ordinate of start of line segment
    /// @param y_line_segment_start - y co-ordinate of start of line segment
    /// @param x_line_segment_end - x co-ordinate of end of line segment
    /// @param y_line_segment_end - y co-ordinate of end of line segment
    /// @return distance between point and line segment
    double pDistance(double x_point, double y_point,
                     double x_line_segment_start, double y_line_segment_start,
                     double x_line_segment_end, double y_line_segment_end) const;

    /// Checks if any neighbors of the current node lie between the current node and the neighbor node
    /// @param current_node - current node of the graph
    /// @param neighbor_node - neighbor node of the current node of the graph
    /// @return true if a neighbor of current node lies within a threshold distance of the two input nodes
    bool is_another_node_in_between(Node* current_node, Node* neighbor_node) const;

    // TODO: Fix Bug in this function
    /// Trims away edges to reduce overlapping edges
    void trim_edges();

    /// Checks collision between two nodes using the DDA Line algorithm
    /// @param current_node
    /// @param neighbor_node
    /// @return true if there is a collision else false
    bool check_collision(const Node &current_node, const Node &neighbor_node) const;

    /// Constructs the graph using the input feature cells (corners)
    /// @param corners
    void construct_graph(const std::vector<std::array<int, 2>> &corners);

    void run_dfs(Node* node, std::unordered_set<int>& visited_nodes, std::vector<Node>& current_connected_component);

    void graph_subset(std::vector<Node>& subset_nodes);

    /// Finds the largest connected component in the graph
    /// @details This function also creates new ids
    Graph find_largest_connected_component();
};

}

#endif //AUTO_MAPPING_ROS_GRAPH_BUILDER_H

#include "impl/graph_builder_impl.h"