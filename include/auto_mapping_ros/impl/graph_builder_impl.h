#ifndef AUTO_MAPPING_ROS_GRAPH_BUILDER_IMPL_H
#define AUTO_MAPPING_ROS_GRAPH_BUILDER_IMPL_H

#include <iostream>
#include <unordered_set>

#include "auto_mapping_ros/graph_builder.h"

namespace amr
{
    std::vector<std::array<int, 2>> GraphBuilder::boundary_corners_ = {};

/// Initialize the configuration of the graph
    void GraphBuilder::init_config() {
        libconfig::Config cfg;
        try {
            const std::string filename = get_package_directory() + "/config/graph_builder.cfg";
            char *tab2 = new char[filename.length() + 1];
            strcpy(tab2, filename.c_str());
            cfg.readFile(tab2);
        }
        catch (const libconfig::FileIOException &fioex) {
            std::__throw_invalid_argument("I/O error while reading file.");
        }

        try {
            dilation_size_ = cfg.lookup("dilation_size");
            block_size_ = cfg.lookup("blockSize");
            aperture_size_ = cfg.lookup("apertureSize");
            k_ = cfg.lookup("k");
            distance_threshold_ = cfg.lookup("distance_threshold");
            obstacle_threshold_ = cfg.lookup("obstacle_threshold");
        }
        catch (const libconfig::SettingNotFoundException &nfex) {
            std::cerr << "Missing setting in configuration file." << std::endl;
        }
    }

/// Builds the graph
    void GraphBuilder::build_graph()
    {
        register_boundary();
        const auto corners = find_corners();
        construct_graph(corners);
        if (DEBUG) {
            visualize_graph(map_, graph_);
        }
    }

/// Get the graph if it is constructed
/// @return Built graph
    Graph GraphBuilder::get_graph() {
        if (graph_.empty()) {
            std::__throw_invalid_argument("Graph needs to be built before using it.");
        }
        return std::move(graph_);
    }

/// Callback function for registering two ends of rectangular boundary using LMB
/// \param event
/// \param x
/// \param y
/// \param flags
/// \param userdata
    void GraphBuilder::CallBackFunc(int event, int x, int y, int flags, void *userdata) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            std::cout << "Boundary corner selected: (" << x << ", " << y << ") \n";
            boundary_corners_.emplace_back(std::array<int, 2>{x, y});
        }
    }

/// Register boundary around the map where the graph is built
    void GraphBuilder::register_boundary() {
        const std::string window_name = "Functional Area Registration";
        cv::namedWindow(window_name, 1);
        cv::setMouseCallback(window_name, this->CallBackFunc, NULL);
        while (true) {
            cv::imshow(window_name, map_);
            if (cv::waitKey(1) & boundary_corners_.size() == 2) break;
        }
        cv::destroyWindow(window_name);
    }

/// Performs morphological operation of dilation on the input image
/// @param img - input image to be diluted
/// @return
    cv::Mat GraphBuilder::dilate(const cv::Mat &img) const {
        const int dilation_size = dilation_size_;
        const int dilation_type = cv::MORPH_RECT;
        cv::Mat element = getStructuringElement(dilation_type,
                                                cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                cv::Point(dilation_size, dilation_size));

        cv::Mat dilated_dilated_img;
        cv::dilate(img, dilated_dilated_img, element);
        return dilated_dilated_img;
    }

/// Computes the centers of multiple blobs in a binary image
/// @param binary_image Input Image (Binary/GrayScale)
/// @return vector of centroids of the white blobs in the input image
    std::vector<std::array<int, 2>> GraphBuilder::compute_blob_centers(const cv::Mat &binary_image) const {
        cv::Mat canny_output;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;

        // detect edges using canny
        cv::Canny(binary_image, canny_output, 50, 150, 3);

        // find contours
        cv::findContours(canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        // get the moments
        std::vector<cv::Moments> mu(contours.size());
        for (size_t i = 0; i < contours.size(); i++) {
            mu[i] = moments(contours[i], false);
        }

        // get the centroid of figures.
        std::vector<cv::Point2f> mc(contours.size());
        for (auto i = 0; i < contours.size(); i++) {
            mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
        }

        if (DEBUG) {
            // draw contours
            cv::Mat drawing(canny_output.size(), CV_8UC3, cv::Scalar(255, 255, 255));
            for (size_t i = 0; i < contours.size(); i++) {
                cv::Scalar color = cv::Scalar(167, 151, 0); // B G R values
                drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
                circle(drawing, mc[i], 4, color, -1, 8, 0);
            }

            // show the resultant image
            namedWindow("Contours", cv::WINDOW_AUTOSIZE);
            imshow("Contours", drawing);
            cv::waitKey(0);
        }

        std::vector<std::array<int, 2>> centroids;
        for (const auto &mc_points: mc) {
            centroids.emplace_back(std::array<int, 2>(
                    {static_cast<int>(mc_points.y), static_cast<int>(mc_points.x)}));
        }

        return centroids;
    }

/// Filters the corners vector by finding sparse matrices using morphological techniques
/// @param corners - vector of features detected
/// @return sparse vector of features
    std::vector<std::array<int, 2>> GraphBuilder::find_sparse_centroid(
            const std::vector<std::array<int, 2>> &corners) const {
        cv::Mat dense_corner_img(map_.size(), CV_8UC1, cv::Scalar(0));
        for (const auto &corner: corners) {
            dense_corner_img.at<uchar>(corner[0], corner[1]) = 255;
        }

//        const auto temp = compute_blob_centers(dense_corner_img);

        const auto dilated_dense_corner_img = dilate(dense_corner_img);
        return compute_blob_centers(dilated_dense_corner_img);
    }

    bool GraphBuilder::is_within_specified_region(int x, int y) {
        return x > boundary_corners_[0][0] && x < boundary_corners_[1][0]
               && y > boundary_corners_[0][1] && y < boundary_corners_[1][1];
    }

/// Finds all the important features/corners in the blueprint which can be used as nodes
/// @return vector of corners/features
    std::vector<std::array<int, 2>> GraphBuilder::find_corners() const {
        cv::Mat dst = cv::Mat::zeros(skeletonized_image_.size(), CV_32FC1);
        cv::cornerHarris(skeletonized_image_, dst, block_size_, aperture_size_, k_);

        cv::Mat dst_norm, dst_norm_scaled;
        normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
        convertScaleAbs(dst_norm, dst_norm_scaled);

        namedWindow("harris corner", cv::WINDOW_AUTOSIZE);
        cv::imshow("harris corner", dst_norm_scaled);
        cv::waitKey(0);

        std::vector<std::array<int, 2>> corner_points;

        for (int i = 0; i < dst_norm.rows; i++) {
            for (int j = 0; j < dst_norm.cols; j++) {
                if (static_cast<int>(dst_norm.at<float>(i, j)) > 100 && is_within_specified_region(j, i)) {
                    corner_points.emplace_back(std::array<int, 2>{i, j});
                }
            }
        }

        const auto dense_corner_centroids = find_sparse_centroid(corner_points);

        std::vector<std::array<int, 2>> unique_sparse_centroids;
        std::set<std::array<int, 2>> sparse_centroid_set(dense_corner_centroids.begin(), dense_corner_centroids.end());
        unique_sparse_centroids.assign(sparse_centroid_set.begin(), sparse_centroid_set.end());

        std::cout << "There are " << unique_sparse_centroids.size() << " features detected in this image.\n";
        return unique_sparse_centroids;
    }

/// Checks collision between two nodes using the DDA Line algorithm
/// @param current_node
/// @param neighbor_node
/// @return true if there is a collision else false
    bool GraphBuilder::check_collision(const Node &current_node, const Node &neighbor_node) const {
        if(current_node.x == 535 && neighbor_node.y == 996)
        {
            int x =1;
        }
        const int dx = neighbor_node.x - current_node.x;
        const int dy = neighbor_node.y - current_node.y;

        int steps = 0;
        if (std::abs(dx) > std::abs(dy)) {
            steps = std::abs(dx);
        } else {
            steps = std::abs(dy);
        }

        double x_increment = dx / static_cast<double>(steps);
        double y_increment = dy / static_cast<double>(steps);

        double intermediate_x = current_node.x;
        double intermediate_y = current_node.y;
        for (size_t v = 0; v < steps; v++) {
            intermediate_x = intermediate_x + x_increment;
            intermediate_y = intermediate_y + y_increment;
            const auto pixel_value = static_cast<int>(
                    map_.at<uchar>(static_cast<int>(intermediate_x), static_cast<int>(intermediate_y)));
            if (pixel_value < obstacle_threshold_) {
                return true;
            }
        }
        assert(intermediate_x = neighbor_node.x);
        assert(intermediate_y = neighbor_node.y);
        return false;
    }

/// Constructs the graph using the input feature cells (corners)
/// @param corners
    void GraphBuilder::construct_graph(const std::vector<std::array<int, 2>> &corners) {
        for (const auto &corner: corners) {
            Node node(corner, unique_id_gen_++);
            graph_.push_back(std::move(node));
        }
        for (auto &node: graph_)
        {
            std::vector<Node *> neighbor_nodes;
            std::vector<double> neighbor_nodes_cost;
            for (auto &candidate_neighbor_node: graph_) {
                if (node == candidate_neighbor_node || check_collision(node, candidate_neighbor_node)) {
                    continue;
                }
                neighbor_nodes.emplace_back(&candidate_neighbor_node);
            }

            node.neighbors = neighbor_nodes;
            node.neighbors_cost = neighbor_nodes_cost;
        }

        trim_edges();

        graph_ = find_largest_connected_component();

        for (auto &node: graph_)
        {
            std::vector<double> neighbor_nodes_cost;
            for (const auto &neighbor_node: node.neighbors) {
                const auto distance = static_cast<double>(
                        sqrt(pow(node.y - neighbor_node->y, 2) + pow(node.x - neighbor_node->x, 2)));
                neighbor_nodes_cost.emplace_back(distance);
            }
            node.neighbors_cost = neighbor_nodes_cost;
        }
    }

/// Trims away edges to reduce overlapping edges
    void GraphBuilder::trim_edges() {
        for (auto &current_node: graph_) {
            const int current_node_neighbors_size = current_node.neighbors.size();

            std::vector<bool> is_valid(current_node_neighbors_size, true);
            std::vector<Node *> new_current_node_neighbors;

            for (const auto &neighbor_node: current_node.neighbors) {
                if (is_another_node_in_between(&current_node, neighbor_node)) {
                    continue;
                }
                new_current_node_neighbors.emplace_back(neighbor_node);
            }
            current_node.neighbors = new_current_node_neighbors;
        }
    }

/// Checks if any neighbors of the current node lie between the current node and the neighbor node
/// @param current_node - current node of the graph
/// @param neighbor_node - neighbor node of the current node of the graph
/// @return true if a neighbor of current node lies within a threshold distance of the two input nodes
    bool GraphBuilder::is_another_node_in_between(Node *current_node, Node *neighbor_node) const {
        double threshold = distance_threshold_;
        for (const auto &current_node_neighbor: current_node->neighbors) {
            if (current_node_neighbor == neighbor_node) continue;
            const auto dist = pDistance(current_node_neighbor->x, current_node_neighbor->y,
                                        current_node->x, current_node->y, neighbor_node->x, neighbor_node->y);
            if (dist < threshold) {
                return true;
            }
        }
        return false;
    }

/// Finds the distance between a point and line segment
/// @param x_point - x co-ordinate of point
/// @param y_point - y co-ordinate of point
/// @param x_line_segment_start - x co-ordinate of start of line segment
/// @param y_line_segment_start - y co-ordinate of start of line segment
/// @param x_line_segment_end - x co-ordinate of end of line segment
/// @param y_line_segment_end - y co-ordinate of end of line segment
/// @return distance between point and line segment
    double GraphBuilder::pDistance(double x_point, double y_point,
                                        double x_line_segment_start, double y_line_segment_start,
                                        double x_line_segment_end, double y_line_segment_end) const {

        double A = x_point - x_line_segment_start;
        double B = y_point - y_line_segment_start;
        double C = x_line_segment_end - x_line_segment_start;
        double D = y_line_segment_end - y_line_segment_start;

        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        double param = -1;

        if (len_sq != 0)
            param = dot / len_sq;

        double xx, yy;

        if (param < 0) {
            xx = x_line_segment_start;
            yy = y_line_segment_start;
        } else if (param > 1) {
            xx = x_line_segment_end;
            yy = y_line_segment_end;
        } else {
            xx = x_line_segment_start + param * C;
            yy = y_line_segment_start + param * D;
        }

        auto dx = x_point - xx;
        auto dy = y_point - yy;
        return sqrt(dx * dx + dy * dy);
    }

    void GraphBuilder::run_dfs(Node* node,
                               std::unordered_set<int>& visited_nodes,
                               std::vector<Node>& current_connected_component)
    {
        // Node has already been visited
        if(visited_nodes.find(node->id) != visited_nodes.end()) return;

        // Add the current node to the visited
        visited_nodes.insert(node->id);
        current_connected_component.emplace_back(Node(node->x, node->y, node->id));

        for(const auto& ngbr_node: node->neighbors)
        {
            run_dfs(ngbr_node, visited_nodes, current_connected_component);
        }
    }

    void GraphBuilder::graph_subset(std::vector<Node>& subset_nodes)
    {
        auto get_node_ptr = [&](int id, Graph& graph){
            for(auto& node: graph) if(node.id == id) return &node;
        };

        for(int i=0; i<subset_nodes.size(); i++)
        {
            std::vector<Node*> neighbors;
            Node* main_graph_current_node = get_node_ptr(subset_nodes[i].id, graph_);

            for(const auto& neighbor_node : main_graph_current_node->neighbors)
            {
                neighbors.emplace_back(get_node_ptr(neighbor_node->id, subset_nodes));
            }
            subset_nodes[i].neighbors = neighbors;
        }
    }

    /// Finds the largest connected component in the graph
    /// @details This function also creates new ids
    Graph GraphBuilder::find_largest_connected_component()
    {
        std::vector<Node> largest_connected_component;
        std::vector<Node> current_connected_component;
        std::unordered_set<int> visited_nodes{};
        for(auto& node: graph_)
        {
            current_connected_component.clear();
            run_dfs(&node, visited_nodes, current_connected_component);
            if(current_connected_component.size() > largest_connected_component.size())
            {
                largest_connected_component = current_connected_component;
            }
        }

        graph_subset(largest_connected_component);
        return largest_connected_component;
    }
}

#endif //AUTO_MAPPING_ROS_GRAPH_BUILDER_IMPL_H
