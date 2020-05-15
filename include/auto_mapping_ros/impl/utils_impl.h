#ifndef AUTO_MAPPING_ROS_UTILS_IMPL_H
#define AUTO_MAPPING_ROS_UTILS_IMPL_H

#include "auto_mapping_ros/types.h"
#include "auto_mapping_ros/utils.h"

#include "../aco_router/types.h"

namespace amr
{

    void print_vector_of_nodes(const std::vector<std::array<double, 2>>& vector_of_nodes)
    {
        for(const auto& node: vector_of_nodes)
        {
            std::cout << node[0] << ", " << node[1] << "\n";
        }
    }

    void print_graph(const Graph& graph)
    {
        std::cout << "Printing out the Graph \n";
        for(const auto& node: graph)
        {
            std::cout << "Node " << node.id  << ", (x: " << node.x << ", y: " << node.y << ") : ";
            for(int i=0; i<node.neighbors.size(); i++)
            {
                std::cout << "\t{" << node.neighbors[i]->id << ", "<< node.neighbors_cost[i]
                          << ", (x: " << node.neighbors[i]->x << ", y: " << node.neighbors[i][0].y << ") }";
            }
            std::cout << "\n";
        }
    }

    void print_graph_with_new_ids(Graph& graph)
    {
        int i = 0;
        for(auto &node: graph)
        {
            node.id = i++;
        }
        print_graph(graph);
    }

    void visualize_graph(const cv::Mat& map, const Graph& graph)
    {
        cv::Mat visual_graph(map.size(), CV_8UC3, cv::Vec3b(0, 0, 0));

        for(int i=0; i < map.rows; i++)
        {
            for(int j=0; j < map.cols; j++)
            {
                if (map.at<uchar>(i, j) > 200)
                {
                    visual_graph.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
                }
            }
        }

        for(const auto& node : graph)
        {
            cv::circle(visual_graph, {node.y, node.x} , 4, cv::Scalar(100, 0, 0));
            for(const auto& neighbor: node.neighbors)
            {
                cv::line(visual_graph, {node.y, node.x}, {neighbor->y, neighbor->x}, cv::Vec3b(0, 0, 100));
            }
        }

        namedWindow( "Visual Graph", cv::WINDOW_AUTOSIZE);
        imshow( "Visual Graph", visual_graph );
        cv::waitKey(0);
    }

    static std::array<int, 2> clicked_point;
    static bool clicked;
    void get_clicked_point(int event, int x, int y, int flags, void *userdata)
    {
        if (event == cv::EVENT_LBUTTONDOWN)
        {
            clicked_point = std::array<int, 2>{y, x};
            std::cout << "Registered click at: (" << clicked_point[0] << ", " << clicked_point[1] << ")\n";
            clicked = true;
        }
    }

    int get_closest_clicked_node_on_map(const cv::Mat& map, aco::Graph& graph)
    {
        clicked_point = {};
        clicked = false;
        cv::Mat visual_graph(map.size(), CV_8UC3, cv::Vec3b(0, 0, 0));

        for(int i=0; i < map.rows; i++)
        {
            for(int j=0; j < map.cols; j++)
            {
                if (map.at<uchar>(i, j) > 200)
                {
                    visual_graph.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
                }
            }
        }

        for(const auto& node : graph)
        {
            cv::circle(visual_graph, {static_cast<int>(node.y), static_cast<int>(node.x)} , 4, cv::Scalar(100, 0, 0));
        }

        const std::string window_name = "Initial Depot Registration";
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(window_name, get_clicked_point, NULL);
        while (true)
        {
            cv::imshow(window_name, visual_graph);
            if (cv::waitKey(1) && clicked==true) break;
        }
        clicked = false;
        cv::destroyWindow(window_name);

        int closest_node_id = -1;
        double closest_node_distance = std::numeric_limits<double>::max();
        for(const auto& node : graph)
        {
            double distance = sqrt(pow(node.x - clicked_point[0], 2) + pow(node.y - clicked_point[1], 2));
            if(distance < closest_node_distance)
            {
                closest_node_distance = distance;
                closest_node_id = node.id;
            }
        }
        return closest_node_id;
    }

    void visualize_sequence_on_graph(
            const cv::Mat& map, const Graph& graph, const std::vector<std::array<int, 2>>& sequence)
    {
        cv::Mat visual_graph(map.size(), CV_8UC3, cv::Vec3b(0, 0, 0));

        for(int i=0; i < map.rows; i++)
        {
            for(int j=0; j < map.cols; j++)
            {
                if (map.at<uchar>(i, j) > 200)
                {
                    visual_graph.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
                }
            }
        }

        for(const auto& node : graph)
        {
            cv::circle(visual_graph, {node.y, node.x} , 4, cv::Scalar(100, 0, 0));
            for(const auto& neighbor: node.neighbors)
            {
                cv::line(visual_graph, {node.y, node.x}, {neighbor->y, neighbor->x}, cv::Vec3b(0, 0, 100));
            }
        }

        cv::Vec3b color = cv::Vec3b(0, 255, 0);
        int x = 1;
        double t = 5;
        for(int i=0; i<sequence.size(); i++)
        {
            cv::circle(visual_graph, {sequence[i][1], sequence[i][0]} , 4, cv::Scalar(100, 0, 0));
            if(i != 0)
            {
                cv::line(visual_graph,  {sequence[i][1], sequence[i][0]},
                         {sequence[i-1][1], sequence[i-1][0]}, color, static_cast<int>(t));
            }
            color[x] = color[x] - 10;
            if(color[x] < 5)
            {
                color[x] = 255;
            }
        }

        namedWindow( "Visual Graph", cv::WINDOW_AUTOSIZE);
        imshow( "Visual Graph", visual_graph );
        cv::waitKey(0);
    }

    template <typename T>
    void write_sequence_to_csv(const std::vector<std::array<T,2>>& sequence, std::string filename)
    {
        std::ofstream file_to_write;
        file_to_write.open(filename);
        if(!file_to_write)
        {
            throw std::runtime_error("Invalid Path for csv file.");
        }
        for(const auto& node: sequence)
        {
            file_to_write << node[0] << ", " << node[1] << "\n";
        }
        file_to_write.close();
    }

    void write_plans_to_csv(const std::vector<Plan>& plans, std::string filename)
    {
        std::ofstream file_to_write;
        file_to_write.open(filename);
        if(!file_to_write)
        {
            throw std::runtime_error("Invalid Path for csv file.");
        }
        for(const auto& plan: plans)
        {
            file_to_write << plan.start[0] << ", " << plan.start[1] << "\n";
            file_to_write << plan.end[0] << ", " << plan.end[1] << "\n";
            for(const auto& node: plan.plan)
            {
                file_to_write << node[0] << ", " << node[1] << ", ";
            }
        }
        file_to_write.close();
    }

    void read_sequence_from_csv(std::vector<std::array<int,2>>* sequence, const std::string& filename)
    {
        std::ifstream file(filename);
        if(!file)
        {
            throw std::runtime_error("Invalid Path for csv file.");
        }

        std::string line = "";
        while (getline(file, line))
        {
            std::vector<std::string> vec;
            boost::algorithm::split(vec, line, boost::is_any_of(","));
            std::array<int,2> node{};
            node[0] = std::stod(vec[0]);
            node[1] = std::stod(vec[1]);
            sequence->emplace_back(node);
        }
    }

    template <typename Arithmetic>
    double distance(const std::array<Arithmetic, 2>& node1, const std::array<Arithmetic, 2>& node2)
    {
        return sqrt(pow(node1[0]-node2[0], 2) + pow(node1[1]-node2[1], 2));
    }

    std::string get_package_directory()
    {
        std::string package_name = "auto_mapping_ros";
        std::string cwd_str(__FILE__);
        std::string package_dir;
        std::string current_filename;
        for(char i : cwd_str)
        {
            if(i != '/')
            {
                current_filename += i;
                continue;
            }
            package_dir += current_filename + "/";
            if(current_filename == package_name)
            {
                package_dir.pop_back();
                break;
            }
            current_filename.clear();
        }
        return package_dir;
    }
}

#endif //AUTO_MAPPING_ROS_UTILS_IMPL_H