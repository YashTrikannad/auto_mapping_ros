//
// Created by yash on 2/19/20.
//

#ifndef AUTO_MAPPING_ROS_UTILS_H
#define AUTO_MAPPING_ROS_UTILS_H

#include <boost/algorithm/string.hpp>
#include <fstream>

#include "auto_mapping_ros/graph_builder.h"
#include "auto_mapping_ros/types.h"

namespace amr
{

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
    double t = 15;
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
        t=t-0.8;
    }

    namedWindow( "Visual Graph", cv::WINDOW_AUTOSIZE);
    imshow( "Visual Graph", visual_graph );
    cv::waitKey(0);
}

void write_sequence_to_csv(const std::vector<std::array<int,2>>& sequence, std::string filename)
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

void read_sequence_from_csv(std::vector<std::array<int,2>>* sequence, const std::string& filename = "sequence.csv")
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

}


#endif //AUTO_MAPPING_ROS_UTILS_H
