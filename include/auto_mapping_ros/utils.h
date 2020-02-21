//
// Created by yash on 2/19/20.
//

#ifndef AUTO_MAPPING_ROS_UTILS_H
#define AUTO_MAPPING_ROS_UTILS_H

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

}


#endif //AUTO_MAPPING_ROS_UTILS_H
