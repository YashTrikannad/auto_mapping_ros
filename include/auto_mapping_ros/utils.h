#ifndef AUTO_MAPPING_ROS_UTILS_H
#define AUTO_MAPPING_ROS_UTILS_H

#include "types.h"

namespace amr
{

void visualize_graph(const cv::Mat &map, const amr::Graph graph)
{
    cv::Mat visual_graph(map.size(), CV_8UC3, cv::Vec3b(0, 0, 0));

    for (int i = 0; i < map.rows; i++)
    {
        for (int j = 0; j < map.cols; j++)
        {
            if (map.at<uchar>(i, j) == 255)
            {
                visual_graph.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
            }
        }
    }

    for (const auto &node : graph)
    {
        cv::circle(visual_graph, {node.y, node.x}, 4, cv::Scalar(100, 0, 0));
        for (const auto &neighbor: node.neighbors)
        {
            cv::line(visual_graph, {node.y, node.x}, {neighbor->y, neighbor->x}, cv::Scalar(0, 0, 100));
        }
    }

    namedWindow("Visual Graph", cv::WINDOW_AUTOSIZE);
    imshow("Visual Graph", visual_graph);
    cv::waitKey(0);
}

}

#endif //AUTO_MAPPING_ROS_UTILS_H
