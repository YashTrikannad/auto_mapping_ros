#define DEBUG 1

#include <ros/ros.h>
#include <ros/package.h>

#include "auto_mapping_ros/graph_builder.h"
#include "auto_mapping_ros/landmarks.h"
#include "auto_mapping_ros/skeletonizer.h"
#include "auto_mapping_ros/utils.h"

static constexpr int radius_of_interest = 20;
static constexpr bool use_real_map = true;

void print_cv_matrix(const cv::Mat& mat)
{
    for(int i=0; i<mat.rows; i++)
    {
        for(int j=0; j<mat.cols; j++)
        {
            std::cout << static_cast<int>(mat.at<uchar>(i, j)) << "\t ";
        }
        std::cout << "\n";
    }
}

int main()
{
    if(use_real_map)
    {
        const auto filepath = ros::package::getPath("auto_mapping_ros") + "/maps/levine.jpg";

        amr::Skeletonizer processor;
        processor.read_map(filepath);
        cv::Mat skeleton = processor.skeletonize();

        cv::Mat map = cv::imread(filepath, 0);

        assert(skeleton.rows == map.rows);
        assert(skeleton.cols == map.cols);

        amr::GraphBuilder builder(skeleton, map);
        builder.build_graph();
        auto graph = builder.get_graph();

        amr::FrontierFinder finder;

        map.convertTo(map, CV_8U);

        for (const auto &node: graph)
        {
            const auto ray_casted_map = finder.ray_cast_to_2d_sub_map(node.get_location(), map);
//            finder.find_frontiers(node.get_location(), map);
            namedWindow("Ray Casted Map", cv::WINDOW_AUTOSIZE);
            cv::imshow("Ray Casted Map", ray_casted_map);
            cv::waitKey(0);
        }
    }
    else
    {
        cv::Mat test_matrix  = cv::Mat(10, 15, CV_8U, cv::Scalar(216));
        for(int i = 0; i<test_matrix.cols; i++)
        {
            test_matrix.at<uchar>(i, 2) = 0;
            test_matrix.at<uchar>(i, 7) = 0;
            test_matrix.at<uchar>(i, 0) = 216;
            test_matrix.at<uchar>(i, 1) = 216;
            test_matrix.at<uchar>(i, 8) = 216;
            test_matrix.at<uchar>(i, 9) = 216;
            test_matrix.at<uchar>(2, i) = 0;
            test_matrix.at<uchar>(7, i) = 0;
        }
        print_cv_matrix(test_matrix);

        amr::FrontierFinder finder;
        const auto ray_casted_map = finder.ray_cast_to_2d_map({5, 5}, test_matrix);
        std::cout << std::endl;
        print_cv_matrix(ray_casted_map);
    }

    return 0;
}

