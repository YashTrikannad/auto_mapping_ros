#include "auto_mapping_ros/graph_builder.h"

int main()
{
    cv::Mat skeleton = cv::Mat::zeros(16,16, CV_8UC1);
    cv::Mat map = cv::Mat::zeros(16,16, CV_8UC1);

    for(int i=1; i < 15; i++)
    {
        map.at<uchar>(i, 1) = 255;
        map.at<uchar>(i, 14) = 255;
    }
    for(int i=1; i < 15; i++)
    {
        map.at<uchar>(1, i) = 255;
        map.at<uchar>(14, i) = 255;
    }
    for(int i=5; i < 11; i++)
    {
        map.at<uchar>(i, 5) = 255;
        map.at<uchar>(i, 10) = 255;
    }
    for(int i=5; i < 11; i++)
    {
        map.at<uchar>(5, i) = 255;
        map.at<uchar>(10, i) = 255;
    }
    for(int i=0; i<16; i++)
    {
        for (int j = 0; j < 16; j++)
        {
            std::cout << static_cast<int>(map.at<uchar>(i, j)) << "\t";
        }
        std::cout << "\n";
    }
    std::cout << "\n";

    for(int i=3; i < 13; i++)
    {
        skeleton.at<uchar>(i, 3) = 255;
        skeleton.at<uchar>(i, 12) = 255;
    }
    for(int i=3; i < 13; i++)
    {
        skeleton.at<uchar>(3, i) = 255;
        skeleton.at<uchar>(12, i) = 255;
    }
    for(int i=0; i<16; i++)
    {
        for (int j = 0; j < 16; j++)
        {
            std::cout << static_cast<int>(skeleton.at<uchar>(i, j)) << "\t";
        }
        std::cout << "\n";
    }
    std::cout << "\n";

    amr::GraphBuilder builder(skeleton, map);
    builder.build_graph();
    const auto graph = builder.get_graph();

    return 0;
}

