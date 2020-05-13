#ifndef AUTO_MAPPING_ROS_SKELETONIZER_H
#define AUTO_MAPPING_ROS_SKELETONIZER_H

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#ifndef DEBUG
#define DEBUG 1
#endif

namespace amr
{

class Skeletonizer
{
public:
    Skeletonizer();

    int read_map(const std::string &map_filename);

    void display_img(cv::Mat img, std::string name = "Skeletonized Image");

    cv::Mat skeletonize();

private:
    cv::Mat map_jpg;
};

}

#endif //AUTO_MAPPING_ROS_SKELETONIZER_H

#include "impl/skeletonizer_impl.h"