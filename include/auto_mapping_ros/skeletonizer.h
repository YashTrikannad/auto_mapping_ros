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
    Skeletonizer() : map_jpg()
    {}

    int read_map(const std::string &map_filename)
    {
        map_jpg = cv::imread(map_filename, 0);
        if (map_jpg.data == nullptr)
        {
            std::cout << "Failed to read the image file: " << map_filename << std::endl;
            return -1;
        }
        return 0;
    }

    void display_img(cv::Mat img, std::string name = "Skeletonized Image")
    {
        namedWindow(name, cv::WINDOW_AUTOSIZE);
        cv::imshow(name, img);
        cv::waitKey(0);
    }

    cv::Mat skeletonize()
    {
        cv::threshold(map_jpg, map_jpg, 230, 255, cv::THRESH_BINARY);
        cv::Mat skel(map_jpg.size(), CV_8UC1, cv::Scalar(0));
        cv::Mat temp(map_jpg.size(), CV_8UC1);

        cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(7, 7));

        bool done;
        do
        {
            cv::morphologyEx(map_jpg, temp, cv::MORPH_OPEN, element);
            cv::bitwise_not(temp, temp);

            cv::bitwise_and(map_jpg, temp, temp);
            cv::bitwise_or(skel, temp, skel);
            cv::erode(map_jpg, map_jpg, element);

            double max;
            cv::minMaxLoc(map_jpg, nullptr, &max);
            done = (max == 0);

        } while (!done);

        if(DEBUG)
        {
            display_img(skel);
        }


        return skel;
    }

private:
    cv::Mat map_jpg;
};

}

#endif //AUTO_MAPPING_ROS_SKELETONIZER_H
