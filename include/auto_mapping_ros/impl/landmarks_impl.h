#ifndef AUTO_MAPPING_LANDMARKS_IMPL_H
#define AUTO_MAPPING_LANDMARKS_IMPL_H

#include "auto_mapping_ros/landmarks.h"

namespace amr
{

/// Finds frontiers around the point
/// @param point - (x, y) around where you want to find frontiers
/// @param map - CV image of the map
/// @return vector of the centers of frontiers
std::vector<std::array<double, 2>> find_frontiers(const std::array<int, 2>& point,
        const cv::Mat& map,
        int radius_of_interest)
{
    return {};
}

/// Finds the number of frontiers around the point
/// @param point - (x, y) around where you want to find frontiers
/// @param map - CV image of the map
/// @return number of frontiers
int find_n_frontiers(const std::array<int, 2>& point,
        const cv::Mat& map,
        int radius_of_interest)
{
    return find_frontiers(point, map, radius_of_interest).size();
}

}

#endif //FMT_STAR_LANDMARKS_IMPL_H
