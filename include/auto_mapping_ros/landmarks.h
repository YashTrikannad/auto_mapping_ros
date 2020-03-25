#ifndef AUTO_MAPPING_LANDMARKS_H
#define AUTO_MAPPING_LANDMARKS_H

namespace amr
{

/// Finds frontiers around the point
/// @param point - (x, y) around where you want to find frontiers
/// @param map - CV image of the map
/// @return vector of the centers of frontiers
std::vector<std::array<double, 2>> find_frontiers(const std::array<int, 2>& point,
                                                  const cv::Mat& map,
                                                  int radius_of_interest);

/// Finds the number of frontiers around the point
/// @param point - (x, y) around where you want to find frontiers
/// @param map - CV image of the map
/// @return number of frontiers
int find_n_frontiers(const std::array<int, 2>& point,
                     const cv::Mat& map,
                     int radius_of_interest);

}

#include "impl/landmarks_impl.h"

#endif //AUTO_MAPPING_LANDMARKS_H
