#ifndef AUTO_MAPPING_LANDMARKS_IMPL_H
#define AUTO_MAPPING_LANDMARKS_IMPL_H

#include <boost/math/constants/constants.hpp>
#include <opencv2/core/mat.hpp>
#include "auto_mapping_ros/landmarks.h"

static constexpr auto PI = boost::math::constants::pi<double>();

namespace amr
{

/// Finds frontiers around the point
/// @param point - (x, y) around where you want to find frontiers
/// @param map - CV image of the map
/// @param config
/// @return vector of the centers of frontiers
std::vector<std::array<double, 2>> FrontierFinder::find_frontiers(const cv::Mat& map)
{
    return {};
}

/// Finds the number of frontiers around the point
/// @param point - (x, y) around where you want to find frontiers
/// @param map - CV image of the map
/// @param config
/// @return number of frontiers
int FrontierFinder::find_n_frontiers(const cv::Mat& map)
{
    return find_frontiers(map).size();
}

/// Construct a 2D map around the point
/// @param point - the current position around which you want to do raycasting
/// @param map - Entire Map as an opencv Mat
/// @param config - RayCasting Config to be used for doing ray casting
/// @return
cv::Mat FrontierFinder::ray_cast_to_2d_map(const std::array<int, 2>& point,
                                           const cv::Mat& map)
{
    cv::Mat ray_casted_map = cv::Mat(cv::Size(map.rows, map.cols), CV_32F, cv::Scalar(frontier_config.unknown_value));
    const auto angle_increment = ray_casting_config.fov/ray_casting_config.n_rays;
    auto current_angle = -(ray_casting_config.fov/2);
    for(int i=0; i<ray_casting_config.n_rays; i++)
    {
        std::cout << "current_angle: " << current_angle << std::endl;
        single_ray_cast(point, current_angle, map, &ray_casted_map);
        current_angle = current_angle + angle_increment;
    }
    return ray_casted_map;
}

/// Casts a single ray (point, current_angle) and updates the ray_cast_map based on the hits in map
/// @param point
/// @param current_angle
/// @param map
/// @param ray_cast_map
void FrontierFinder::single_ray_cast(const std::array<int, 2>& point,
        const double current_angle,
        const cv::Mat& map,
        cv::Mat* ray_cast_map)
{
    double current_x_m = point[0] * ray_casting_config.grid_size + ray_casting_config.grid_size / 2;
    double current_y_m = point[1] * ray_casting_config.grid_size + ray_casting_config.grid_size / 2;
    const auto [dir_x, dir_y] = get_xy_direction_components(current_angle);
    const auto dx = dir_x*ray_casting_config.step_size;
    const auto dy = dir_y*ray_casting_config.step_size;

    while(current_x_m >= 0 && current_y_m >= 0 &&
          current_x_m < map.rows * ray_casting_config.grid_size &&
          current_y_m < map.cols * ray_casting_config.grid_size)
    {
        const auto[x, y] =  get_tile_coords({current_x_m, current_y_m});
        if(map.at<float>(x, y) <= static_cast<float>(frontier_config.max_occupied_threshold))
        {
            ray_cast_map->at<float>(x, y) = frontier_config.occupied_value;
            return;
        }
        else
        {
            ray_cast_map->at<float>(x, y) = frontier_config.free_value;
        }

        current_x_m = current_x_m + dx;
        current_y_m = current_y_m + dy;
    }
}

/// Get coordinates of the tiles/cells from the point (in meters)
/// @param cell_size
/// @param point
/// @return
std::array<int, 2> FrontierFinder::get_tile_coords(const std::array<double, 2>& point)
{
    return std::array<int, 2>{static_cast<int>(floor(point[0] / ray_casting_config.grid_size)),
                              static_cast<int>(floor(point[1] / ray_casting_config.grid_size))};
}

/// Get the x y components of the ray direction (at current angle)
/// @param current_angle
/// @return
std::array<double, 2> FrontierFinder::get_xy_direction_components(const double current_angle)
{
    if(current_angle < -(PI/2) && current_angle >= -PI)
    {
        std::cout << "Quadrant 1" << std::endl;
        return {sin(current_angle + PI), cos(current_angle + PI)};
    }
    if(current_angle < 0 && current_angle >= -(PI/2))
    {
        std::cout << "Quadrant 2" << std::endl;
        return {sin(current_angle), cos(current_angle)};
    }
    if(current_angle >= 0 && current_angle < (PI/2))
    {
        std::cout << "Quadrant 3" << std::endl;
        return {-sin(current_angle), -cos(current_angle)};
    }
    if(current_angle >= (PI/2) && current_angle <= PI)
    {
        std::cout << "Quadrant 4" << std::endl;
        std::cout << "dx: "<< sin(current_angle-PI) << " dy: " << cos(current_angle-PI) << std::endl;
        return {-sin(current_angle-PI), -cos(current_angle-PI)};
    }
    std::__throw_logic_error("Input angle is not between PI and -PI");
}


}

#endif //FMT_STAR_LANDMARKS_IMPL_H
