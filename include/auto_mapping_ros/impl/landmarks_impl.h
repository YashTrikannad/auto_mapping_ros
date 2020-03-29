#ifndef AUTO_MAPPING_LANDMARKS_IMPL_H
#define AUTO_MAPPING_LANDMARKS_IMPL_H

#include <boost/math/constants/constants.hpp>
#include <opencv2/core/mat.hpp>

#include "auto_mapping_ros/landmarks.h"
#include "auto_mapping_ros/utils.h"

static constexpr auto PI = boost::math::constants::pi<double>();
static constexpr std::array<std::array<int, 2>, 8> neighbor_indices = {
        std::array<int, 2>{-1, -1},
        std::array<int, 2>{-1, 0},
        std::array<int, 2>{-1, 1},
        std::array<int, 2>{0, -1},
        std::array<int, 2>{0, 1},
        std::array<int, 2>{1, -1},
        std::array<int, 2>{1, 0},
        std::array<int, 2>{1, 1}
};

namespace amr
{

/// Update the mean x,y of the frontier
void Frontier::update_frontier_mean()
{
    frontier_mean[0] = 0;
    frontier_mean[1] = 0;
    for(const auto& frontier_cell: frontier)
    {
        frontier_mean[0] += frontier_cell[0];
        frontier_mean[1] += frontier_cell[1];
    }
    frontier_mean[0] = frontier_mean[0]/frontier.size();
    frontier_mean[1] = frontier_mean[1]/frontier.size();
}

/// Finds frontiers around the point
/// @param point - (x, y) around where you want to find frontiers
/// @param map - CV image of the map
/// @param config
/// @return vector of the centers of frontiers
std::vector<Frontier> FrontierFinder::find_frontiers(const std::array<int, 2>& point,
        const cv::Mat& map) const
{
    const auto ray_casted_map = ray_cast_to_2d_map(point, map);
    const auto frontier_map = get_frontier_cell_mat(ray_casted_map);

    std::vector<std::array<double, 2>> frontier_means{};
    cv::Mat visited = cv::Mat(map.rows, map.cols, cv::DataType<bool>::type, cv::Scalar(false));

    std::vector<Frontier> frontiers;

    for(int i=0; i<frontier_map.rows; i++)
    {
        for(int j=0; j<frontier_map.cols; j++)
        {
            if(frontier_map.at<bool>(i, j) && !static_cast<bool>(visited.at<uchar>(i, j)))
            {
                Frontier frontier{};
                run_dfs_and_update_frontier_groups(i, j, frontier_map, &visited, &frontier.frontier);
                frontier.update_frontier_mean();
                frontiers.emplace_back(frontier);
            }
            else
            {
                visited.at<bool>(i, j) = true;
            }
        }
    }

    return frontiers;
}

/// Finds the number of frontiers around the point
/// @param point - (x, y) around where you want to find frontiers
/// @param map - CV image of the map
/// @param config
/// @return number of frontiers
int FrontierFinder::find_n_frontiers(const std::array<int, 2>& point, const cv::Mat& map) const
{
    return find_frontiers(point, map).size();
}

void FrontierFinder::run_dfs_and_update_frontier_groups(int row_index, int col_index,
        const cv::Mat& frontier_map,
        cv::Mat* visited,
        std::vector<std::array<int, 2>>* frontier_group) const
{
    if(static_cast<bool>(visited->at<uchar>(row_index, col_index))) return;
    frontier_group->push_back({row_index, col_index});
    visited->at<uchar>(row_index, col_index) = true;

    for(int i=0; i<neighbor_indices.size(); i++)
    {
        const auto neighbor_point_x = row_index + neighbor_indices[i][0];
        const auto neighbor_point_y = col_index + neighbor_indices[i][1];

        if(!is_valid_cell(neighbor_point_x, neighbor_point_y, frontier_map.rows, frontier_map.cols) ||
        !frontier_map.at<bool>(neighbor_point_x, neighbor_point_y)) continue;

        run_dfs_and_update_frontier_groups(neighbor_point_x, neighbor_point_y, frontier_map,
                                           visited, frontier_group);
    }
    return;
}

/// Get a CV Matrix of all cells which are frontiers marked as 1 otherwise 06
/// @param map
/// @return
cv::Mat FrontierFinder::get_frontier_cell_mat(const cv::Mat& map) const
{
    cv::Mat frontier_map = cv::Mat(map.rows, map.cols, cv::DataType<bool>::type, cv::Scalar(false));
    for(int i=0; i<map.rows; i++)
    {
        for(int j=0; j<map.cols; j++)
        {
            if(static_cast<int>(map.at<uchar>(i, j)) == frontier_config.free_value &&
            is_frontier_cell(i, j, map))
            {
                frontier_map.at<bool>(i, j) = true;
            }
        }
    }
    return frontier_map;
}

/// Determines if the current FREE cell at map(i, j) is a frontier cell
/// @param row_index
/// @param col_index
/// @param map
/// @return
bool FrontierFinder::is_frontier_cell(int row_index, int col_index, const cv::Mat& map) const
{
    for(int i=0; i<neighbor_indices.size(); i++)
    {
        int neighbor_row_index = row_index+neighbor_indices[i][0];
        int neighbor_col_index = col_index+neighbor_indices[i][1];
        if(is_valid_cell(row_index+neighbor_indices[i][0], col_index+neighbor_indices[i][1], map.rows, map.cols))
        {
            if(static_cast<int>(map.at<uchar>(neighbor_row_index, neighbor_col_index)) == frontier_config.unknown_value)
            {
                return true;
            }
        }
    }
}

/// Determines whether cell (row_index, col_index) is a valid cell
/// @param row_index - current row index
/// @param col_index - current column index
/// @param map_rows - Rows in Map
/// @param map_cols - Cols in Map
/// @return
bool FrontierFinder::is_valid_cell(int row_index, int col_index, int map_rows, int map_cols) const
{
    if(row_index < 0 || row_index > map_rows-1 || col_index < 0 || col_index > map_cols -1)
    {
        return false;
    }
    return true;
}

std::vector<Frontier> FrontierFinder::remove_minor_frontiers(const std::vector<Frontier>& frontiers) const
{
    std::vector<Frontier> new_frontiers;
    for(const auto& frontier: frontiers)
    {
        if(frontier.frontier.size() >= frontier.min_frontier_cells)
        {
            new_frontiers.emplace_back(frontier);
        }
    }
    return new_frontiers;
}

/// Construct a 2D map around the point
/// @param point - the current position around which you want to do raycasting
/// @param map - Entire Map as an opencv Mat
/// @param config - RayCasting Config to be used for doing ray casting
/// @return
cv::Mat FrontierFinder::ray_cast_to_2d_map(const std::array<int, 2>& point,
                                           const cv::Mat& map) const
{
    cv::Mat ray_casted_map = cv::Mat(map.rows, map.cols, CV_8UC1, cv::Scalar(frontier_config.unknown_value));
    const auto angle_increment = ray_casting_config.fov/ray_casting_config.n_rays;
    auto current_angle = -(ray_casting_config.fov/2);
    for(int i=0; i<ray_casting_config.n_rays; i++)
    {
        single_ray_cast(point, current_angle, map, &ray_casted_map);
        current_angle = current_angle + angle_increment;
    }
    return ray_casted_map;
}

/// Construct a 2D map around the point and returns an image of the size constant value more than maximum
/// range of raycasting
/// @param point
/// @param map
/// @return
cv::Mat FrontierFinder::ray_cast_to_2d_sub_map(const std::array<int, 2>& point, const cv::Mat& map) const
{
    const auto map_size = static_cast<int>(ray_casting_config.max_valid_range/ray_casting_config.grid_size)+2;

    const auto full_size_map = ray_cast_to_2d_map(point, map);

    const auto x_min = std::clamp(point[0]-map_size, 0, map.rows-1);
    const auto x_max = std::clamp(point[0]+map_size, 0, map.rows-1);
    const auto y_min = std::clamp(point[1]-map_size, 0, map.cols-1);
    const auto y_max = std::clamp(point[1]+map_size, 0, map.cols-1);

    return full_size_map(cv::Range(x_min, x_max), cv::Range(y_min, y_max));
}

double distance_temp(const std::array<double, 2>& node1, const std::array<double, 2>& node2)
{
    return sqrt(pow(node1[0]-node2[0], 2) + pow(node1[1]-node2[1], 2));
}

/// Casts a single ray (point, current_angle) and updates the ray_cast_map based on the hits in map
/// @param point
/// @param current_angle
/// @param map
/// @param ray_cast_map
void FrontierFinder::single_ray_cast(const std::array<int, 2>& point,
        const double current_angle,
        const cv::Mat& map,
        cv::Mat* ray_cast_map) const
{
    double start_x_m = point[0] * ray_casting_config.grid_size + ray_casting_config.grid_size / 2;
    double start_y_m = point[1] * ray_casting_config.grid_size + ray_casting_config.grid_size / 2;
    double current_x_m = start_x_m;
    double current_y_m = start_y_m;
    const auto [dir_x, dir_y] = get_xy_direction_components(current_angle);
    const auto dx = dir_x*ray_casting_config.step_size;
    const auto dy = dir_y*ray_casting_config.step_size;

    while(current_x_m >= 0 && current_y_m >= 0 &&
          current_x_m < map.rows * ray_casting_config.grid_size &&
          current_y_m < map.cols * ray_casting_config.grid_size &&
          distance_temp(std::array<double, 2>{start_x_m, start_y_m}, std::array<double, 2>{current_x_m, current_y_m})
          < ray_casting_config.max_valid_range)
    {
        const auto[x, y] =  get_tile_coords({current_x_m, current_y_m});
        if(static_cast<int>(map.at<uchar>(x, y)) <= static_cast<int>(frontier_config.max_occupied_threshold))
        {
            ray_cast_map->at<uchar>(x, y) = frontier_config.occupied_value;
            return;
        }
        else
        {
            ray_cast_map->at<uchar>(x, y) = frontier_config.free_value;
        }

        current_x_m = current_x_m + dx;
        current_y_m = current_y_m + dy;
    }
}

/// Get coordinates of the tiles/cells from the point (in meters)
/// @param cell_size
/// @param point
/// @return
std::array<int, 2> FrontierFinder::get_tile_coords(const std::array<double, 2>& point) const
{
    return std::array<int, 2>{static_cast<int>(floor(point[0] / ray_casting_config.grid_size)),
                              static_cast<int>(floor(point[1] / ray_casting_config.grid_size))};
}

/// Get the x y components of the ray direction (at current angle)
/// @param current_angle
/// @return
std::array<double, 2> FrontierFinder::get_xy_direction_components(const double current_angle) const
{
    if(current_angle < -(PI/2) && current_angle >= -PI)
    {
        return {sin(current_angle + PI), cos(current_angle + PI)};
    }
    if(current_angle < 0 && current_angle >= -(PI/2))
    {
        return {sin(current_angle), cos(current_angle)};
    }
    if(current_angle >= 0 && current_angle < (PI/2))
    {
        return {-sin(current_angle), -cos(current_angle)};
    }
    if(current_angle >= (PI/2) && current_angle <= PI)
    {
        return {-sin(current_angle-PI), -cos(current_angle-PI)};
    }
    std::__throw_logic_error("Input angle is not between PI and -PI");
}


}

#endif //FMT_STAR_LANDMARKS_IMPL_H
