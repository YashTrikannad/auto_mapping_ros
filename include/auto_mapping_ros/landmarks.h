#ifndef AUTO_MAPPING_LANDMARKS_H
#define AUTO_MAPPING_LANDMARKS_H

#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>

namespace amr
{

struct RayCastingConfig
{
    double fov = 6.28;
    int n_rays = 1080;
    double max_valid_range = 5;
    double grid_size = 0.05;
    double step_size = grid_size*0.9;
};

struct FrontierConfig
{
    int max_occupied_threshold = 120;
    int min_free_threshold = 200;
    int occupied_value = 0;
    int free_value = 255;
    int unknown_value = 100;
};

class FrontierFinder
{
public:
    FrontierFinder(){};

    /// Finds frontiers around the point
    /// @param point - (x, y) around where you want to find frontiers
    /// @param map - CV image of the map
    /// @param config
    /// @return vector of the centers of frontiers
    std::vector<std::array<double, 2>> find_frontiers(const std::array<int, 2>& point, const cv::Mat& map) const;

    /// Finds the number of frontiers around the point
    /// @param point - (x, y) around where you want to find frontiers
    /// @param map - CV image of the map
    /// @param config
    /// @return number of frontiers
    int find_n_frontiers(const std::array<int, 2>& point, const cv::Mat& map) const;

    /// Construct a 2D map around the point
    /// @param point
    /// @param map
    /// @param config
    /// @return
    cv::Mat ray_cast_to_2d_map(const std::array<int, 2>& point,
                               const cv::Mat& map) const;

private:
    RayCastingConfig ray_casting_config;
    FrontierConfig frontier_config;

    /// Get a CV Matrix of all cells which are frontiers marked as 1 otherwise 06
    /// @param map
    /// @return
    cv::Mat get_frontier_cell_mat(const cv::Mat& map) const;

    /// Determines if the current FREE cell at map(i, j) is a frontier cell
    /// @param row_index
    /// @param col_index
    /// @param map
    /// @return
    bool is_frontier_cell(int row_index, int col_index, const cv::Mat& map) const;

    /// Determines whether cell (row_index, col_index) is a valid cell
    /// @param row_index - current row index
    /// @param col_index - current column index
    /// @param map_rows - Rows in Map
    /// @param map_cols - Cols in Map
    /// @return
    bool is_valid_cell(int row_index, int col_index, int map_rows, int map_cols) const;

    /// Get the x y components of the ray direction (at current angle)
    /// @param current_angle
    /// @return
    std::array<double, 2> get_xy_direction_components(const double current_angle) const;

    /// Get coordinates of the tiles/cells from the point (in meters)
    /// @param cell_size
    /// @param point
    /// @return
    std::array<int, 2> get_tile_coords(const std::array<double, 2>& point) const;

    /// Casts a single ray (point, current_angle) and updates the ray_cast_map based on the hits in map
    /// @param point
    /// @param current_angle
    /// @param map
    /// @param ray_cast_map
    void single_ray_cast(const std::array<int, 2>& point,
                                         const double current_angle,
                                         const cv::Mat& map,
                                         cv::Mat* ray_cast_map) const;
};


}

#include "impl/landmarks_impl.h"

#endif //AUTO_MAPPING_LANDMARKS_H
