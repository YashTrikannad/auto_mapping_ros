#ifndef VIZ_H
#define VIZ_H
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

// Helper class for generating pretty visualizations of various functions

class Visualizer
{
public:
    static std::vector<geometry_msgs::Point> GenerateVizPoints(std::vector<std::pair<float,float>> &points);
    static std::vector<std_msgs::ColorRGBA> GenerateVizColors(std::vector<std::pair<float,float>> &points, float r = 1, float g = 1, float b = 1);
    static visualization_msgs::Marker GenerateList(std::vector<geometry_msgs::Point> &marker_points, std::vector<std_msgs::ColorRGBA> &marker_colors, int type = visualization_msgs::Marker::SPHERE_LIST,  double scale_x = 0.1, double scale_y = 0.1, double scale_z = 0.1);
    static visualization_msgs::Marker GenerateList(std::vector<std::pair<float,float>> &points, std::vector<std_msgs::ColorRGBA> &marker_colors, int type = visualization_msgs::Marker::SPHERE_LIST,  double scale_x = 0.1, double scale_y = 0.1, double scale_z = 0.1);
    static visualization_msgs::Marker GenerateSphereList(std::vector<std::pair<float,float>> &points, float r = 1, float g = 1, float b = 1);
    static visualization_msgs::Marker GenerateSphereList(std::vector<std::pair<float,float>> &points, std::vector<std_msgs::ColorRGBA> &marker_colors);
};

#endif