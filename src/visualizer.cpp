#include "auto_mapping_ros/visualizer.hpp"

std::vector<geometry_msgs::Point> Visualizer::GenerateVizPoints(std::vector<std::pair<float,float>> &points)
{
    std::vector<geometry_msgs::Point> vis_points;
    for (unsigned int ii = 0; ii < points.size(); ii += 1)
    {
        geometry_msgs::Point curr;
        curr.x = points[ii].first;
        curr.y = points[ii].second;
        curr.z = 0.1;
        vis_points.push_back(curr);
    }
    return vis_points;
}
std::vector<std_msgs::ColorRGBA> Visualizer::GenerateVizColors(std::vector<std::pair<float,float>> &points, float r, float g, float b)
{
    std::vector<std_msgs::ColorRGBA> vis_colors;
    for (unsigned int ii = 0; ii < points.size(); ii += 1)
    {
        std_msgs::ColorRGBA curr;
        curr.r = r;
        curr.g = g;
        curr.b = b;
        curr.a = 1;
        vis_colors.push_back(curr);
    }
    return vis_colors;
}

visualization_msgs::Marker Visualizer::GenerateList(std::vector<geometry_msgs::Point> &marker_points, std::vector<std_msgs::ColorRGBA> &marker_colors, int type, double scale_x, double scale_y, double scale_z)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "current";
    marker.id = 0;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.points = marker_points;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;
    marker.color.a = 0.6; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.colors = marker_colors;
    return marker;
}

visualization_msgs::Marker Visualizer::GenerateList(std::vector<std::pair<float,float>> &points, std::vector<std_msgs::ColorRGBA> &marker_colors, int type, double scale_x, double scale_y, double scale_z)
{
    std::vector<geometry_msgs::Point> temp = GenerateVizPoints(points);
    return GenerateList(temp, marker_colors, type, scale_x, scale_y, scale_z);
}



visualization_msgs::Marker Visualizer::GenerateSphereList(std::vector<std::pair<float,float>> &points, float r, float g, float b)
{
    std::vector<std_msgs::ColorRGBA> colors = GenerateVizColors(points, r, g, b);
    return GenerateSphereList(points, colors);
}

visualization_msgs::Marker Visualizer::GenerateSphereList(std::vector<std::pair<float,float>> &points, std::vector<std_msgs::ColorRGBA> &marker_colors)
{
    return GenerateList(points, marker_colors, visualization_msgs::Marker::SPHERE_LIST);
}