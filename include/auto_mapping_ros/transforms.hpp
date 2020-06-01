#ifndef TRANS_H
#define TRANS_H
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Static helper functions for transforming to and from the car frame

class Transforms
{
public:
    static std::pair<float, float> CarPointToWorldPoint(float x, float y, geometry_msgs::Pose &current_pose);
    static geometry_msgs::TransformStamped WorldToCarTransform(const geometry_msgs::Pose &pose);
    static std::pair<float, float> TransformPoint(std::pair<float, float> point, geometry_msgs::TransformStamped &transform_msg);
    static float GetCarOrientation(geometry_msgs::Pose pose);
    static float CalcDist(std::pair<float,float> p1, std::pair<float,float> p2);
};
#endif