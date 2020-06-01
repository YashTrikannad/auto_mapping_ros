#include "auto_mapping_ros/transforms.hpp"

std::pair<float, float> Transforms::CarPointToWorldPoint(float x, float y, geometry_msgs::Pose &current_pose)
{
    tf2::Transform car_to_world;
    geometry_msgs::Transform car_to_world_msg;
    geometry_msgs::TransformStamped car_to_world_stamped;
    tf2::fromMsg(current_pose, car_to_world);
    car_to_world_msg = tf2::toMsg(car_to_world);
    car_to_world_stamped.transform = car_to_world_msg;
    geometry_msgs::Vector3 carPoint;
    geometry_msgs::Vector3 worldPoint;
    carPoint.x = x;
    carPoint.y = y;
    carPoint.z = 0;
    tf2::doTransform(carPoint, worldPoint, car_to_world_stamped);
    float carPoseX = current_pose.position.x;
    float carPoseY = current_pose.position.y;
    return std::pair<float,float>(worldPoint.x+carPoseX, worldPoint.y+carPoseY);
}

geometry_msgs::TransformStamped Transforms::WorldToCarTransform(const geometry_msgs::Pose &pose)
{
    tf2::Transform car_to_world;
    tf2::fromMsg(pose, car_to_world);
    tf2::Transform world_to_car = car_to_world.inverse();
    geometry_msgs::Transform world_to_car_msg = tf2::toMsg(world_to_car); //H^c_w
    geometry_msgs::TransformStamped world_to_car_msg_stamped;
    world_to_car_msg_stamped.transform = world_to_car_msg;
    return world_to_car_msg_stamped;
}

std::pair<float, float> Transforms::TransformPoint(std::pair<float, float> point, geometry_msgs::TransformStamped &transform_msg)
{
    geometry_msgs::Vector3 waypoint_in;
    geometry_msgs::Vector3 waypoint_out;
    waypoint_in.x = point.first;
    waypoint_in.y = point.second;
    waypoint_in.z = 0;
    tf2::doTransform(waypoint_in, waypoint_out, transform_msg);
    waypoint_out.x += transform_msg.transform.translation.x;
    waypoint_out.y += transform_msg.transform.translation.y;
    return std::pair<float,float>(waypoint_out.x, waypoint_out.y);
}

float Transforms::GetCarOrientation(geometry_msgs::Pose pose)
{
    return atan2(2 * pose.orientation.w * pose.orientation.z, 1 - 2 * pose.orientation.z * pose.orientation.z);
}

float Transforms::CalcDist(std::pair<float,float> p1, std::pair<float,float> p2)
{
    float dist = sqrt(pow((p1.first - p2.first),2) + pow((p1.second-p2.second),2));
    return dist;
}
