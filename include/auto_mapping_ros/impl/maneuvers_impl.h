#ifndef AUTO_MAPPING_ROS_MANEUVERS_IMPL_H
#define AUTO_MAPPING_ROS_MANEUVERS_IMPL_H

#include "auto_mapping_ros/maneuvers.h"

/// Stops the Vehicle by sending stop signal to the vehicle
/// \param brake_pub
void amr::stop_vehicle(ros::Publisher* brake_pub)
{
    std_msgs::Bool brake_bool;
    brake_bool.data = true;
    brake_pub->publish(brake_bool);
    ROS_INFO("Published Brake");
}

/// Starts the Vehicle by sending start signal to the vehicle
/// \param vehicle_id
/// \param velocity
/// \param steering_angle
/// \param drive_pub
/// \param base_frame
void amr::start_vehicle(int vehicle_id,
                      double velocity,
                      double steering_angle,
                      ros::Publisher* drive_pub,
                      const std::string& base_frame)
{
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = ros::Time::now();
    drive_msg.header.frame_id = base_frame;
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = velocity;
    drive_pub->publish(drive_msg);
}

#endif //AUTO_MAPPING_ROS_MANEUVERS_IMPL_H
