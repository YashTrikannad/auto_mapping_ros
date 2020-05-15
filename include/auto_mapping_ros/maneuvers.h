#ifndef AUTO_MAPPING_ROS_MANEUVERS_H
#define AUTO_MAPPING_ROS_MANEUVERS_H

#include <ackermann_msgs/AckermannDriveStamped.h>

namespace amr
{
    /// Stops the Vehicle by sending stop signal to the vehicle
    /// \param brake_pub
    void stop_vehicle(ros::Publisher* brake_pub);

    /// Starts the Vehicle by sending start signal to the vehicle
    /// \param vehicle_id
    /// \param velocity
    /// \param steering_angle
    /// \param drive_pub
    /// \param base_frame
    void start_vehicle(int vehicle_id,
                      double velocity,
                      double steering_angle,
                      ros::Publisher* drive_pub,
                      const std::string& base_frame);
}

#endif //AUTO_MAPPING_ROS_MANEUVERS_H

#include "impl/maneuvers_impl.h"
