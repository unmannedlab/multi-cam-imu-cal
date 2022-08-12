//--------------------------------------------------------------------------------------------------------------------//
//                                                                                                                    //
//                                                    xCalibrate:                                                     //
//                                                                                                                    //
//                                       Kalman Filter-Based Sensor Calibration                                       //
//                                                                                                                    //
//                                          Copyright (C) 2022 Jacob Hartzer                                          //
//                                                                                                                    //
// This program is free software: you can redistribute it and/or modify it under the terms of the                     //
// GNU General Public License as published by the Free Software Foundation, either version 3 of the License,          //
// or (at your option) any later version.                                                                             //
//                                                                                                                    //
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;                          //
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                          //
// See the GNU General Public License for more details.                                                               //
//                                                                                                                    //
// You should have received a copy of the GNU General Public License along with this program.                         //
// If not, see <https://www.gnu.org/licenses/>.                                                                       //
//                                                                                                                    //
//--------------------------------------------------------------------------------------------------------------------//

#ifndef VIZ_NODE_HPP
#define VIZ_NODE_HPP

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

///
/// @brief ROS node for managing the visualization
///
class viz_node
{
  public:
    ///
    /// @brief Constructor for visualization node
    ///
    viz_node();

    ///
    /// @brief Continuous run method for visualization node
    ///
    void run();

  private:
    ros::NodeHandle node_handle;

    ros::Publisher camera_pub;
    ros::Publisher target_pub;
    ros::Publisher lidar_pub;
    ros::Publisher imu_pub;

    visualization_msgs::Marker cameraMarker;
    visualization_msgs::Marker targetMarker;
    visualization_msgs::Marker lidarMarker;
    visualization_msgs::Marker imuMarker;
};

#endif