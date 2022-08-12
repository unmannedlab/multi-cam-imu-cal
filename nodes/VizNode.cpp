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

#include "VizNode.hpp"

#include "RosHelper.hpp"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

viz_node::viz_node()
{
    // Publishers
    camera_pub = node_handle.advertise<visualization_msgs::Marker>("viz_node/camera_marker", 0);
    target_pub = node_handle.advertise<visualization_msgs::Marker>("viz_node/target_marker", 0);
    lidar_pub  = node_handle.advertise<visualization_msgs::Marker>("viz_node/lidar_marker", 0);
    imu_pub    = node_handle.advertise<visualization_msgs::Marker>("viz_node/imu_marker", 0);

    // Camera Parameters
    std::string camFile = RosHelper::ReadParam<std::string>(node_handle, "/viz_node/camera_stl");

    // Target Parameters
    std::string tgtFile         = RosHelper::ReadParam<std::string>(node_handle, "viz_node/target_stl");
    std::vector<double> tgtPos  = RosHelper::ReadParam<std::vector<double>>(node_handle, "viz_node/target_pos");
    std::vector<double> tgtQuat = RosHelper::ReadParam<std::vector<double>>(node_handle, "viz_node/target_quat");

    // Lidar Parameters
    std::string lidarFile = RosHelper::ReadParam<std::string>(node_handle, "viz_node/lidar_stl");

    // IMU Parameters
    std::string imuFile = RosHelper::ReadParam<std::string>(node_handle, "viz_node/imu_stl");

    // Camera marker properties
    cameraMarker.header.frame_id    = "blackfly";
    cameraMarker.header.stamp       = ros::Time();
    cameraMarker.ns                 = "viz_node";
    cameraMarker.id                 = 0U;
    cameraMarker.type               = visualization_msgs::Marker::MESH_RESOURCE;
    cameraMarker.action             = visualization_msgs::Marker::ADD;
    cameraMarker.pose.position.x    = -0.0625;
    cameraMarker.pose.position.y    = -0.0625;
    cameraMarker.pose.position.z    = -0.0625;
    cameraMarker.pose.orientation.w = 1.0;
    cameraMarker.pose.orientation.x = 0.0;
    cameraMarker.pose.orientation.y = 0.0;
    cameraMarker.pose.orientation.z = 0.0;
    cameraMarker.scale.z            = 0.125;
    cameraMarker.scale.x            = 0.125;
    cameraMarker.scale.y            = 0.125;
    cameraMarker.color.a            = 1.0;
    cameraMarker.color.r            = 1.0;
    cameraMarker.color.g            = 1.0;
    cameraMarker.color.b            = 1.0;
    cameraMarker.mesh_resource      = camFile;

    // Target marker properties
    targetMarker.header.frame_id    = "target";
    targetMarker.header.stamp       = ros::Time();
    targetMarker.ns                 = "viz_node";
    targetMarker.id                 = 1U;
    targetMarker.type               = visualization_msgs::Marker::MESH_RESOURCE;
    targetMarker.action             = visualization_msgs::Marker::ADD;
    targetMarker.pose.position.x    = 0.0;
    targetMarker.pose.position.y    = 0.0;
    targetMarker.pose.position.z    = -0.025;
    targetMarker.pose.orientation.w = 1.0;
    targetMarker.pose.orientation.x = 0.0;
    targetMarker.pose.orientation.y = 0.0;
    targetMarker.pose.orientation.z = 0.0;
    targetMarker.scale.x            = 0.05;
    targetMarker.scale.y            = 0.05;
    targetMarker.scale.z            = 0.05;
    targetMarker.color.a            = 1.0;
    targetMarker.color.r            = 1.0;
    targetMarker.color.g            = 1.0;
    targetMarker.color.b            = 1.0;
    targetMarker.mesh_resource      = tgtFile;

    // Lidar marker properties
    lidarMarker.header.frame_id    = "velodyne";
    lidarMarker.header.stamp       = ros::Time();
    lidarMarker.ns                 = "viz_node";
    lidarMarker.id                 = 2U;
    lidarMarker.type               = visualization_msgs::Marker::MESH_RESOURCE;
    lidarMarker.action             = visualization_msgs::Marker::ADD;
    lidarMarker.pose.position.x    = -0.05;
    lidarMarker.pose.position.y    = 0.05;
    lidarMarker.pose.position.z    = -0.04;
    lidarMarker.pose.orientation.w = 0.7071;
    lidarMarker.pose.orientation.x = 0.7071;
    lidarMarker.pose.orientation.y = 0.0;
    lidarMarker.pose.orientation.z = 0.0;
    lidarMarker.scale.x            = 0.001;
    lidarMarker.scale.y            = 0.001;
    lidarMarker.scale.z            = 0.001;
    lidarMarker.color.a            = 1.0;
    lidarMarker.color.r            = 1.0;
    lidarMarker.color.g            = 1.0;
    lidarMarker.color.b            = 1.0;
    lidarMarker.mesh_resource      = lidarFile;

    // IMU marker properties
    imuMarker.header.frame_id    = "imu_link";
    imuMarker.header.stamp       = ros::Time();
    imuMarker.ns                 = "viz_node";
    imuMarker.id                 = 3U;
    imuMarker.type               = visualization_msgs::Marker::MESH_RESOURCE;
    imuMarker.action             = visualization_msgs::Marker::ADD;
    imuMarker.pose.position.x    = -0.0625;
    imuMarker.pose.position.y    = 0.0625;
    imuMarker.pose.position.z    = -0.015625;
    imuMarker.pose.orientation.w = 0.7071;
    imuMarker.pose.orientation.x = 0.7071;
    imuMarker.pose.orientation.y = 0.0;
    imuMarker.pose.orientation.z = 0.0;
    imuMarker.scale.x            = 0.0125;
    imuMarker.scale.y            = 0.0125;
    imuMarker.scale.z            = 0.0125;
    imuMarker.color.a            = 1.0;
    imuMarker.color.r            = 1.0;
    imuMarker.color.g            = 1.0;
    imuMarker.color.b            = 1.0;
    imuMarker.mesh_resource      = imuFile;
}
