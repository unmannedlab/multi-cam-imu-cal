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

#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <string>
#include <vector>

TEST(test_VizNode, Basic)
{
    ros::NodeHandle nh;
    std::string pathInit {""};
    std::vector<double> posInit {0.0, 0.0, 0.0};
    std::vector<double> quatInit {1.0, 0.0, 0.0, 0.0};

    nh.setParam("viz_node/camera_stl", pathInit);
    nh.setParam("viz_node/target_stl", pathInit);
    nh.setParam("viz_node/target_pos", posInit);
    nh.setParam("viz_node/target_quat", quatInit);
    nh.setParam("viz_node/lidar_stl", pathInit);
    nh.setParam("viz_node/imu_stl", pathInit);

    viz_node vNode;

    ros::master::V_TopicInfo topics;
    EXPECT_TRUE(ros::master::getTopics(topics));

    EXPECT_EQ(topics[2U].name, "/viz_node/camera_marker");
    EXPECT_EQ(topics[2U].datatype, "visualization_msgs/Marker");

    EXPECT_EQ(topics[3U].name, "/viz_node/target_marker");
    EXPECT_EQ(topics[3U].datatype, "visualization_msgs/Marker");

    EXPECT_EQ(topics[4U].name, "/viz_node/lidar_marker");
    EXPECT_EQ(topics[4U].datatype, "visualization_msgs/Marker");

    EXPECT_EQ(topics[5U].name, "/viz_node/imu_marker");
    EXPECT_EQ(topics[5U].datatype, "visualization_msgs/Marker");

    nh.deleteParam("viz_node");
}
