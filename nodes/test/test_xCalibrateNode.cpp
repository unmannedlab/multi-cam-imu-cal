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

#include "xCalibrateNode.hpp"

#include <gtest/gtest.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <string>

TEST(test_xCalibrateNode, Basic)
{
    ros::NodeHandle nh;
    std::vector<double> vec3Init {0.0, 0.0, 0.0};
    std::vector<double> quatInit {1.0, 0.0, 0.0, 0.0};
    std::vector<std::string> imuSensorVec {"imu_link"};
    std::vector<std::string> camSensorVec {"blackfly"};

    nh.setParam("/xcalibrate_node/blackfly/calFile", "blackfly2.yaml");
    nh.setParam("/xcalibrate_node/blackfly/posInit", vec3Init);
    nh.setParam("/xcalibrate_node/blackfly/quatInit", quatInit);
    nh.setParam("/xcalibrate_node/blackfly/topic", "/camera_array/BFY_camera/image_raw");
    nh.setParam("/xcalibrate_node/camExtrinsicList", camSensorVec);
    nh.setParam("/xcalibrate_node/imu_link/biasInit", vec3Init);
    nh.setParam("/xcalibrate_node/imu_link/posInit", vec3Init);
    nh.setParam("/xcalibrate_node/imu_link/quatInit", quatInit);
    nh.setParam("/xcalibrate_node/imu_link/topic", "/vectornav/IMU");
    nh.setParam("/xcalibrate_node/imuExtrinsicList", imuSensorVec);
    nh.setParam("/xcalibrate_node/target_pos", vec3Init);
    nh.setParam("/xcalibrate_node/target_quat", quatInit);

    xCalibrateNode xNode;

    ros::master::V_TopicInfo topics;
    EXPECT_TRUE(ros::master::getTopics(topics));

    EXPECT_EQ(topics[2U].name, "/tf");
    EXPECT_EQ(topics[2U].datatype, "tf2_msgs/TFMessage");

    EXPECT_EQ(topics[3U].name, "/tf_static");
    EXPECT_EQ(topics[3U].datatype, "tf2_msgs/TFMessage");

    EXPECT_EQ(topics[4U].name, "/xcalibrate_node/detections");
    EXPECT_EQ(topics[4U].datatype, "sensor_msgs/Image");

    EXPECT_EQ(topics[5U].name, "/xcalibrate_node/bias");
    EXPECT_EQ(topics[5U].datatype, "geometry_msgs/Vector3");

    ros::Publisher imuPub = nh.advertise<sensor_msgs::Imu>("/vectornav/IMU", 0);
    ros::Publisher camPub = nh.advertise<sensor_msgs::Image>("/camera_array/BFY_camera/image_raw", 0);

    sensor_msgs::Imu imuMsg;
    imuMsg.linear_acceleration.x = 1.0;
    imuMsg.linear_acceleration.y = 2.0;
    imuMsg.linear_acceleration.z = 3.0;
    imuMsg.angular_velocity.x    = 1.0;
    imuMsg.angular_velocity.y    = 2.0;
    imuMsg.angular_velocity.z    = 3.0;

    // Bad IMU Name
    imuPub.publish(imuMsg);
    ros::Duration(0.5).sleep();
    ros::spinOnce();

    // Fix IMU Name
    imuMsg.header.frame_id = imuSensorVec[0U];
    imuPub.publish(imuMsg);
    ros::Duration(0.5).sleep();
    ros::spinOnce();

    nh.deleteParam("/xcalibrate_node");
}
