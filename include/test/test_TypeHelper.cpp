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

#include "TypeHelper.hpp"

#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

TEST(test_TypeHelper, CvToRos)
{
    cv::Vec3d input {1.0, -2.0, 3.0};

    geometry_msgs::Vector3 expected;
    expected.x = input[0U];
    expected.y = input[1U];
    expected.z = input[2U];

    geometry_msgs::Vector3 output = TypeHelper::CvToRos(input);

    EXPECT_EQ(expected, output);
}

TEST(test_TypeHelper, RosToEig_quaternion)
{
    geometry_msgs::Quaternion input;
    input.w = 0.5;
    input.x = 0.5;
    input.y = -0.5;
    input.z = -0.5;

    Eigen::Quaterniond expected {0.5, 0.5, -0.5, -0.5};

    Eigen::Quaterniond output = TypeHelper::RosToEig(input);

    EXPECT_EQ(expected, output);
}

TEST(test_TypeHelper, RosToEig_vector)
{
    geometry_msgs::Vector3 input;
    input.x = -1.0;
    input.y = 2.0;
    input.z = -3.0;

    Eigen::Vector<double, 3U> expected {input.x, input.y, input.z};

    Eigen::Vector<double, 3U> output = TypeHelper::RosToEig(input);

    EXPECT_EQ(expected, output);
}

TEST(test_TypeHelper, RosToEig_point)
{
    geometry_msgs::Point input;
    input.x = -1.0;
    input.y = 2.0;
    input.z = -3.0;

    Eigen::Vector3d expected {input.x, input.y, input.z};

    Eigen::Vector3d output = TypeHelper::RosToEig(input);

    EXPECT_EQ(expected, output);
}

TEST(test_TypeHelper, StdToEigQuat)
{
    std::vector<double> input {0.5, -0.5, -0.5, 0.5};

    Eigen::Quaterniond expected {input[0U], input[1U], input[2U], input[3U]};

    Eigen::Quaterniond output = TypeHelper::StdToEigQuat(input);

    EXPECT_EQ(expected, output);
}

TEST(test_TypeHelper, StdToEigQuat_bad)
{
    std::vector<double> input {0.5, -0.5, -0.5};

    Eigen::Quaterniond expected {1.0, 0.0, 0.0, 0.0};

    Eigen::Quaterniond output = TypeHelper::StdToEigQuat(input);

    EXPECT_EQ(expected, output);
}

TEST(test_TypeHelper, StdToEigVec)
{
    std::vector<double> input {-3.0, 2.0, -1.0};

    Eigen::Vector3d expected {input[0U], input[1U], input[2U]};

    Eigen::Vector3d output = TypeHelper::StdToEigVec(input);

    EXPECT_EQ(expected, output);
}

TEST(test_TypeHelper, StdToEigVec_bad)
{
    std::vector<double> input {-3.0, 2.0};

    Eigen::Vector3d expected {0.0, 0.0, 0.0};

    Eigen::Vector3d output = TypeHelper::StdToEigVec(input);

    EXPECT_EQ(expected, output);
}

TEST(test_TypeHelper, Tf2ToEig)
{
    tf2::Quaternion input {-0.5, -0.5, 0.5, 0.5};

    Eigen::Quaterniond expected {input.w(), input.x(), input.y(), input.z()};

    Eigen::Quaterniond output = TypeHelper::Tf2ToEig(input);

    EXPECT_EQ(expected, output);
}

TEST(test_TypeHelper, CvToEig)
{
    cv::Vec3d input {4.0, -5.0, 6.0};

    Eigen::Vector3d expected {input[0U], input[1U], input[2U]};

    Eigen::Vector3d output = TypeHelper::CvToEig(input);

    EXPECT_EQ(expected, output);
}

TEST(test_TypeHelper, EigToRos)
{
    Eigen::Vector3d input {-4.0, 5.0, -6.0};

    geometry_msgs::Vector3 expected;
    expected.x = input.x();
    expected.y = input.y();
    expected.z = input.z();

    geometry_msgs::Vector3 output = TypeHelper::EigToRos(input);

    EXPECT_EQ(expected, output);
}
