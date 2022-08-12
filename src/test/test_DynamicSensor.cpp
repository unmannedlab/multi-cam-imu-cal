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

#include "DynamicSensor.hpp"

#include <gtest/gtest.h>
#include <string>

TEST(test_DynamicSensor, Basic)
{
    std::string sensorName {"Test"};
    Eigen::Vector3d posInit {0.0F, 0.0F, 0.0F};
    Eigen::Quaterniond quatInit {1.0F, 0.0F, 0.0F, 0.0F};

    xcalibrate::DynamicSensor dSensor(sensorName, posInit, quatInit);

    EXPECT_EQ(sensorName, dSensor.GetSensorName());
    EXPECT_EQ(posInit, dSensor.GetSensorPos());
    EXPECT_EQ(quatInit, dSensor.GetSensorQuat());

    Eigen::Vector3d posNew {1.0F, -2.0F, 3.0F};
    Eigen::Quaterniond quatNew {0.5F, 0.5F, 0.5F, 0.5F};

    dSensor.SetSensorPos(posNew);
    dSensor.SetSensorQuat(quatNew);

    EXPECT_EQ(posNew, dSensor.GetSensorPos());
    EXPECT_EQ(quatNew, dSensor.GetSensorQuat());
}

TEST(test_DynamicSensor, EstimateTime)
{
    std::string sensorName {"Test"};
    Eigen::Vector3d posInit {0.0F, 0.0F, 0.0F};
    Eigen::Quaterniond quatInit {1.0F, 0.0F, 0.0F, 0.0F};

    xcalibrate::DynamicSensor dSensor(sensorName, posInit, quatInit);
    double estimatedTime;
    estimatedTime = dSensor.EstimateTime(0.00, 0.00);
    EXPECT_NEAR(estimatedTime, 0.00, 0.01);

    estimatedTime = dSensor.EstimateTime(0.24, 0.25);
    EXPECT_NEAR(estimatedTime, 0.25, 0.01);

    estimatedTime = dSensor.EstimateTime(0.51, 0.50);
    EXPECT_NEAR(estimatedTime, 0.50, 0.01);

    estimatedTime = dSensor.EstimateTime(0.74, 0.75);
    EXPECT_NEAR(estimatedTime, 0.75, 0.01);

    estimatedTime = dSensor.EstimateTime(1.01, 1.00);
    EXPECT_NEAR(estimatedTime, 1.00, 0.01);
}