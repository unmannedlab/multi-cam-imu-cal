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

#include "RosHelper.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(test_RosHelper, ParamTest)
{
    ros::NodeHandle nh;
    std::string paramOut;
    std::string paramIn {"TestString"};
    std::string paramName {"TestParamName"};
    std::string badParamName {"BadTestParamName"};

    nh.setParam(paramName, paramIn);

    paramOut = RosHelper::ReadParam<std::string>(nh, paramName);

    EXPECT_EQ(paramIn, paramOut);
    EXPECT_TRUE(nh.ok());

    std::string paramExpect {""};
    nh.setParam(paramName, paramIn);

    paramOut = RosHelper::ReadParam<std::string>(nh, badParamName);

    EXPECT_EQ(paramOut, paramExpect);
    EXPECT_FALSE(nh.ok());

    nh.deleteParam(paramName);
}