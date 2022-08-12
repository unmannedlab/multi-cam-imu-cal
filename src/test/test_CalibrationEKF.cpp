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

#include "CalibrationEKF.hpp"

#include <gtest/gtest.h>
#include <string>

TEST(test_CalibrationEKF, Basic)
{
    xcalibrate::CalibrationEKF calEKF {};

    // Eigen::Quaterniond imuQuat;
    // Eigen::Vector3d gyroBias;
    // Eigen::Vector3d imuVel;
    // Eigen::Vector3d imuBias;
    // Eigen::Vector3d imuPos;
    // Eigen::Quaterniond camQuat;
    // Eigen::Vector3d camPos;

    // calEKF.Initialize(imuQuat, gyroBias, imuVel, imuBias, imuPos, camQuat, camPos);

    // std::string imuName {"imu"};
    // Eigen::Vector3d imuPosInit {1.0, -2.0, 3.0};
    // Eigen::Quaterniond imuQuatInit {0.5, -0.5, -0.5, 0.5};

    // calEKF.RegisterImuExtrinsic(imuName, imuPosInit, imuQuatInit);

    // std::string camName {"cam"};
    // Eigen::Vector3d camPosInit {3.0, -2.0, 1.0};
    // Eigen::Quaterniond camQuatInit {0.5, 0.5, -0.5, -0.5};

    // calEKF.RegisterCamExtrinsic(camName, camPosInit, camQuatInit);

    // std::vector<std::tuple<std::string, Eigen::Vector3d, Eigen::Quaterniond>> extrinsics =
    // calEKF.GetSensorExtrinsics();
}
