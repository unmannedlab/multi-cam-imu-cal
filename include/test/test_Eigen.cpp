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

#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <gtest/gtest.h>

// Test ensures the correct version of eigen is pulled that has templating
TEST(test_Eigen, Templates)
{
    Eigen::Vector<double, 3U> vector1 {1.0, 2.0, 3.0};
    Eigen::Vector3d vector2 {1.0, 2.0, 3.0};
    EXPECT_EQ(vector1, vector2);
    EXPECT_TRUE(true);

    Eigen::Matrix<double, 3U, 1U> matrix1 {1.0, 2.0, 3.0};
    EXPECT_EQ(vector1, matrix1);

    Eigen::Quaterniond quaternion1 {0.5, 0.5, 0.5, 0.5};
    Eigen::Quaternion<double> quaternion2 {0.5, 0.5, 0.5, 0.5};
    EXPECT_EQ(quaternion1, quaternion2);
}
