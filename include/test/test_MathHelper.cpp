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

#include "MathHelper.hpp"

#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <gtest/gtest.h>

TEST(test_MathHelper, CrossMatrix_Basic)
{
    Eigen::Vector3d testVector {1.0, -2.0, 3.0};

    Eigen::Matrix<double, 3U, 3U> expectedMatrix = Eigen::Matrix<double, 3U, 3U>::Zero();

    expectedMatrix(0U, 1U) = -testVector.z();
    expectedMatrix(0U, 2U) = testVector.y();
    expectedMatrix(1U, 2U) = -testVector.x();
    expectedMatrix(1U, 0U) = testVector.z();
    expectedMatrix(2U, 0U) = -testVector.y();
    expectedMatrix(2U, 1U) = testVector.x();

    Eigen::Matrix<double, 3U, 3U> outMatrix = MathHelper::CrossMatrix(testVector);

    EXPECT_EQ(outMatrix, expectedMatrix);
}

TEST(test_MathHelper, CrossMatrix_Multiplication)
{
    Eigen::Vector3d iVector {1.0, 0.0, 0.0};
    Eigen::Vector3d jVector {0.0, 1.0, 0.0};
    Eigen::Vector3d kVector {0.0, 0.0, 1.0};

    // Right handed unit vector cross products
    EXPECT_EQ(MathHelper::CrossMatrix(iVector) * jVector, kVector);
    EXPECT_EQ(MathHelper::CrossMatrix(jVector) * kVector, iVector);
    EXPECT_EQ(MathHelper::CrossMatrix(kVector) * iVector, jVector);

    // Left handed unit vector cross products
    EXPECT_EQ(MathHelper::CrossMatrix(jVector) * iVector, -kVector);
    EXPECT_EQ(MathHelper::CrossMatrix(kVector) * jVector, -iVector);
    EXPECT_EQ(MathHelper::CrossMatrix(iVector) * kVector, -jVector);
}

TEST(test_MathHelper, RotAngleToQuat)
{
    Eigen::Vector3d theta {0.01, 0.02, 0.03};

    double w = std::sqrt(1.0 - theta.x() * theta.x() - theta.y() * theta.y() - theta.z() * theta.z());
    Eigen::Quaterniond quatExpect {w, theta.x(), theta.y(), theta.z()};
    Eigen::Quaterniond quatOut = MathHelper::RotAngleToQuat(theta);

    EXPECT_EQ(quatExpect, quatOut);
}

TEST(test_MathHelper, PredictQuat)
{
    Eigen::Vector3d omega {1.0, 0.0, 0.0};
    double dT {0.1};

    Eigen::Quaterniond quatIn {1.0, 0.0, 0.0, 0.0};
    Eigen::Quaterniond quatOut = MathHelper::PredictQuat(quatIn, omega, dT);

    Eigen::Quaterniond quatExpect {0.99874921777190895, 0.05, 0, 0};

    EXPECT_EQ(quatOut, quatExpect);
}