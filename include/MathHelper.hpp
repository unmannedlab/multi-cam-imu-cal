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

#ifndef MATH_HELPER_HPP
#define MATH_HELPER_HPP

#include <eigen3/Eigen/Eigen>

namespace MathHelper
{
    ///
    /// @brief Produces a cross product matrix
    /// @param inputMatrix Input vector with which to find the left hand size cross product matrix
    /// @return Cross product matrix
    ///
    static Eigen::Matrix<double, 3U, 3U> CrossMatrix(Eigen::Vector3d const &inputMatrix)
    {
        Eigen::Matrix<double, 3U, 3U> outputMatrix = Eigen::Matrix<double, 3U, 3U>::Zero();

        outputMatrix(0U, 1U) = -inputMatrix[2U];
        outputMatrix(0U, 2U) = inputMatrix[1U];
        outputMatrix(1U, 2U) = -inputMatrix[0U];
        outputMatrix(1U, 0U) = inputMatrix[2U];
        outputMatrix(2U, 0U) = -inputMatrix[1U];
        outputMatrix(2U, 1U) = inputMatrix[0U];

        return outputMatrix;
    }

    ///
    /// @brief Transforms rotation angle into quaternion rotation
    /// @param theta Angle vector to convert to quaternion
    /// @return Output quaternion
    ///
    static Eigen::Quaterniond RotAngleToQuat(Eigen::Vector3d const &theta)
    {
        double w = std::sqrt(1.0 - theta[0U] * theta[0U] - theta[1U] * theta[1U] - theta[2U] * theta[2U]);
        return Eigen::Quaterniond(w, theta[0U], theta[1U], theta[2U]);
    }

    ///
    /// @brief Predict new quaternion using current quaternion and a rotation rate
    /// @param quatIn Current quaternion
    /// @param omegaIn Angular rate in radians per second
    /// @param deltaTime change in time in seconds
    /// @return Output quaternion
    ///
    static Eigen::Quaterniond PredictQuat(Eigen::Quaterniond const &quatIn, Eigen::Vector3d const &omegaIn,
                                          double deltaTime)
    {
        return quatIn * RotAngleToQuat(omegaIn * deltaTime * 0.5);
    }

    ///
    /// @brief Calculate the jacobian of a quaternion product with respect to the quaternion
    /// @param quatIn Input quaternion
    /// @param vectorIn Input vector
    /// @return 4x3 Jacobian matrix
    ///
    static Eigen::Matrix<double, 3U, 4U> QuaternionProductJacobian(Eigen::Quaterniond const &quatIn,
                                                                   Eigen::Vector3d const &vectorIn)
    {
        Eigen::Matrix<double, 3U, 4U> outMat;
        double quatW = quatIn.w();
        Eigen::Vector3d quatVec {quatIn.x(), quatIn.y(), quatIn.z()};

        outMat.block<3U, 1U>(0U, 0U) = quatW * vectorIn + quatVec.cross(vectorIn);

        outMat.block<3U, 3U>(0U, 1U) = quatVec.transpose() * vectorIn * Eigen::Matrix3d::Identity()
                                       + quatVec * vectorIn.transpose() - vectorIn * quatVec.transpose()
                                       - quatW * CrossMatrix(vectorIn);

        return 2.0 * outMat;
    }

    ///
    /// @brief Calculate the jacobian of an inverse quaternion product with respect to the quaternion
    /// @param quatIn Input quaternion
    /// @param vectorIn Input vector
    /// @return 4x3 Jacobian matrix
    ///
    static Eigen::Matrix<double, 3U, 4U> InverseQuaternionProductJacobian(Eigen::Quaterniond const &quatIn,
                                                                          Eigen::Vector3d const &vectorIn)
    {
        Eigen::Matrix<double, 3U, 4U> outMat;
        double quatW = quatIn.w();
        Eigen::Vector3d quatVec {quatIn.x(), quatIn.y(), quatIn.z()};

        outMat.block<3U, 1U>(0U, 0U) = quatW * vectorIn + vectorIn.cross(quatVec);

        outMat.block<3U, 3U>(0U, 1U) = quatVec.transpose() * vectorIn * Eigen::Matrix3d::Identity()
                                       + quatVec * vectorIn.transpose() - vectorIn * quatVec.transpose()
                                       + quatW * CrossMatrix(vectorIn);

        return 2.0 * outMat;
    }

}    // namespace MathHelper

#endif