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

#ifndef TYPE_HELPER_HPP
#define TYPE_HELPER_HPP

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

namespace TypeHelper
{
    ///
    /// @brief Converts OpenCV Vector into ROS Geometry Vector3
    /// @param vectorIn Input vector
    /// @return ROS Geometry message vector3
    ///
    static geometry_msgs::Vector3 CvToRos(cv::Vec3d const &vectorIn)
    {
        geometry_msgs::Vector3 output;
        output.x = vectorIn.val[0U];
        output.y = vectorIn.val[1U];
        output.z = vectorIn.val[2U];
        return output;
    }

    ///
    /// @brief Converts OpenCV Vector into ROS Geometry Vector3
    /// @param quatRos
    /// @return
    ///
    static Eigen::Quaterniond RosToEig(geometry_msgs::Quaternion const &quatRos)
    {
        return Eigen::Quaterniond {quatRos.w, quatRos.x, quatRos.y, quatRos.z};
    }

    ///
    /// @brief Converts ROS Geometry Vector3 to Eigen Vector3
    /// @param vecRos Input ROS Geometry Vector3
    /// @return Output
    ///
    static Eigen::Vector<double, 3U> RosToEig(geometry_msgs::Vector3 const &vecRos)
    {
        return Eigen::Vector3d {vecRos.x, vecRos.y, vecRos.z};
    }

    ///
    /// @brief Converts ROS Geometry Point to Eigen Vector3
    /// @param ptRos Input ROS Geometry Point
    /// @return Output Eigen Vector3
    ///
    static Eigen::Vector3d RosToEig(geometry_msgs::Point const &ptRos)
    {
        return Eigen::Vector3d {ptRos.x, ptRos.y, ptRos.z};
    }

    ///
    /// @brief Converts std::vector into Eigen Quaternion
    /// @param in Input std::vector
    /// @return Output Eigen Quaternion
    ///
    static Eigen::Quaterniond StdToEigQuat(std::vector<double> const &in)
    {
        if (in.size() == 4U)
        {
            Eigen::Quaterniond quat {in[0U], in[1U], in[2U], in[3U]};
            quat.normalize();
            return quat;
        }
        else
        {
            ROS_WARN("Vector incorrect size for Eigen conversion");
            return Eigen::Quaterniond {1.0, 0.0, 0.0, 0.0};
        }
    }

    ///
    /// @brief Converts std::vector into Eigen Quaternion
    /// @param in Input std::vector
    /// @return Output Eigen Vector3
    ///
    static Eigen::Vector3d StdToEigVec(std::vector<double> const &in)
    {
        if (in.size() == 3U)
        {
            return Eigen::Vector3d {in[0U], in[1U], in[2U]};
        }
        else
        {
            ROS_WARN("Vector incorrect size for Eigen conversion");
            return Eigen::Vector3d {0.0, 0.0, 0.0};
        }
    }

    ///
    /// @brief Converts TF2 Quaternion into Eigen Quaternion
    /// @param in Input TF2 Quaternion
    /// @return Output Eigen Quaternion
    ///
    static Eigen::Quaterniond Tf2ToEig(tf2::Quaternion const &in)
    {
        return Eigen::Quaterniond {in.w(), in.x(), in.y(), in.z()};
    }

    ///
    /// @brief Converts OpenCV Vector3 into Eigen Vector3
    /// @param in Input OpenCV Vector3
    /// @return Output Eigen Vector3
    ///
    static Eigen::Vector3d CvToEig(cv::Vec3d const &in)
    {
        return Eigen::Vector3d {in.val[0U], in.val[1U], in.val[2U]};
    }

    ///
    /// @brief Converts Eigen Vector3 into ROS Geometry Vector3
    /// @param vecEig Input Eigen Vector3
    /// @return Output ROS Geometry Vector3
    ///
    static geometry_msgs::Vector3 EigToRos(Eigen::Vector3d const &vecEig)
    {
        geometry_msgs::Vector3 vecOut;
        vecOut.x = vecEig.x();
        vecOut.y = vecEig.y();
        vecOut.z = vecEig.z();
        return vecOut;
    }

}    // namespace TypeHelper

#endif