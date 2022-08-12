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

#ifndef ROS_HELPER_HPP
#define ROS_HELPER_HPP

#include <ros/ros.h>
#include <string>

namespace RosHelper
{
    ///
    /// @brief Read parameter from ROS node handle namespace
    /// @tparam Type of requested parameter
    /// @param nodeHandle Node Handle used for parameter search
    /// @param name String name of requested parameter
    /// @return Requested parameter
    ///
    template <typename T>
    T ReadParam(ros::NodeHandle &nodeHandle, std::string const &name)
    {
        T parameter;
        std::string fullName = nodeHandle.getNamespace() + "/" + name;
        if (nodeHandle.getParam(fullName, parameter))
        {
            ROS_INFO_STREAM("Loaded " << fullName);
        }
        else
        {
            ROS_ERROR_STREAM("Failed to load " << fullName);
            nodeHandle.shutdown();
        }
        return parameter;
    }

};    // namespace RosHelper

#endif