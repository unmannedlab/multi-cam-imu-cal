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

#include "VizNode.hpp"

#include <ros/ros.h>

void viz_node::run()
{
    // 1 Hz publish rate
    ros::Rate loop_rate(20);

    // While ros core is running, publish marker meshes
    while (ros::ok())
    {
        cameraMarker.header.stamp = ros::Time();
        // lidarMarker.header.stamp  = ros::Time();
        imuMarker.header.stamp    = ros::Time();
        targetMarker.header.stamp = ros::Time();

        camera_pub.publish(cameraMarker);
        // lidar_pub.publish(lidarMarker);
        imu_pub.publish(imuMarker);
        target_pub.publish(targetMarker);

        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "viz_node");

    viz_node viz;
    viz.run();

    return 0;
}
