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

#include "MathHelper.hpp"
#include "TypeHelper.hpp"

// Re-Declare Static Variables
Eigen::Vector3d xcalibrate::DynamicSensor::m_rigidBodyPos;
Eigen::Vector3d xcalibrate::DynamicSensor::m_rigidBodyVel;
Eigen::Vector3d xcalibrate::DynamicSensor::m_rigidBodyAcc;
Eigen::Quaterniond xcalibrate::DynamicSensor::m_rigidBodyQuat;
Eigen::Vector3d xcalibrate::DynamicSensor::m_rigidBodyAngVel;

/// @todo Update filter to use Tc = (Ts * (1+ alpha)) + beta
double xcalibrate::DynamicSensor::EstimateTime(double sensorTime, double computerTime)
{
    // EstimateSensorTime
    // 1-D Kalman filter for time synchronization
    // Measurement model: Time_computer = (Time_sensor * alpha) + beta

    // Initial Parameters
    static double alpha {0.0};
    static double beta {0.0};
    static double aCov {1.0E-03};
    static double bCov {1.0E-06};
    static double R {1.0E-06};

    if (!isInitialized)
    {
        beta          = computerTime - sensorTime;
        isInitialized = true;
        return computerTime;
    }
    else
    {
        // KF intermediate values
        const double S {aCov * sensorTime * sensorTime + R + bCov};

        // KF State Update
        alpha = alpha - (sensorTime * aCov * (beta - computerTime + sensorTime * alpha)) / S;
        beta  = beta - (bCov * (beta - computerTime + sensorTime * alpha)) / S;

        // KF Covariance Update
        aCov = -aCov * ((sensorTime * sensorTime * aCov) / S - 1U);
        bCov = -bCov * (bCov / S - 1U);

        return alpha * sensorTime + beta;
    }
}
