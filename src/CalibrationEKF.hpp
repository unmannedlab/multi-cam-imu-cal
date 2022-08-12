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

#ifndef CALIBRATION_EKF_HPP
#define CALIBRATION_EKF_HPP

#include "DynamicSensor.hpp"

#include <eigen3/Eigen/Eigen>
#include <memory>
#include <stdint.h>
#include <string>
#include <tuple>
#include <vector>

namespace xcalibrate
{
    using SensorExtrinsics = std::vector<std::tuple<std::string, Eigen::Vector3d, Eigen::Quaterniond>>;

    /// @todo Update the global initialization to track an offset instead of just waiting for the first cam measurement
    class CalibrationEKF
    {
      public:
        ///
        /// @brief
        ///
        CalibrationEKF();

        ///
        /// @brief Initialize the filter with given values
        /// @param imuQuat Initial IMU orientation in the global frame
        /// @param gyroBias Initial gyro bias
        /// @param imuVel Initial IMU velocity in the global frame
        /// @param imuBias Initial IMU bias
        /// @param imuPos Initial IMU position in the global frame
        /// @param camQuat Initial camera orientation in the IMU frame
        /// @param camPos Initial camera position in the IMU frame
        ///
        void Initialize(Eigen::Quaterniond &imuQuat, Eigen::Vector3d &gyroBias, Eigen::Vector3d &imuVel,
                        Eigen::Vector3d &imuBias, Eigen::Vector3d &imuPos, Eigen::Quaterniond &camQuat,
                        Eigen::Vector3d &camPos);

        ///
        /// @brief Register a new camera sensor
        /// @param sensorName Name associated with new sensor
        /// @param posInit Initial position estimate of sensor
        /// @param quatInit Initial orientation estimate of sensor
        ///
        size_t RegisterCamExtrinsic(std::string sensorName, Eigen::Vector3d &posInit, Eigen::Quaterniond &quatInit);

        ///
        /// @brief Register a new IMU sensor
        /// @param sensorName Name associated with new sensor
        /// @param posInit Initial position estimate of sensor
        /// @param quatInit Initial orientation estimate of sensor
        ///
        size_t RegisterImuExtrinsic(std::string sensorName, Eigen::Vector3d &posInit, Eigen::Quaterniond &quatInit);

        ///
        /// @brief Callback function for IMU messages
        /// @param sensorIndex Index of IMU associated with message
        /// @param sensorTime Sensor measurement time
        /// @param computerTime Computer time at receipt of message
        /// @param q_i_G Measured IMU orientation
        /// @param angVel Measured IMU angular velocity
        /// @param imuAcc Measured IMU acceleration
        ///
        void ImuCallback(size_t sensorIndex, double sensorTime, double computerTime, Eigen::Quaterniond &q_i_G,
                         Eigen::Vector3d &angVel, Eigen::Vector3d &imuAcc);

        ///
        /// @brief Callback function for camera messages
        /// @param sensorIndex Index of camera associated with message
        /// @param sensorTime Sensor measurement time
        /// @param computerTime Computer time at receipt of message
        /// @param q_c_b Orientation of the target in the camera frame
        /// @param p_c_b Position of the target in the camera frame
        /// @param q_G_b Orientation of target in global frame
        /// @param p_G_b Position of target in global frame
        ///
        void CamCallback(size_t sensorIndex, double sensorTime, double computerTime, Eigen::Quaterniond const &q_c_b,
                         Eigen::Vector3d const &p_c_b, Eigen::Quaterniond const &q_G_b, Eigen::Vector3d const &p_G_b);

        ///
        /// @brief Getter method for extrinsic sensors
        /// @return Returns a vectorized list of sensors and their extrinsics
        ///
        inline SensorExtrinsics GetSensorExtrinsics();

      private:
        // Vectors of registered sensors
        std::vector<DynamicSensor> m_camExtrinsicSensors;
        std::vector<DynamicSensor> m_imuExtrinsicSensors;

        // EKF Values
        Eigen::Matrix<double, 23U, 1U> m_state;
        Eigen::Matrix<double, 23U, 23U> m_covariance;

        double m_imuTimePrev {0.0};
        bool m_globalInit {false};

    };    // class CalibrationEKF

    SensorExtrinsics CalibrationEKF::GetSensorExtrinsics()
    {
        SensorExtrinsics sensorExtrinsicsOut;

        for (auto &elem : m_imuExtrinsicSensors)
        {
            sensorExtrinsicsOut.push_back(
                std::make_tuple(elem.GetSensorName(), elem.GetSensorPos(), elem.GetSensorQuat()));
        }
        for (auto &elem : m_camExtrinsicSensors)
        {
            sensorExtrinsicsOut.push_back(
                std::make_tuple(elem.GetSensorName(), elem.GetSensorPos(), elem.GetSensorQuat()));
        }
        return sensorExtrinsicsOut;
    }

}    // namespace xcalibrate

#endif