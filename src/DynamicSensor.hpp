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

#ifndef DYNAMIC_SENSOR_HPP
#define DYNAMIC_SENSOR_HPP

#include <eigen3/Eigen/Eigen>
#include <string>

namespace xcalibrate
{
    ///
    /// @brief Dynamic sensor parent class
    ///
    class DynamicSensor
    {
      public:
        ///
        /// @brief DynamicSensor constructor
        /// @param sensorName Name of the sensor
        /// @param posInit Initial position estimate of the sensor
        /// @param quatInit Initial orientation estimate of the sensor
        ///
        DynamicSensor(std::string sensorName, Eigen::Vector3d posInit, Eigen::Quaterniond quatInit) :
            m_sensorName(sensorName), m_sensorPosBF(posInit), m_sensorQuatBF(quatInit) {};

        ///
        /// @brief Function to retreive sensor name
        /// @return Sensor name
        ///
        std::string GetSensorName();

        ///
        /// @brief Function to retreive sensor position
        /// @return Sensor position
        ///
        Eigen::Vector3d GetSensorPos();

        ///
        /// @brief Function to retreive sensor orientation
        /// @return Sensor orientation
        ///
        Eigen::Quaterniond GetSensorQuat();

        ///
        /// @brief Function to check if time is initialized
        /// @return Boolean flag indicating time initialization
        ///
        // static bool IsTimeInitialized();

        ///
        /// @brief Function to retrieve time
        /// @return Rigid body time as a double
        ///
        static double GetRigidBodyTime();

        ///
        /// @brief Function to set sensor position
        /// @param in Input sensor position
        ///
        void SetSensorPos(Eigen::Vector3d const &in);

        ///
        /// @brief Function to set sensor orientation
        /// @param in Input sensor orientation
        ///
        void SetSensorQuat(Eigen::Quaterniond const &in);

        ///
        /// @brief Initialize the rigid body
        /// @param bodyTimeInit Initial time used for propagation
        ///
        static void Initialize(double bodyTimeInit);

        ///
        /// @brief Propagate the rigid body position
        /// @param sensorTime Requested time for rigid body propagation
        ///
        static void PropagateTime(double sensorTime);

        ///
        /// @brief Using One-way time translation to estimate sensor time in computer reference
        /// @param sensorTime Sensor time at measurement
        /// @param computerTime Computer time at receipt of measurement
        ///
        double EstimateTime(double sensorTime, double computerTime);

      protected:
        // Static Variables - Rigid Body
        static Eigen::Vector3d m_rigidBodyPos;        ///< @brief Rigid body position
        static Eigen::Vector3d m_rigidBodyVel;        ///< @brief Rigid body velocity
        static Eigen::Vector3d m_rigidBodyAcc;        ///< @brief Rigid body acceleration
        static Eigen::Vector3d m_rigidBodyAngVel;     ///< @brief Rigid body angular velocity
        static Eigen::Quaterniond m_rigidBodyQuat;    ///< @brief Rigid body orientation

        // Extrinsic variables in body frame
        Eigen::Vector3d m_sensorPosBF;        ///< @brief Sensor Position in the IMU frame
        Eigen::Quaterniond m_sensorQuatBF;    ///< @brief Sensor Orientation in the IMU frame

        bool isInitialized {false};    ///< @brief Is sensor time KF initialized

      private:
        std::string m_sensorName;

    };    // class DynamicSensor

    inline std::string DynamicSensor::GetSensorName()
    {
        return m_sensorName;
    }

    inline Eigen::Vector3d DynamicSensor::GetSensorPos()
    {
        return m_sensorPosBF;
    }

    inline void DynamicSensor::SetSensorPos(Eigen::Vector3d const &in)
    {
        m_sensorPosBF = in;
    }

    inline Eigen::Quaterniond DynamicSensor::GetSensorQuat()
    {
        return m_sensorQuatBF;
    }

    inline void DynamicSensor::SetSensorQuat(Eigen::Quaterniond const &in)
    {
        m_sensorQuatBF = in;
    }

}    // namespace xcalibrate

#endif
