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

#include "DynamicSensor.hpp"
#include "MathHelper.hpp"
#include "TypeHelper.hpp"

#include <iostream>
#include <ostream>
#include <stdint.h>

xcalibrate::CalibrationEKF::CalibrationEKF()
{
    // State Initialization
    m_state = Eigen::Matrix<double, 23U, 1U>::Zero();

    // Initialize quaternions with zero rotation
    m_state(0U)  = 1.0;
    m_state(16U) = 1.0;

    // Covariance initialization
    m_covariance = Eigen::Matrix<double, 23U, 23U>::Identity();

    // List of states:
    // m_state(0U) = q_i_G : Orientation of the Global frame in the IMU frame - w
    // m_state(1U) = q_i_G : Orientation of the Global frame in the IMU frame - x
    // m_state(2U) = q_i_G : Orientation of the Global frame in the IMU frame - y
    // m_state(3U) = q_i_G : Orientation of the Global frame in the IMU frame - z

    // m_state(4U) = b_g : Gyro bias in the IMU frame - x
    // m_state(5U) = b_g : Gyro bias in the IMU frame - y
    // m_state(6U) = b_g : Gyro bias in the IMU frame - z

    // m_state(7U) = v_i : IMU velocity in the Global frame - x
    // m_state(8U) = v_i : IMU velocity in the Global frame - y
    // m_state(9U) = v_i : IMU velocity in the Global frame - z

    // m_state(10U) = b_a : Accelerometer bias in the IMU frame - x
    // m_state(11U) = b_a : Accelerometer bias in the IMU frame - y
    // m_state(12U) = b_a : Accelerometer bias in the IMU frame - z

    // m_state(13U) = p_G_i : IMU position in the Global frame - x
    // m_state(14U) = p_G_i : IMU position in the Global frame - y
    // m_state(15U) = p_G_i : IMU position in the Global frame - z

    // m_state(16U) = q_i_c : Orientation of the camera frame in the IMU frame - w
    // m_state(17U) = q_i_c : Orientation of the camera frame in the IMU frame - x
    // m_state(18U) = q_i_c : Orientation of the camera frame in the IMU frame - y
    // m_state(19U) = q_i_c : Orientation of the camera frame in the IMU frame - z

    // m_state(13U) = p_i_c : IMU position in the Global frame - x
    // m_state(14U) = p_i_c : IMU position in the Global frame - y
    // m_state(15U) = p_i_c : IMU position in the Global frame - z
}

void xcalibrate::CalibrationEKF::Initialize(Eigen::Quaterniond &imuQuat, Eigen::Vector3d &gyroBias,
                                            Eigen::Vector3d &imuVel, Eigen::Vector3d &imuBias, Eigen::Vector3d &imuPos,
                                            Eigen::Quaterniond &camQuat, Eigen::Vector3d &camPos)
{
    m_state(0U) = imuQuat.w();    // IMU to Global Frame rotation w
    m_state(1U) = imuQuat.x();    // IMU to Global Frame rotation x
    m_state(2U) = imuQuat.y();    // IMU to Global Frame rotation y
    m_state(3U) = imuQuat.z();    // IMU to Global Frame rotation z

    m_state(4U) = gyroBias.x();    // gyro bias x
    m_state(5U) = gyroBias.y();    // gyro bias y
    m_state(6U) = gyroBias.z();    // gyro bias z

    m_state(7U) = imuVel.x();    // IMU velocity x
    m_state(8U) = imuVel.y();    // IMU velocity y
    m_state(9U) = imuVel.z();    // IMU velocity z

    m_state(10U) = imuBias.x();    // IMU bias x
    m_state(11U) = imuBias.y();    // IMU bias y
    m_state(12U) = imuBias.z();    // IMU bias z

    m_state(13U) = imuPos.x();    // IMU position x
    m_state(14U) = imuPos.y();    // IMU position y
    m_state(15U) = imuPos.z();    // IMU position z

    m_state(16U) = camQuat.w();    // IMU to Camera Frame rotation w
    m_state(17U) = camQuat.x();    // IMU to Camera Frame rotation x
    m_state(18U) = camQuat.y();    // IMU to Camera Frame rotation y
    m_state(19U) = camQuat.z();    // IMU to Camera Frame rotation z

    m_state(20U) = camPos.x();    // camera position x
    m_state(21U) = camPos.y();    // camera position y
    m_state(22U) = camPos.z();    // camera position z

    m_covariance = Eigen::Matrix<double, 23U, 23U>::Identity();
}

size_t xcalibrate::CalibrationEKF::RegisterImuExtrinsic(std::string sensorName, Eigen::Vector3d &posInit,
                                                        Eigen::Quaterniond &quatInit)
{
    m_imuExtrinsicSensors.push_back(DynamicSensor(sensorName, posInit, quatInit));

    return m_imuExtrinsicSensors.size() - 1U;
}

size_t xcalibrate::CalibrationEKF::RegisterCamExtrinsic(std::string sensorName, Eigen::Vector3d &posInit,
                                                        Eigen::Quaterniond &quatInit)
{
    m_camExtrinsicSensors.push_back(DynamicSensor(sensorName, posInit, quatInit));

    return m_camExtrinsicSensors.size() - 1U;
}

/// @todo Implement actual gyro prediction equations and remove quaternion from IMU assumption
void xcalibrate::CalibrationEKF::ImuCallback(size_t sensorIndex, double sensorTime, double computerTime,
                                             Eigen::Quaterniond &q_i_G, Eigen::Vector3d &imuAngVel,
                                             Eigen::Vector3d &imuAcc)
{
    // IMU Callback function
    double timeEst = m_imuExtrinsicSensors[0U].EstimateTime(sensorTime, computerTime);

    static Eigen::Matrix<double, 23U, 23U> F = Eigen::Matrix<double, 23U, 23U>::Zero();
    static Eigen::Matrix<double, 23U, 23U> Q = Eigen::Matrix<double, 23U, 23U>::Zero();
    static Eigen::Vector3d gravity {0.0, 0.0, -8.2};

    if (!m_globalInit)
    {
        return;
    }

    if (m_imuTimePrev == 0.0)
    {
        // Estimated timestep
        static constexpr double imuDeltaTime {1.0 / 200.0};

        // Assign constant identities
        F(0U, 0U)                = 1.0;
        F.block<3U, 3U>(1U, 4U)  = -Eigen::Matrix3d::Identity();
        F.block<3U, 3U>(13U, 7U) = Eigen::Matrix3d::Identity();

        // Assign covariance constants
        // Global frame orientation
        Q(0U, 0U) = 0.01;
        Q(1U, 1U) = 0.01;
        Q(2U, 2U) = 0.01;
        Q(3U, 3U) = 0.01;

        // Gyro bias
        Q(4U, 4U) = 0.0001;
        Q(5U, 5U) = 0.0001;
        Q(6U, 6U) = 0.0001;

        // IMU Velocity
        Q(7U, 7U) = imuDeltaTime * 0.01;
        Q(8U, 8U) = imuDeltaTime * 0.01;
        Q(9U, 9U) = imuDeltaTime * 0.01;

        // Accelerometer Bias
        Q(10U, 10U) = 0.0001;
        Q(11U, 11U) = 0.0001;
        Q(12U, 12U) = 0.0001;

        // IMU Position
        Q(13U, 13U) = imuDeltaTime * imuDeltaTime * 0.01;
        Q(14U, 14U) = imuDeltaTime * imuDeltaTime * 0.01;
        Q(15U, 15U) = imuDeltaTime * imuDeltaTime * 0.01;

        // Camera frame orientation
        Q(16U, 16U) = 0.0001;
        Q(17U, 17U) = 0.0001;
        Q(18U, 18U) = 0.0001;
        Q(19U, 19U) = 0.0001;

        // Camera position
        Q(20U, 20U) = 0.0001;
        Q(21U, 21U) = 0.0001;
        Q(22U, 22U) = 0.0001;
    }
    else
    {
        // Calculated timestep
        double dT = timeEst - m_imuTimePrev;

        /// @todo implement RK4 integration
        /// @todo include bias in state transition matrix
        Eigen::Vector3d imuBias        = m_state.block<3U, 1U>(10, 0U);
        Eigen::Vector3d imuAccUnbiased = imuAcc - imuBias;
        Eigen::Vector3d imuAccNED      = q_i_G.toRotationMatrix() * imuAccUnbiased - gravity;
        Eigen::Vector3d gyroBias       = m_state.block<3U, 1U>(4U, 0U);
        Eigen::Vector3d angVelUnbiased = imuAngVel - gyroBias;
        Eigen::Quaterniond quatDel {1.0, dT * angVelUnbiased.x(), dT * angVelUnbiased.y(), dT * angVelUnbiased.z()};

        m_state(0U) = quatDel.w();    // NED to IMU Frame rotation w
        m_state(1U) = quatDel.x();    // NED to IMU Frame rotation x
        m_state(2U) = quatDel.y();    // NED to IMU Frame rotation y
        m_state(3U) = quatDel.z();    // NED to IMU Frame rotation z

        m_state.block<3U, 1U>(7U, 0U) += dT * imuAccNED;    // Velocity

        m_state.block<3U, 1U>(13U, 0U) += 1 / 2 * dT * dT * imuAccNED;    // Position

        Eigen::Vector3d rigidBodyPos(m_state(13U), m_state(14U), m_state(15U));
        Eigen::Quaterniond rigidBodyQuat(m_state(0U), m_state(1U), m_state(2U), m_state(3U));

        m_imuExtrinsicSensors[0U].SetSensorPos(rigidBodyPos);
        m_imuExtrinsicSensors[0U].SetSensorQuat(rigidBodyQuat);

        /// @todo add gyro bias
        F.block<3U, 3U>(1U, 1U)  = -MathHelper::CrossMatrix(imuAngVel - gyroBias);
        F.block<3U, 3U>(7U, 1U)  = -q_i_G.toRotationMatrix().transpose() * MathHelper::CrossMatrix(imuAccUnbiased);
        F.block<3U, 3U>(7U, 10U) = -q_i_G.toRotationMatrix().transpose();

        m_covariance = F * m_covariance * F.transpose() + Q;
    }

    m_imuTimePrev = timeEst;
}

/// @todo get R value from yaml
void xcalibrate::CalibrationEKF::CamCallback(size_t sensorIndex, double sensorTime, double computerTime,
                                             Eigen::Quaterniond const &q_c_b, Eigen::Vector3d const &p_c_b,
                                             Eigen::Quaterniond const &q_G_b, Eigen::Vector3d const &p_G_b)
{
    // IMU to global frame from state
    Eigen::Quaterniond q_i_G {m_state(0U), m_state(1U), m_state(2U), m_state(3U)};
    Eigen::Quaterniond q_G_i = q_i_G.inverse();

    // IMU position in global frame
    Eigen::Vector3d p_G_i = m_state.block<3U, 1U>(13U, 0U);

    // IMU to camera frame from state
    Eigen::Quaterniond q_i_c {m_state(16U), m_state(17U), m_state(18U), m_state(19U)};
    Eigen::Quaterniond q_c_i = q_i_c.inverse();

    // Camera position in IMU frame from state
    Eigen::Vector3d p_i_c = m_state.block<3U, 1U>(20U, 0U);

    // Transform measurements to NED
    Eigen::Vector3d p_G_c_hat    = q_G_b.toRotationMatrix() * (-p_c_b);
    Eigen::Quaterniond q_G_i_hat = q_G_b * q_c_b.inverse() * q_c_i;

    // If this is the first measurement, set the global position to the measurement
    if (!m_globalInit)
    {
        // Set initialization flag to true. Proceed to update on subsequent update calls
        m_globalInit = true;

        Eigen::Quaterniond q_i_G_hat = q_G_i_hat.inverse();
        Eigen::Vector3d p_G_i_hat    = p_G_c_hat + p_G_b - q_G_i.toRotationMatrix() * p_i_c;

        m_state(0U) = q_i_G_hat.w();
        m_state(1U) = q_i_G_hat.x();
        m_state(2U) = q_i_G_hat.y();
        m_state(3U) = q_i_G_hat.z();

        m_state(13U) = p_G_i_hat.x();
        m_state(14U) = p_G_i_hat.y();
        m_state(15U) = p_G_i_hat.z();
    }
    else
    {
        Eigen::Matrix<double, 3U, 23U> H       = Eigen::Matrix<double, 3U, 23U>::Zero();
        static Eigen::Matrix<double, 3U, 3U> R = Eigen::Matrix<double, 3U, 3U>::Identity() * 10.0;

        // Rigid body position from state
        Eigen::Vector3d imuPos = m_state.block<3U, 1U>(13U, 0U);

        // Measurement residual
        Eigen::Vector3d resid
            = (-p_c_b) - (q_G_b.toRotationMatrix().transpose() * (p_G_i - p_G_b + q_G_i.toRotationMatrix() * p_i_c));

        // IMU to global frame rotation
        H.block<3U, 4U>(0U, 0U)
            = q_G_b.toRotationMatrix().transpose() * MathHelper::InverseQuaternionProductJacobian(q_i_G, p_i_c);

        // IMU position in global frame
        H.block<3U, 3U>(0U, 13U) = q_G_b.toRotationMatrix().transpose();

        // Camera position in IMU frame
        H.block<3U, 3U>(0U, 20U) = q_G_b.toRotationMatrix().transpose() * q_G_i.toRotationMatrix();

        Eigen::Matrix<double, 3U, 3U> S = H * m_covariance * H.transpose() + R;

        // Outlier rejection
        double chi = resid.transpose() * S.inverse() * resid;
        if (chi < 1.0e9)
        {
            Eigen::Matrix<double, 23U, 3U> K = m_covariance * H.transpose() * S.inverse();

            Eigen::Matrix<double, 23U, 1U> correction = K * resid;

            m_state += correction;
            m_covariance = (Eigen::Matrix<double, 23U, 23U>::Identity() - K * H) * m_covariance;

            Eigen::Quaterniond q_i_G_hat = q_G_i_hat.inverse();

            /// @todo need to not use imuQuaternion from complimentary filter
            // Normalize IMU quaternion
            // Eigen::Quaterniond imuQuat {m_state(0U), m_state(1U), m_state(2U), m_state(3U)};
            // imuQuat.normalize();
            // m_state(0U) = imuQuat.w();
            // m_state(1U) = imuQuat.x();
            // m_state(2U) = imuQuat.y();
            // m_state(3U) = imuQuat.z();

            // Normalize camera quaternion
            Eigen::Quaterniond camQuat {m_state(16U), m_state(17U), m_state(18U), m_state(19U)};
            camQuat.normalize();
            m_state(16U) = camQuat.w();
            m_state(17U) = camQuat.x();
            m_state(18U) = camQuat.y();
            m_state(19U) = camQuat.z();
        }
        else
        {
            // std::cout << "Outlier Rejected: Chi = " << chi << std::endl;
        }
    }

    // Grab newest position and orientation
    Eigen::Vector3d imuPosOut = m_state.block<3U, 1U>(13U, 0U);
    Eigen::Vector3d camPosOut = m_state.block<3U, 1U>(20U, 0U);
    Eigen::Quaterniond imuQuatOut(m_state(0U), m_state(1U), m_state(2U), m_state(3U));
    Eigen::Quaterniond camQuatOut(m_state(16U), m_state(17U), m_state(18U), m_state(19U));

    // Set imu data using state info
    m_imuExtrinsicSensors[0U].SetSensorPos(imuPosOut);
    m_imuExtrinsicSensors[0U].SetSensorQuat(imuQuatOut.inverse());
    m_camExtrinsicSensors[0U].SetSensorPos(camPosOut);
    m_camExtrinsicSensors[0U].SetSensorQuat(camQuatOut);
}
