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

#ifndef XCALIBRATE_NODE_HPP
#define XCALIBRATE_NODE_HPP

#include "CalibrationEKF.hpp"

#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Eigen>
#include <functional>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <stdint.h>
#include <string.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tuple>

class xCalibrateNode
{
  public:
    ///
    /// @class xCalibrateNode
    /// @brief A ROS node for manipulating ros topic data before it is passed to the calibration EKF
    /// @todo  Yaml input for all tuning parameters
    ///        - Accelerometer intrinsics, random walk, scale factor
    ///        - Q and R matrices for each sensor
    ///
    /// @todo Flag for publishing diagnostic parameters
    ///       - Measurement Data
    ///       - Any possible substates
    ///       - Verbosity?
    /// @todo Add high level logging flag for outputting camera and imu data into csv log files
    ///
    xCalibrateNode();

  private:
    ///
    /// @brief Function to load the sensor parameters from the given yaml configuration files
    ///
    void LoadSensorParams();

    ///
    /// @brief Constructor for visualization node
    /// @param cam_config_file_path File path used to load the camera calibration parameters
    /// @param distortion_mat Camera distortion matrix output
    /// @param camera_matrix Camera projection matrix output
    ///
    void ReadCameraConfig(std::string const &cam_config_file_path, cv::Mat &distortion_mat, cv::Mat &camera_matrix);

    ///
    /// @brief Callback function used for IMU measurements
    /// @param msg The received IMU sensor message
    ///
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    ///
    /// @brief Callback function used for camera measurements
    /// @param msg The received camera sensor message
    ///
    void CamCallback(const sensor_msgs::Image::ConstPtr &msg);

    ///
    /// @brief A helper function to transform an image to a rotation and translation
    /// @param imgPtr Pointer to the input image
    /// @param distCoeffs Distortion coefficients for the camera
    /// @param camMatrix Projection matrix for the camera
    /// @return (quaternion, vector3) rotation and translation of camera in the target reference frame
    ///
    std::tuple<bool, Eigen::Quaterniond, Eigen::Vector3d> CamMeasure(const cv_bridge::CvImagePtr &imgPtr,
                                                                     const cv::Mat &distCoeffs,
                                                                     const cv::Mat &camMatrix);

    ros::Publisher m_imgPub;
    ros::Publisher m_imuPub;
    void SendStaticTransform();
    void TransformCallback();
    xcalibrate::CalibrationEKF m_calibrationEKF;
    std::string m_baseImuName;

    // Ros Members
    ros::NodeHandle m_nodeHandle;
    ros::Time m_lastStamp = ros::Time::now();

    // Sensor Subscriber Vectors
    std::vector<ros::Subscriber> m_imuSubscribers;
    std::vector<ros::Subscriber> m_camSubscribers;
    std::vector<std::tuple<std::string, size_t>> m_imuSensors;
    std::vector<std::tuple<std::string, size_t, cv::Mat, cv::Mat>> m_camSensors;

    // Aruco Board Members
    // cv::Ptr<cv::aruco::Dictionary> m_dict {cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250)};
    // cv::Ptr<cv::aruco::Dictionary> m_dictPtr {std::shared_ptr<cv::aruco::Dictionary>(m_dict)};
    // cv::Ptr<cv::aruco::GridBoard> m_board {cv::aruco::GridBoard::create(7U, 5U, 0.031496, 0.00635, m_dictPtr, 0U)};

    // CharucoBoard Members
    cv::Ptr<cv::aruco::Dictionary> m_dict    = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> m_board = cv::aruco::CharucoBoard::create(7, 5, 0.0319278, 0.023876, m_dict);

    Eigen::Vector3d m_tgtPos;
    Eigen::Quaterniond m_tgtQuat;

    // Static Transform Members
    ros::Timer m_tfTimer;
    tf2_ros::TransformBroadcaster m_tfBroadcaster;
    tf2_ros::StaticTransformBroadcaster m_staticTfBroadcaster;

  public:
    tf2_ros::Buffer m_tfBuffer;    ///< @brief Buffer used to hold transforms
};

#endif