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

#include "xCalibrateNode.hpp"

#include "CalibrationEKF.hpp"
#include "RosHelper.hpp"
#include "TypeHelper.hpp"

// Eigen
#include <eigen3/Eigen/Eigen>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/opencv.hpp>

// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Standard
#include <functional>
#include <iomanip>
#include <iostream>
#include <stdint.h>
#include <string>
#include <tuple>
#include <vector>

xCalibrateNode::xCalibrateNode()
{
    m_nodeHandle = ros::NodeHandle("xcalibrate_node");

    LoadSensorParams();

    m_imgPub = m_nodeHandle.advertise<sensor_msgs::Image>("detections", 1000U);
    m_imuPub = m_nodeHandle.advertise<geometry_msgs::Vector3>("bias", 1000U);

    m_tfBroadcaster = tf2_ros::TransformBroadcaster();
    SendStaticTransform();
}

void xCalibrateNode::LoadSensorParams()
{
    // IMU Subscribers
    std::vector<std::string> imuExtrinsicList
        = RosHelper::ReadParam<std::vector<std::string>>(m_nodeHandle, "imuExtrinsicList");

    // Loop over list of IMU sensors
    bool baseImuInit = false;
    for (std::string &imuName : imuExtrinsicList)
    {
        // Get IMU initialization data
        std::vector<double> pos     = RosHelper::ReadParam<std::vector<double>>(m_nodeHandle, imuName + "/posInit");
        std::vector<double> quat    = RosHelper::ReadParam<std::vector<double>>(m_nodeHandle, imuName + "/quatInit");
        Eigen::Vector3d posInit     = TypeHelper::StdToEigVec(pos);
        Eigen::Quaterniond quatInit = TypeHelper::StdToEigQuat(quat);

        // Register new IMU
        size_t imuSensorIndex = m_calibrationEKF.RegisterImuExtrinsic(imuName, posInit, quatInit);

        if (!baseImuInit)
        {
            // Base IMU member variables set with first IMU
            m_baseImuName = imuName;
            baseImuInit   = true;

            if (imuSensorIndex != 0U)
            {
                ROS_ERROR("XCALIBRATE: Base IMU not first registered IMU");
            }
        }

        // Subscribe to IMU topic
        std::string topicName = RosHelper::ReadParam<std::string>(m_nodeHandle, imuName + "/topic");
        m_imuSubscribers.push_back(
            m_nodeHandle.subscribe<sensor_msgs::Imu>(topicName, 100U, &xCalibrateNode::ImuCallback, this));

        m_imuSensors.push_back(std::make_tuple(imuName, m_imuSubscribers.size() - 1U));
    }

    // Camera Subscribers
    std::vector<std::string> camExtrinsicList
        = RosHelper::ReadParam<std::vector<std::string>>(m_nodeHandle, "camExtrinsicList");

    // Loop over list of camera sensors
    std::string packagePath = ros::package::getPath("xcalibrate");
    for (std::string &camName : camExtrinsicList)
    {
        // Get Camera initialization data
        std::string calFile         = RosHelper::ReadParam<std::string>(m_nodeHandle, camName + "/calFile");
        std::vector<double> pos     = RosHelper::ReadParam<std::vector<double>>(m_nodeHandle, camName + "/posInit");
        std::vector<double> quat    = RosHelper::ReadParam<std::vector<double>>(m_nodeHandle, camName + "/quatInit");
        Eigen::Vector3d posInit     = TypeHelper::StdToEigVec(pos);
        Eigen::Quaterniond quatInit = TypeHelper::StdToEigQuat(quat);

        // Get Camera calibration data
        cv::Mat distortionCoefficients;
        cv::Mat camMatrix;
        ReadCameraConfig(packagePath + "/params/" + calFile, distortionCoefficients, camMatrix);

        // Register new Camera
        size_t camSensorIndex = m_calibrationEKF.RegisterCamExtrinsic(camName, posInit, quatInit);

        m_camSensors.push_back(std::make_tuple(camName, camSensorIndex, distortionCoefficients, camMatrix));

        // Subscribe to cam topic
        std::string topicName = RosHelper::ReadParam<std::string>(m_nodeHandle, camName + "/topic");
        m_camSubscribers.push_back(
            m_nodeHandle.subscribe<sensor_msgs::Image>(topicName, 100U, &xCalibrateNode::CamCallback, this));
    }

    // Initialize filter with parameters
    Eigen::Quaterniond imuQuat
        = TypeHelper::StdToEigQuat(RosHelper::ReadParam<std::vector<double>>(m_nodeHandle, "vectornav/quatInit"));
    Eigen::Vector3d gyroBias(0.0, 0.0, 0.0);
    Eigen::Vector3d imuVel(0.0, 0.0, 0.0);
    Eigen::Vector3d imuBias
        = TypeHelper::StdToEigVec(RosHelper::ReadParam<std::vector<double>>(m_nodeHandle, "vectornav/biasInit"));
    Eigen::Vector3d imuPos
        = TypeHelper::StdToEigVec(RosHelper::ReadParam<std::vector<double>>(m_nodeHandle, "vectornav/posInit"));
    Eigen::Quaterniond camQuat
        = TypeHelper::StdToEigQuat(RosHelper::ReadParam<std::vector<double>>(m_nodeHandle, "blackfly/quatInit"));
    Eigen::Vector3d camPos
        = TypeHelper::StdToEigVec(RosHelper::ReadParam<std::vector<double>>(m_nodeHandle, "blackfly/posInit"));
    m_calibrationEKF.Initialize(imuQuat, gyroBias, imuVel, imuBias, imuPos, camQuat, camPos);

    // Load target data
    m_tgtPos  = TypeHelper::StdToEigVec(RosHelper::ReadParam<std::vector<double>>(m_nodeHandle, "target_pos"));
    m_tgtQuat = TypeHelper::StdToEigQuat(RosHelper::ReadParam<std::vector<double>>(m_nodeHandle, "target_quat"));
    m_tgtQuat.normalize();
}

void xCalibrateNode::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    double sensorTime = msg->header.stamp.toSec();

    for (size_t i = 0U; i < m_imuSensors.size(); ++i)
    {
        std::string imuName;
        size_t imuIndex;
        std::tie(imuName, imuIndex) = m_imuSensors[i];

        if (imuName.compare(msg->header.frame_id) == 0)
        {
            m_lastStamp                = ros::Time::now();
            Eigen::Quaterniond imuQuat = TypeHelper::RosToEig(msg->orientation);
            Eigen::Vector3d imuAngVel  = TypeHelper::RosToEig(msg->angular_velocity);
            Eigen::Vector3d imuAcc     = TypeHelper::RosToEig(msg->linear_acceleration);

            m_calibrationEKF.ImuCallback(imuIndex, sensorTime, m_lastStamp.toSec(), imuQuat, imuAngVel, imuAcc);

            Eigen::Vector3d imuAccNED = imuQuat.toRotationMatrix() * imuAcc;
            m_imuPub.publish(TypeHelper::EigToRos(imuAccNED));

            TransformCallback();

            /// @todo: Add flag here to export to csv
            // std::cout << std::setw(12) << imuAngVel.x() << "," << std::setw(12) << imuAngVel.y() << "," <<
            // std::setw(12)
            //           << imuAngVel.z() << "," << std::setw(12) << imuAcc.x() << "," << std::setw(12) << imuAcc.y()
            //           << "," << std::setw(12) << imuAcc.z() << std::endl;

            return;
        }
    }

    ROS_ERROR("IMU topic header frame not found in list of names");
}

void xCalibrateNode::CamCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    double sensorTime = msg->header.stamp.toSec();

    for (size_t i = 0U; i < m_camSensors.size(); ++i)
    {
        std::string camName;
        size_t camIndex;
        cv::Mat distortionCoefficients;
        cv::Mat camMatrix;

        std::tie(camName, camIndex, distortionCoefficients, camMatrix) = m_camSensors[i];

        // if (camName.compare(msg->header.frame_id) == 0U)
        // {
        if (msg->height > 0U)
        {
            try
            {
                bool detectionFound;
                Eigen::Quaterniond measQuat;
                Eigen::Vector3d measPos;

                cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

                std::tie(detectionFound, measQuat, measPos) = CamMeasure(imgPtr, distortionCoefficients, camMatrix);

                // Only invoke camera callback on successful detection
                if (detectionFound)
                {
                    double computerTime = ros::Time::now().toSec();
                    static Eigen::Quaterniond q_n_l {0.8771663904190063, -0.0113869290798902, 0.0139230927452445,
                                                     0.4798494577407837};

                    // Convert target position to NED frame
                    Eigen::Quaterniond tgtQuatNED = q_n_l * m_tgtQuat;
                    Eigen::Vector3d tgtPosNED     = q_n_l.toRotationMatrix() * m_tgtPos;
                    m_calibrationEKF.CamCallback(camIndex, sensorTime, computerTime, measQuat, measPos, tgtQuatNED,
                                                 tgtPosNED);
                }

                return;
            }
            catch (cv_bridge::Exception &err)
            {
                ROS_ERROR("cv_bridge exception: %s", err.what());
                return;
            }
        }
        ROS_WARN("Invalid Image Received");
        return;
        // }
    }

    ROS_ERROR("cam topic header frame not found in list of names");
}

std::tuple<bool, Eigen::Quaterniond, Eigen::Vector3d> xCalibrateNode::CamMeasure(const cv_bridge::CvImagePtr &imgPtr,
                                                                                 const cv::Mat &distortionCoefficients,
                                                                                 const cv::Mat &camMatrix)
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<std::vector<cv::Point2f>> rejectedCandidates;

    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    // parameters->markerBorderBits                      = 1U;

    cv::aruco::detectMarkers(imgPtr->image, m_dict, markerCorners, markerIds, parameters, rejectedCandidates, camMatrix,
                             distortionCoefficients);
    bool detectionFound {false};
    cv::Vec3d rVector;
    cv::Vec3d tVector;
    tf2::Quaternion tf2_quat;

    // Only estimate board position if markers are found
    if (markerIds.size() > 0U)
    {
        cv::aruco::drawDetectedMarkers(imgPtr->image, markerCorners, markerIds);
        // cv::aruco::drawDetectedCornersCharuco(imgPtr->image, markerCorners, markerIds);

        /// @todo use the zero output for a check here
        cv::aruco::estimatePoseBoard(markerCorners, markerIds, m_board, camMatrix, distortionCoefficients, rVector,
                                     tVector, false);

        geometry_msgs::TransformStamped tfStamped;

        // convert the rotation vector to a rotation matrix
        cv::Mat rMat(3U, 3U, CV_64F);
        cv::Rodrigues(rVector, rMat);

        tf2::Matrix3x3 tf2Rot(rMat.at<double>(0U, 0U), rMat.at<double>(0U, 1U), rMat.at<double>(0U, 2U),
                              rMat.at<double>(1U, 0U), rMat.at<double>(1U, 1U), rMat.at<double>(1U, 2U),
                              rMat.at<double>(2U, 0U), rMat.at<double>(2U, 1U), rMat.at<double>(2U, 2U));

        tf2Rot.getRotation(tf2_quat);

        detectionFound = true;

        cv::aruco::drawDetectedMarkers(imgPtr->image, markerCorners, markerIds);
        cv::aruco::drawAxis(imgPtr->image, camMatrix, distortionCoefficients, rVector, tVector, 0.1);
        m_imgPub.publish(imgPtr->toImageMsg());

        /// @todo: Add flag here to export to csv
        std::cout << std::setw(12) << tf2_quat.w() << "," << std::setw(12) << tf2_quat.x() << "," << std::setw(12)
                  << tf2_quat.y() << "," << std::setw(12) << tf2_quat.z() << "," << std::setw(12) << tVector[0U] << ","
                  << std::setw(12) << tVector[1U] << "," << std::setw(12) << tVector[2U] << std::endl;
    }
    else
    {
        std::cout << std::setw(12) << 0.0 << "," << std::setw(12) << 0.0 << "," << std::setw(12) << 0.0 << ","
                  << std::setw(12) << 0.0 << "," << std::setw(12) << 0.0 << "," << std::setw(12) << 0.0 << ","
                  << std::setw(12) << 0.0 << std::endl;
    }
    return std::make_tuple(detectionFound, TypeHelper::Tf2ToEig(tf2_quat), TypeHelper::CvToEig(tVector));
}

void xCalibrateNode::TransformCallback()
{
    xcalibrate::SensorExtrinsics sensorExtrinsics = m_calibrationEKF.GetSensorExtrinsics();

    std::string name;
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;
    geometry_msgs::TransformStamped tfStamped;

    for (auto &elem : sensorExtrinsics)
    {
        std::tie(name, pos, quat) = elem;
        tfStamped.header.stamp    = m_lastStamp;

        if (name == m_baseImuName)
        {
            tfStamped.header.frame_id = "NED";
            tfStamped.child_frame_id  = m_baseImuName;
        }
        else
        {
            tfStamped.header.frame_id = m_baseImuName;
            tfStamped.child_frame_id  = name;
        }
        tfStamped.transform.translation.x = pos.x();
        tfStamped.transform.translation.y = pos.y();
        tfStamped.transform.translation.z = pos.z();
        tfStamped.transform.rotation.w    = 1.0;
        tfStamped.transform.rotation.x    = 0.0;
        tfStamped.transform.rotation.y    = 0.0;
        tfStamped.transform.rotation.z    = 0.0;

        m_tfBroadcaster.sendTransform(tfStamped);
    }
}

void xCalibrateNode::ReadCameraConfig(std::string const &cam_config_file_path, cv::Mat &distortion_mat,
                                      cv::Mat &camera_matrix)
{
    cv::FileStorage fs;

    fs.open(cam_config_file_path, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
    if (!fs.isOpened())
    {
        ROS_ERROR_STREAM("Failed to open: " << cam_config_file_path);
    }
    else
    {
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> distortion_mat;
        ROS_INFO_STREAM("Camera calibration loaded" << camera_matrix << distortion_mat);
    }
}

void xCalibrateNode::SendStaticTransform()
{
    geometry_msgs::TransformStamped tfStamped;

    // NED to World
    tfStamped.header.stamp            = ros::Time::now();
    tfStamped.header.frame_id         = "world";
    tfStamped.child_frame_id          = "NED";
    tfStamped.transform.translation.x = 0.0;
    tfStamped.transform.translation.y = 0.0;
    tfStamped.transform.translation.z = 0.0;
    tfStamped.transform.rotation.w    = 0.0;
    tfStamped.transform.rotation.x    = 1.0;
    tfStamped.transform.rotation.y    = 0.0;
    tfStamped.transform.rotation.z    = 0.0;
    m_staticTfBroadcaster.sendTransform(tfStamped);

    // Target to NED
    tfStamped.header.stamp            = ros::Time::now();
    tfStamped.header.frame_id         = "NED";
    tfStamped.child_frame_id          = "lab";
    tfStamped.transform.translation.x = 0.0;
    tfStamped.transform.translation.y = 0.0;
    tfStamped.transform.translation.z = 0.0;
    tfStamped.transform.rotation.w    = -0.47985;
    tfStamped.transform.rotation.x    = -0.01392;
    tfStamped.transform.rotation.y    = -0.01139;
    tfStamped.transform.rotation.z    = 0.87717;

    m_staticTfBroadcaster.sendTransform(tfStamped);

    // Target to NED
    tfStamped.header.stamp            = ros::Time::now();
    tfStamped.header.frame_id         = "lab";
    tfStamped.child_frame_id          = "target";
    tfStamped.transform.translation.x = m_tgtPos.x();
    tfStamped.transform.translation.y = m_tgtPos.y();
    tfStamped.transform.translation.z = m_tgtPos.z();
    tfStamped.transform.rotation.w    = m_tgtQuat.w();
    tfStamped.transform.rotation.x    = m_tgtQuat.x();
    tfStamped.transform.rotation.y    = m_tgtQuat.y();
    tfStamped.transform.rotation.z    = m_tgtQuat.z();
    m_staticTfBroadcaster.sendTransform(tfStamped);
}
