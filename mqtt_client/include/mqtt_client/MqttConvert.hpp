/*
==============================================================================
MIT License

Copyright 2022 Institute for Automotive Engineering of RWTH Aachen University.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
==============================================================================
*/


#pragma once
#include <iostream>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <sensor_msgs/msg/joy.hpp>


#ifdef HAVE_TRIORB_INTERFACE
#include <string>
#include <regex>
#define GET_NODE_NAME(s) std::regex_replace(std::string(getenv("ROS_PREFIX")) + std::string("_") + std::string(s), std::regex("^_"), "")
#define GET_TOPIC_NAME(s) std::regex_replace(std::string(getenv("ROS_PREFIX")) + std::string(s), std::regex("//"), "/")

#include <triorb_collaboration_interface/msg/parent_bind.hpp>

#include <triorb_cv_interface/msg/bounding_box.hpp>
#include <triorb_cv_interface/msg/detection.hpp>

#include <triorb_drive_interface/msg/drive_gains.hpp>
#include <triorb_drive_interface/msg/motor_params.hpp>
#include <triorb_drive_interface/msg/motor_status.hpp>
#include <triorb_drive_interface/msg/route.hpp>
#include <triorb_drive_interface/msg/triorb_align_pos3.hpp>
#include <triorb_drive_interface/msg/triorb_pos3.hpp>
#include <triorb_drive_interface/msg/triorb_pos3_stamped.hpp>
#include <triorb_drive_interface/msg/triorb_run_pos3.hpp>
#include <triorb_drive_interface/msg/triorb_run_result.hpp>
#include <triorb_drive_interface/msg/triorb_run_result_stamped.hpp>
#include <triorb_drive_interface/msg/triorb_run_state.hpp>
#include <triorb_drive_interface/msg/triorb_run_setting.hpp>
#include <triorb_drive_interface/msg/triorb_run_vel3.hpp>
#include <triorb_drive_interface/msg/triorb_run_vel3_stamped.hpp>
#include <triorb_drive_interface/msg/triorb_set_path.hpp>
#include <triorb_drive_interface/msg/triorb_set_pos3.hpp>
#include <triorb_drive_interface/msg/triorb_speed.hpp>
#include <triorb_drive_interface/msg/triorb_vel3.hpp>

#include <triorb_sensor_interface/msg/camera_device.hpp>
#include <triorb_sensor_interface/msg/distance_sensor.hpp>
#include <triorb_sensor_interface/msg/imu_sensor.hpp>
#include <triorb_sensor_interface/msg/obstacles.hpp>

#include <triorb_slam_interface/msg/cameras_landmark_info.hpp>
#include <triorb_slam_interface/msg/cameras_pose.hpp>
#include <triorb_slam_interface/msg/point_array_stamped.hpp>
#include <triorb_slam_interface/msg/pose_dev_stamped.hpp>
#include <triorb_slam_interface/msg/u_int32_multi_array_stamped.hpp>
#include <triorb_slam_interface/msg/xy_array_stamped.hpp>
#include <triorb_slam_interface/msg/slam_status.hpp>

#include <triorb_static_interface/msg/clock_sync.hpp>
#include <triorb_static_interface/msg/host_status.hpp>
#include <triorb_static_interface/msg/node_info.hpp>
#include <triorb_static_interface/msg/robot_error.hpp>
#include <triorb_static_interface/msg/robot_status.hpp>
#include <triorb_static_interface/msg/setting_i_pv4.hpp>
#include <triorb_static_interface/msg/setting_ros.hpp>
#include <triorb_static_interface/msg/setting_ssid.hpp>
#include <triorb_static_interface/msg/string_list.hpp>

#endif //HAVE_TRIORB_INTERFACE

#include <nlohmann/json.hpp> 
using json = nlohmann::json;
// clang-format on


/**
 * @brief Namespace for the mqtt_client package
 */
namespace mqtt_client {
  void toHeader(const json& j_msg, std_msgs::msg::Header &msg);
  void toInt8MultiArray(const json& j_msg, std_msgs::msg::Int8MultiArray &msg);
  void toUInt8MultiArray(const json& j_msg, std_msgs::msg::UInt8MultiArray &msg);
  void toInt16MultiArray(const json& j_msg, std_msgs::msg::Int16MultiArray &msg);
  void toUInt16MultiArray(const json& j_msg, std_msgs::msg::UInt16MultiArray &msg);
  void toInt32MultiArray(const json& j_msg, std_msgs::msg::Int32MultiArray &msg);
  void toUInt32MultiArray(const json& j_msg, std_msgs::msg::UInt32MultiArray &msg);

  void toTransform(const json& j_msg, geometry_msgs::msg::Transform &msg);
  void toTransformStamped(const json& j_msg, geometry_msgs::msg::TransformStamped &msg);
  void toVector3(const json& j_msg, geometry_msgs::msg::Vector3 &msg);
  void toQuaternion(const json& j_msg, geometry_msgs::msg::Quaternion &msg);

  void toJoy(const json& j_msg, sensor_msgs::msg::Joy &msg);


  #ifdef HAVE_TRIORB_INTERFACE
  void toParentBind(const json& j_msg, triorb_collaboration_interface::msg::ParentBind &msg);
  
  void toBoundingBox(const json& j_msg, triorb_cv_interface::msg::BoundingBox &msg);
  void toDetection(const json& j_msg, triorb_cv_interface::msg::Detection &msg);

  void toDriveGains(const json& j_msg, triorb_drive_interface::msg::DriveGains &msg);
  void toMotorParams(const json& j_msg, triorb_drive_interface::msg::MotorParams &msg);
  void toMotorStatus(const json& j_msg, triorb_drive_interface::msg::MotorStatus &msg);
  void toRoute(const json& j_msg, triorb_drive_interface::msg::Route &msg);
  void toTriorbAlignPos3(const json& j_msg, triorb_drive_interface::msg::TriorbAlignPos3 &msg);
  void toTriorbPos3(const json& j_msg, triorb_drive_interface::msg::TriorbPos3 &msg);
  void toTriorbPos3Stamped(const json& j_msg, triorb_drive_interface::msg::TriorbPos3Stamped &msg);
  void toTriorbRunPos3(const json& j_msg, triorb_drive_interface::msg::TriorbRunPos3 &msg);
  void toTriorbRunResult(const json& j_msg, triorb_drive_interface::msg::TriorbRunResult &msg);
  void toTriorbRunResultStamped(const json& j_msg, triorb_drive_interface::msg::TriorbRunResultStamped &msg);
  void toTriorbRunState(const json& j_msg, triorb_drive_interface::msg::TriorbRunState &msg);
  void toTriorbRunSetting(const json& j_msg, triorb_drive_interface::msg::TriorbRunSetting &msg);
  void toTriorbRunVel3(const json& j_msg, triorb_drive_interface::msg::TriorbRunVel3 &msg);
  void toTriorbRunVel3Stamped(const json& j_msg, triorb_drive_interface::msg::TriorbRunVel3Stamped &msg);
  void toTriorbSetPath(const json& j_msg, triorb_drive_interface::msg::TriorbSetPath &msg);
  void toTriorbSetPos3(const json& j_msg, triorb_drive_interface::msg::TriorbSetPos3 &msg);
  void toTriorbSpeed(const json& j_msg, triorb_drive_interface::msg::TriorbSpeed &msg);
  void toTriorbVel3(const json& j_msg, triorb_drive_interface::msg::TriorbVel3 &msg);

  void toCameraDevice(const json& j_msg, triorb_sensor_interface::msg::CameraDevice &msg);
  void toDistanceSensor(const json& j_msg, triorb_sensor_interface::msg::DistanceSensor &msg);
  void toImuSensor(const json& j_msg, triorb_sensor_interface::msg::ImuSensor &msg);
  void toObstacles(const json& j_msg, triorb_sensor_interface::msg::Obstacles &msg);

  void toCamerasLandmarkInfo(const json& j_msg, triorb_slam_interface::msg::CamerasLandmarkInfo &msg);
  void toCamerasPose(const json& j_msg, triorb_slam_interface::msg::CamerasPose &msg);
  void toPointArrayStamped(const json& j_msg, triorb_slam_interface::msg::PointArrayStamped &msg);
  void toPoseDevStamped(const json& j_msg, triorb_slam_interface::msg::PoseDevStamped &msg);
  void toUInt32MultiArrayStamped(const json& j_msg, triorb_slam_interface::msg::UInt32MultiArrayStamped &msg);
  void toXyArrayStamped(const json& j_msg, triorb_slam_interface::msg::XyArrayStamped &msg);
  void toSlamStatus(const json& j_msg, triorb_slam_interface::msg::SlamStatus &msg);

  void toClockSync(const json& j_msg, triorb_static_interface::msg::ClockSync &msg);
  void toHostStatus(const json& j_msg, triorb_static_interface::msg::HostStatus &msg);
  void toNodeInfo(const json& j_msg, triorb_static_interface::msg::NodeInfo &msg);
  void toRobotError(const json& j_msg, triorb_static_interface::msg::RobotError &msg);
  void toRobotStatus(const json& j_msg, triorb_static_interface::msg::RobotStatus &msg);
  void toSettingIPv4(const json& j_msg, triorb_static_interface::msg::SettingIPv4 &msg);
  void toSettingROS(const json& j_msg, triorb_static_interface::msg::SettingROS &msg);
  void toSettingSSID(const json& j_msg, triorb_static_interface::msg::SettingSSID &msg);
  void toStringList(const json& j_msg, triorb_static_interface::msg::StringList &msg);
  #endif



  void fromHeader(const std_msgs::msg::Header& msg, json& j_msg);
  void fromInt8MultiArray(const std_msgs::msg::Int8MultiArray& msg, json& j_msg);
  void fromUInt8MultiArray(const std_msgs::msg::UInt8MultiArray& msg, json& j_msg);
  void fromInt16MultiArray(const std_msgs::msg::Int16MultiArray& msg, json& j_msg);
  void fromUInt16MultiArray(const std_msgs::msg::UInt16MultiArray& msg, json& j_msg);
  void fromInt32MultiArray(const std_msgs::msg::Int32MultiArray& msg, json& j_msg);
  void fromUInt32MultiArray(const std_msgs::msg::UInt32MultiArray& msg, json& j_msg);

  void fromTransform(const geometry_msgs::msg::Transform& msg, json& j_msg);
  void fromTransformStamped(const geometry_msgs::msg::TransformStamped& msg, json& j_msg);
  void fromVector3(const geometry_msgs::msg::Vector3& msg, json& j_msg);
  void fromQuaternion(const geometry_msgs::msg::Quaternion& msg, json& j_msg);

  void fromJoy(const sensor_msgs::msg::Joy& msg, json& j_msg);


  #ifdef HAVE_TRIORB_INTERFACE
  void fromParentBind(const triorb_collaboration_interface::msg::ParentBind& msg, json& j_msg);

  void fromBoundingBox(const triorb_cv_interface::msg::BoundingBox& msg, json& j_msg);
  void fromDetection(const triorb_cv_interface::msg::Detection& msg, json& j_msg);

  void fromDriveGains(const triorb_drive_interface::msg::DriveGains& msg, json& j_msg);
  void fromMotorParams(const triorb_drive_interface::msg::MotorParams& msg, json& j_msg);
  void fromMotorStatus(const triorb_drive_interface::msg::MotorStatus& msg, json& j_msg);
  void fromRoute(const triorb_drive_interface::msg::Route& msg, json& j_msg);
  void fromTriorbAlignPos3(const triorb_drive_interface::msg::TriorbAlignPos3& msg, json& j_msg);
  void fromTriorbPos3(const triorb_drive_interface::msg::TriorbPos3& msg, json& j_msg);
  void fromTriorbPos3Stamped(const triorb_drive_interface::msg::TriorbPos3Stamped& msg, json& j_msg);
  void fromTriorbRunPos3(const triorb_drive_interface::msg::TriorbRunPos3& msg, json& j_msg);
  void fromTriorbRunResult(const triorb_drive_interface::msg::TriorbRunResult& msg, json& j_msg);
  void fromTriorbRunResultStamped(const triorb_drive_interface::msg::TriorbRunResultStamped& msg, json& j_msg);
  void fromTriorbRunState(const triorb_drive_interface::msg::TriorbRunState& msg, json& j_msg);
  void fromTriorbRunSetting(const triorb_drive_interface::msg::TriorbRunSetting& msg, json& j_msg);
  void fromTriorbRunVel3(const triorb_drive_interface::msg::TriorbRunVel3& msg, json& j_msg);
  void fromTriorbRunVel3Stamped(const triorb_drive_interface::msg::TriorbRunVel3Stamped& msg, json& j_msg);
  void fromTriorbSetPath(const triorb_drive_interface::msg::TriorbSetPath& msg, json& j_msg);
  void fromTriorbSetPos3(const triorb_drive_interface::msg::TriorbSetPos3& msg, json& j_msg);
  void fromTriorbSpeed(const triorb_drive_interface::msg::TriorbSpeed& msg, json& j_msg);
  void fromTriorbVel3(const triorb_drive_interface::msg::TriorbVel3& msg, json& j_msg);

  void fromCameraDevice(const triorb_sensor_interface::msg::CameraDevice& msg, json& j_msg);
  void fromDistanceSensor(const triorb_sensor_interface::msg::DistanceSensor& msg, json& j_msg);
  void fromImuSensor(const triorb_sensor_interface::msg::ImuSensor& msg, json& j_msg);
  void fromObstacles(const triorb_sensor_interface::msg::Obstacles& msg, json& j_msg);

  void fromCamerasLandmarkInfo(const triorb_slam_interface::msg::CamerasLandmarkInfo& msg, json& j_msg);
  void fromCamerasPose(const triorb_slam_interface::msg::CamerasPose& msg, json& j_msg);
  void fromPointArrayStamped(const triorb_slam_interface::msg::PointArrayStamped& msg, json& j_msg);
  void fromPoseDevStamped(const triorb_slam_interface::msg::PoseDevStamped& msg, json& j_msg);
  void fromUInt32MultiArrayStamped(const triorb_slam_interface::msg::UInt32MultiArrayStamped& msg, json& j_msg);
  void fromXyArrayStamped(const triorb_slam_interface::msg::XyArrayStamped& msg, json& j_msg);
  void fromSlamStatus(const triorb_slam_interface::msg::SlamStatus& msg, json& j_msg);

  void fromClockSync(const triorb_static_interface::msg::ClockSync& msg, json& j_msg);
  void fromHostStatus(const triorb_static_interface::msg::HostStatus& msg, json& j_msg);
  void fromNodeInfo(const triorb_static_interface::msg::NodeInfo& msg, json& j_msg);
  void fromRobotError(const triorb_static_interface::msg::RobotError& msg, json& j_msg);
  void fromRobotStatus(const triorb_static_interface::msg::RobotStatus& msg, json& j_msg);
  void fromSettingIPv4(const triorb_static_interface::msg::SettingIPv4& msg, json& j_msg);
  void fromSettingROS(const triorb_static_interface::msg::SettingROS& msg, json& j_msg);
  void fromSettingSSID(const triorb_static_interface::msg::SettingSSID& msg, json& j_msg);
  void fromStringList(const triorb_static_interface::msg::StringList& msg, json& j_msg);
  #endif // HAVE_TRIORB_INTERFACE
}
