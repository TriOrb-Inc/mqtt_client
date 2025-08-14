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


#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>

// clang-format off
#include <mqtt_client/MqttClient.ros2.hpp>
#include <mqtt_client/MqttConvert.hpp>
#include <mqtt_client_interfaces/msg/ros_msg_type.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcpputils/env.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int8.hpp>



RCLCPP_COMPONENTS_REGISTER_NODE(mqtt_client::MqttClient)


namespace mqtt_client {


const std::string MqttClient::kRosMsgTypeMqttTopicPrefix =
  "mqtt_client/ros_msg_type/";

const std::string MqttClient::kLatencyRosTopicPrefix = "~/latencies/";

template <typename T>
T mqtt2float(mqtt::const_message_ptr mqtt_msg) {
  auto str_msg = mqtt_msg->to_string ();
  std::size_t pos;
  const T v = std::stold(str_msg, &pos);

  if (pos != str_msg.size())
    throw std::invalid_argument ("not all charaters processed");

  return v;
}

template <typename T>
T mqtt2int(mqtt::const_message_ptr mqtt_msg) {
  auto str_msg = mqtt_msg->to_string ();
  std::size_t pos;
  const T v = std::stoll(str_msg, &pos);

  if (pos != str_msg.size())
    throw std::invalid_argument ("not all charaters processed");

  return v;
}

bool mqtt2bool(mqtt::const_message_ptr mqtt_msg) {
  const std::string str_msg = mqtt_msg->to_string ();
  std::string bool_str = mqtt_msg->to_string ();
  std::transform(str_msg.cbegin(), str_msg.cend(), bool_str.begin(),
                 ::tolower);
  if (bool_str == "true" || bool_str == "1") return true;
  if (bool_str == "false" || bool_str == "0") return false;

  throw std::invalid_argument ("unable to decode string");
}

bool fixedMqtt2PrimitiveRos(mqtt::const_message_ptr mqtt_msg,
                            const std::string& msg_type,
                            rclcpp::SerializedMessage &serialized_msg) {
  try {
    if (msg_type == "std_msgs/msg/String") {
      std_msgs::msg::String msg;
      msg.data = mqtt_msg->to_string();

      serializeRosMessage(msg, serialized_msg);
    } else if (msg_type == "std_msgs/msg/Empty") {
      std_msgs::msg::Empty msg;

      serializeRosMessage(msg, serialized_msg);
    } else if (msg_type == "std_msgs/msg/Bool") {
      std_msgs::msg::Bool msg;
      msg.data = mqtt2bool(mqtt_msg);

      serializeRosMessage(msg, serialized_msg);
    } else if (msg_type == "std_msgs/msg/Char") {
      std_msgs::msg::Char msg;
      msg.data = mqtt2int<int8_t>(mqtt_msg);

      serializeRosMessage(msg, serialized_msg);
    } else if (msg_type == "std_msgs/msg/UInt8") {
      std_msgs::msg::UInt8 msg;
      msg.data = mqtt2int<uint8_t>(mqtt_msg);

      serializeRosMessage(msg, serialized_msg);
    } else if (msg_type == "std_msgs/msg/UInt16") {
      std_msgs::msg::UInt16 msg;
      msg.data = mqtt2int<uint16_t>(mqtt_msg);

      serializeRosMessage(msg, serialized_msg);
    } else if (msg_type == "std_msgs/msg/UInt32") {
      std_msgs::msg::UInt32 msg;
      msg.data = mqtt2int<uint32_t>(mqtt_msg);

      serializeRosMessage(msg, serialized_msg);
    } else if (msg_type == "std_msgs/msg/UInt64") {
      std_msgs::msg::UInt64 msg;
      msg.data = mqtt2int<uint64_t>(mqtt_msg);

      serializeRosMessage(msg, serialized_msg);
    } else if (msg_type == "std_msgs/msg/Int8") {
      std_msgs::msg::Int8 msg;
      msg.data = mqtt2int<int8_t>(mqtt_msg);

      serializeRosMessage(msg, serialized_msg);
    } else if (msg_type == "std_msgs/msg/Int16") {
      std_msgs::msg::Int16 msg;
      msg.data = mqtt2int<int16_t>(mqtt_msg);

      serializeRosMessage(msg, serialized_msg);
    } else if (msg_type == "std_msgs/msg/Int32") {
      std_msgs::msg::Int32 msg;
      msg.data = mqtt2int<int32_t>(mqtt_msg);

      serializeRosMessage(msg, serialized_msg);
    } else if (msg_type == "std_msgs/msg/Int64") {
      std_msgs::msg::Int32 msg;
      msg.data = mqtt2int<int32_t>(mqtt_msg);

      serializeRosMessage(msg, serialized_msg);
    } else if (msg_type == "std_msgs/msg/Float32") {
      std_msgs::msg::Float32 msg;
      msg.data = mqtt2float<float>(mqtt_msg);

      serializeRosMessage(msg, serialized_msg);
    } else if (msg_type == "std_msgs/msg/Float64") {
      std_msgs::msg::Float64 msg;
      msg.data = mqtt2float<double>(mqtt_msg);

      serializeRosMessage(msg, serialized_msg);
    } else {
      try {
        json j_msg = json::parse(mqtt_msg->to_string());
        // 各Keyが存在しない場合はデフォルト値を代入する
        if (msg_type == "std_msgs/msg/Header") {
          std_msgs::msg::Header msg;
          toHeader(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
  
        } else if (msg_type == "std_msgs/msg/Int8MultiArray") {
          // layout非対応のため１回だけ警告を出す
          RCLCPP_WARN_ONCE(
            rclcpp::get_logger("mqtt_client"),
            "Int8MultiArray layout is not supported. Only data is used.");
          std_msgs::msg::Int8MultiArray msg;
          toInt8MultiArray(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        } else if (msg_type == "std_msgs/msg/UInt8MultiArray") {
          // layout非対応のため１回だけ警告を出す
          RCLCPP_WARN_ONCE(
            rclcpp::get_logger("mqtt_client"),
            "UInt8MultiArray layout is not supported. Only data is used.");
          std_msgs::msg::UInt8MultiArray msg;
          toUInt8MultiArray(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        } else if (msg_type == "std_msgs/msg/Int16MultiArray") {
          // layout非対応のため１回だけ警告を出す
          RCLCPP_WARN_ONCE(
            rclcpp::get_logger("mqtt_client"),
            "Int16MultiArray layout is not supported. Only data is used.");
          std_msgs::msg::Int16MultiArray msg;
          toInt16MultiArray(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        } else if (msg_type == "std_msgs/msg/UInt16MultiArray") {
          // layout非対応のため１回だけ警告を出す
          RCLCPP_WARN_ONCE(
            rclcpp::get_logger("mqtt_client"),
            "UInt16MultiArray layout is not supported. Only data is used.");
          std_msgs::msg::UInt16MultiArray msg;
          toUInt16MultiArray(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        } else if (msg_type == "std_msgs/msg/Int32MultiArray") {
          // layout非対応のため１回だけ警告を出す
          RCLCPP_WARN_ONCE(
            rclcpp::get_logger("mqtt_client"),
            "Int32MultiArray layout is not supported. Only data is used.");
          std_msgs::msg::Int32MultiArray msg;
          toInt32MultiArray(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        } else if (msg_type == "std_msgs/msg/UInt32MultiArray") {
          // layout非対応のため１回だけ警告を出す
          RCLCPP_WARN_ONCE(
            rclcpp::get_logger("mqtt_client"),
            "UInt32MultiArray layout is not supported. Only data is used.");
          std_msgs::msg::UInt32MultiArray msg;
          toUInt32MultiArray(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        } else if (msg_type == "geometry_msgs/msg/Transform") {
          geometry_msgs::msg::Transform msg;
          toTransform(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        } else if (msg_type == "geometry_msgs/msg/TransformStamped") {
          geometry_msgs::msg::TransformStamped msg;
          toTransformStamped(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        } else if (msg_type == "geometry_msgs/msg/Vector3") {
          geometry_msgs::msg::Vector3 msg;
          toVector3(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        } else if (msg_type == "geometry_msgs/msg/Quaternion") {
          geometry_msgs::msg::Quaternion msg;
          toQuaternion(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        } else if (msg_type == "sensor_msgs/msg/Joy"){
          sensor_msgs::msg::Joy msg;
          toJoy(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
  #ifdef HAVE_TRIORB_INTERFACE
        else if (msg_type == "triorb_collaboration_interface/msg/ParentBind") {
          triorb_collaboration_interface::msg::ParentBind msg;
          toParentBind(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_cv_interface/msg/BoundingBox") {
          triorb_cv_interface::msg::BoundingBox msg;
          toBoundingBox(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_cv_interface/msg/Detection") {
          triorb_cv_interface::msg::Detection msg;
          toDetection(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/DriveGains") {
          triorb_drive_interface::msg::DriveGains msg;
          toDriveGains(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/MotorParams") {
          triorb_drive_interface::msg::MotorParams msg;
          toMotorParams(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/MotorStatus") {
          triorb_drive_interface::msg::MotorStatus msg;
          toMotorStatus(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/Route") {
          triorb_drive_interface::msg::Route msg;
          toRoute(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/TriorbAlignPos3") {
          triorb_drive_interface::msg::TriorbAlignPos3 msg;
          toTriorbAlignPos3(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/TriorbPos3") {
          triorb_drive_interface::msg::TriorbPos3 msg;
          toTriorbPos3(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/TriorbPos3Stamped") {
          triorb_drive_interface::msg::TriorbPos3Stamped msg;
          toTriorbPos3Stamped(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/TriorbRunPos3") {
          triorb_drive_interface::msg::TriorbRunPos3 msg;
          toTriorbRunPos3(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/TriorbRunResult") {
          triorb_drive_interface::msg::TriorbRunResult msg;
          toTriorbRunResult(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/TriorbRunResultStamped") {
          triorb_drive_interface::msg::TriorbRunResultStamped msg;
          toTriorbRunResultStamped(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/TriorbRunState") {
          triorb_drive_interface::msg::TriorbRunState msg;
          toTriorbRunState(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/TriorbRunSetting") {
          triorb_drive_interface::msg::TriorbRunSetting msg;
          toTriorbRunSetting(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/TriorbRunVel3") {
          triorb_drive_interface::msg::TriorbRunVel3 msg;
          toTriorbRunVel3(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/TriorbRunVel3Stamped") {
          triorb_drive_interface::msg::TriorbRunVel3Stamped msg;
          toTriorbRunVel3Stamped(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/TriorbSetPath") {
          triorb_drive_interface::msg::TriorbSetPath msg;
          toTriorbSetPath(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/TriorbSetPos3") {
          triorb_drive_interface::msg::TriorbSetPos3 msg;
          toTriorbSetPos3(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/TriorbSpeed") {
          triorb_drive_interface::msg::TriorbSpeed msg;
          toTriorbSpeed(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_drive_interface/msg/TriorbVel3") {
          triorb_drive_interface::msg::TriorbVel3 msg;
          toTriorbVel3(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_sensor_interface/msg/CameraDevice") {
          triorb_sensor_interface::msg::CameraDevice msg;
          toCameraDevice(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_sensor_interface/msg/DistanceSensor") {
          triorb_sensor_interface::msg::DistanceSensor msg;
          toDistanceSensor(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_sensor_interface/msg/ImuSensor") {
          triorb_sensor_interface::msg::ImuSensor msg;
          toImuSensor(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_sensor_interface/msg/Obstacles") {
          triorb_sensor_interface::msg::Obstacles msg;
          toObstacles(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_slam_interface/msg/CamerasLandmarkInfo") {
          triorb_slam_interface::msg::CamerasLandmarkInfo msg;
          toCamerasLandmarkInfo(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_slam_interface/msg/CamerasPose") {
          triorb_slam_interface::msg::CamerasPose msg;
          toCamerasPose(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_slam_interface/msg/PointArrayStamped") {
          triorb_slam_interface::msg::PointArrayStamped msg;
          toPointArrayStamped(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_slam_interface/msg/PoseDevStamped") {
          triorb_slam_interface::msg::PoseDevStamped msg;
          toPoseDevStamped(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type ==
                 "triorb_slam_interface/msg/UInt32MultiArrayStamped") {
          triorb_slam_interface::msg::UInt32MultiArrayStamped msg;
          toUInt32MultiArrayStamped(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_slam_interface/msg/XyArrayStamped") {
          triorb_slam_interface::msg::XyArrayStamped msg;
          toXyArrayStamped(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_static_interface/msg/ClockSync") {
          triorb_static_interface::msg::ClockSync msg;
          toClockSync(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_static_interface/msg/HostStatus") {
          triorb_static_interface::msg::HostStatus msg;
          toHostStatus(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_static_interface/msg/NodeInfo") {
          triorb_static_interface::msg::NodeInfo msg;
          toNodeInfo(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_static_interface/msg/RobotError") {
          triorb_static_interface::msg::RobotError msg;
          toRobotError(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_static_interface/msg/RobotStatus") {
          triorb_static_interface::msg::RobotStatus msg;
          toRobotStatus(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_static_interface/msg/SettingIPv4") {
          triorb_static_interface::msg::SettingIPv4 msg;
          toSettingIPv4(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_static_interface/msg/SettingROS") {
          triorb_static_interface::msg::SettingROS msg;
          toSettingROS(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_static_interface/msg/SettingSSID") {
          triorb_static_interface::msg::SettingSSID msg;
          toSettingSSID(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
        else if (msg_type == "triorb_static_interface/msg/StringList") {
          triorb_static_interface::msg::StringList msg;
          toStringList(j_msg, msg);
          serializeRosMessage(msg, serialized_msg);
        }
  #endif  // HAVE_TRIORB_INTERFACE
        else {
          throw std::domain_error("Unhandled message type (" + msg_type + ")");
        }
      }catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_client"),
                     "Error parsing message: %s", e.what());
#ifdef HAVE_TRIORB_INTERFACE
        //std_msgs::msg::String _msg; _msg.data = "mqrr_client / Error parsing message";
        //this->pub_except_error_str_add_->publish(_msg); // 
#endif // HAVE_TRIORB_INTERFACE
        return false;
      }
    }

    return true;

  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    std::cerr << "Failed to deserialize message: " << msg_type << std::endl;
    std::cerr << "Serialized message: " << mqtt_msg << std::endl;
    return false;
  }
}

/**
 * @brief Extracts string of primitive data types from ROS message.
 *
 * This is helpful to extract the actual data payload of a primitive ROS
 * message. If e.g. an std_msgs/msg/String is serialized to a string
 * representation, it also contains the field name 'data'. This function
 * instead returns the underlying value as string.
 *
 * @param [in]  serialized_msg  generic serialized ROS message
 * @param [in]  msg_type        ROS message type, e.g. `std_msgs/msg/String`
 * @param [out] primitive       string representation of primitive message data
 *
 * @return true  if primitive ROS message type was found
 * @return false if ROS message type is not primitive
 */
bool primitiveRosMessageToString(
  const std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg,
  const std::string& msg_type, std::string& primitive) {

  bool found_primitive = true;

  if (msg_type == "std_msgs/msg/String") {
    std_msgs::msg::String msg;
    deserializeRosMessage(*serialized_msg, msg);
    primitive = msg.data;
  } else if (msg_type == "std_msgs/msg/Empty") {
    std_msgs::msg::Empty msg;
    deserializeRosMessage(*serialized_msg, msg);
    primitive = std::string();
  } else if (msg_type == "std_msgs/msg/Bool") {
    std_msgs::msg::Bool msg;
    deserializeRosMessage(*serialized_msg, msg);
    primitive = msg.data ? "true" : "false";
  } else if (msg_type == "std_msgs/msg/Char") {
    std_msgs::msg::Char msg;
    deserializeRosMessage(*serialized_msg, msg);
    primitive = std::to_string(msg.data);
  } else if (msg_type == "std_msgs/msg/UInt8") {
    std_msgs::msg::UInt8 msg;
    deserializeRosMessage(*serialized_msg, msg);
    primitive = std::to_string(msg.data);
  } else if (msg_type == "std_msgs/msg/UInt16") {
    std_msgs::msg::UInt16 msg;
    deserializeRosMessage(*serialized_msg, msg);
    primitive = std::to_string(msg.data);
  } else if (msg_type == "std_msgs/msg/UInt32") {
    std_msgs::msg::UInt32 msg;
    deserializeRosMessage(*serialized_msg, msg);
    primitive = std::to_string(msg.data);
  } else if (msg_type == "std_msgs/msg/UInt64") {
    std_msgs::msg::UInt64 msg;
    deserializeRosMessage(*serialized_msg, msg);
    primitive = std::to_string(msg.data);
  } else if (msg_type == "std_msgs/msg/Int8") {
    std_msgs::msg::Int8 msg;
    deserializeRosMessage(*serialized_msg, msg);
    primitive = std::to_string(msg.data);
  } else if (msg_type == "std_msgs/msg/Int16") {
    std_msgs::msg::Int16 msg;
    deserializeRosMessage(*serialized_msg, msg);
    primitive = std::to_string(msg.data);
  } else if (msg_type == "std_msgs/msg/Int32") {
    std_msgs::msg::Int32 msg;
    deserializeRosMessage(*serialized_msg, msg);
    primitive = std::to_string(msg.data);
  } else if (msg_type == "std_msgs/msg/Int64") {
    std_msgs::msg::Int64 msg;
    deserializeRosMessage(*serialized_msg, msg);
    primitive = std::to_string(msg.data);
  } else if (msg_type == "std_msgs/msg/Float32") {
    std_msgs::msg::Float32 msg;
    deserializeRosMessage(*serialized_msg, msg);
    primitive = std::to_string(msg.data);
  } else if (msg_type == "std_msgs/msg/Float64") {
    std_msgs::msg::Float64 msg;
    deserializeRosMessage(*serialized_msg, msg);
    primitive = std::to_string(msg.data);
  } else {
    json j_msg = json();
    if (msg_type == "std_msgs/msg/Header") {
      std_msgs::msg::Header msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromHeader(msg, j_msg);
    } else if (msg_type == "std_msgs/msg/Int8MultiArray") {
      // layout非対応のため1回だけ警告を出す
      RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mqtt_client"),
        "std_msgs/msg/Int8MultiArray layout is not supported");
      std_msgs::msg::Int8MultiArray msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromInt8MultiArray(msg, j_msg);
    } else if (msg_type == "std_msgs/msg/UInt8MultiArray") {
      // layout非対応のため1回だけ警告を出す
      RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mqtt_client"),
        "std_msgs/msg/UInt8MultiArray layout is not supported");
      std_msgs::msg::UInt8MultiArray msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromUInt8MultiArray(msg, j_msg);
    } else if (msg_type == "std_msgs/msg/Int16MultiArray") {
      // layout非対応のため1回だけ警告を出す
      RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mqtt_client"),
        "std_msgs/msg/Int16MultiArray layout is not supported");
      std_msgs::msg::Int16MultiArray msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromInt16MultiArray(msg, j_msg);
    } else if (msg_type == "std_msgs/msg/UInt16MultiArray") {
      // layout非対応のため1回だけ警告を出す
      RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mqtt_client"),
        "std_msgs/msg/UInt16MultiArray layout is not supported");
      std_msgs::msg::UInt16MultiArray msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromUInt16MultiArray(msg, j_msg);
    } else if (msg_type == "std_msgs/msg/Int32MultiArray") {
      // layout非対応のため1回だけ警告を出す
      RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mqtt_client"),
        "std_msgs/msg/Int32MultiArray layout is not supported");
      std_msgs::msg::Int32MultiArray msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromInt32MultiArray(msg, j_msg);
    } else if (msg_type == "std_msgs/msg/UInt32MultiArray") {
      // layout非対応のため1回だけ警告を出す
      RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mqtt_client"),
        "std_msgs/msg/UInt32MultiArray layout is not supported");
      std_msgs::msg::UInt32MultiArray msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromUInt32MultiArray(msg, j_msg);
    } else if (msg_type == "geometry_msgs/msg/Transform") {
      geometry_msgs::msg::Transform msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTransform(msg, j_msg);
    } else if (msg_type == "geometry_msgs/msg/TransformStamped") {
      geometry_msgs::msg::TransformStamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTransformStamped(msg, j_msg);
    } else if (msg_type == "geometry_msgs/msg/Vector3") {
      geometry_msgs::msg::Vector3 msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromVector3(msg, j_msg);
    } else if (msg_type == "geometry_msgs/msg/Quaternion") {
      geometry_msgs::msg::Quaternion msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromQuaternion(msg, j_msg);
    } else if (msg_type == "sensor_msgs/msg/Joy"){
      sensor_msgs::msg::Joy msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromJoy(msg, j_msg);
    }
#ifdef HAVE_TRIORB_INTERFACE
    else if (msg_type == "triorb_collaboration_interface/msg/ParentBind") {
      triorb_collaboration_interface::msg::ParentBind msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromParentBind(msg, j_msg);
    }
    else if (msg_type == "triorb_cv_interface/msg/BoundingBox") {
      triorb_cv_interface::msg::BoundingBox msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromBoundingBox(msg, j_msg);
    }
    else if (msg_type == "triorb_cv_interface/msg/Detection") {
      triorb_cv_interface::msg::Detection msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromDetection(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/DriveGains") {
      triorb_drive_interface::msg::DriveGains msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromDriveGains(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/MotorParams") {
      triorb_drive_interface::msg::MotorParams msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromMotorParams(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/MotorStatus") {
      triorb_drive_interface::msg::MotorStatus msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromMotorStatus(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/Route") {
      triorb_drive_interface::msg::Route msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromRoute(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/TriorbAlignPos3") {
      triorb_drive_interface::msg::TriorbAlignPos3 msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTriorbAlignPos3(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/TriorbPos3") {
      triorb_drive_interface::msg::TriorbPos3 msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTriorbPos3(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/TriorbPos3Stamped") {
      triorb_drive_interface::msg::TriorbPos3Stamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTriorbPos3Stamped(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/TriorbRunPos3") {
      triorb_drive_interface::msg::TriorbRunPos3 msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTriorbRunPos3(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/TriorbRunResult") {
      triorb_drive_interface::msg::TriorbRunResult msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTriorbRunResult(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/TriorbRunResultStamped") {
      triorb_drive_interface::msg::TriorbRunResultStamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTriorbRunResultStamped(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/TriorbRunState") {
      triorb_drive_interface::msg::TriorbRunState msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTriorbRunState(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/TriorbRunSetting") {
      triorb_drive_interface::msg::TriorbRunSetting msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTriorbRunSetting(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/TriorbRunVel3") {
      triorb_drive_interface::msg::TriorbRunVel3 msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTriorbRunVel3(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/TriorbRunVel3Stamped"){
      triorb_drive_interface::msg::TriorbRunVel3Stamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTriorbRunVel3Stamped(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/TriorbSetPath") {
      triorb_drive_interface::msg::TriorbSetPath msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTriorbSetPath(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/TriorbSetPos3") {
      triorb_drive_interface::msg::TriorbSetPos3 msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTriorbSetPos3(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/TriorbSpeed") {
      triorb_drive_interface::msg::TriorbSpeed msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTriorbSpeed(msg, j_msg);
    }
    else if (msg_type == "triorb_drive_interface/msg/TriorbVel3") {
      triorb_drive_interface::msg::TriorbVel3 msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromTriorbVel3(msg, j_msg);
    }
    else if (msg_type == "triorb_sensor_interface/msg/CameraDevice") {
      triorb_sensor_interface::msg::CameraDevice msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromCameraDevice(msg, j_msg);
    }
    else if (msg_type == "triorb_sensor_interface/msg/DistanceSensor") {
      triorb_sensor_interface::msg::DistanceSensor msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromDistanceSensor(msg, j_msg);
    }
    else if (msg_type == "triorb_sensor_interface/msg/ImuSensor") {
      triorb_sensor_interface::msg::ImuSensor msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromImuSensor(msg, j_msg);
    }
    else if (msg_type == "triorb_sensor_interface/msg/Obstacles") {
      triorb_sensor_interface::msg::Obstacles msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromObstacles(msg, j_msg);
    }
    else if (msg_type == "triorb_slam_interface/msg/CamerasLandmarkInfo") {
      triorb_slam_interface::msg::CamerasLandmarkInfo msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromCamerasLandmarkInfo(msg, j_msg);
    }
    else if (msg_type == "triorb_slam_interface/msg/CamerasPose") {
      triorb_slam_interface::msg::CamerasPose msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromCamerasPose(msg, j_msg);
    }
    else if (msg_type == "triorb_slam_interface/msg/PointArrayStamped") {
      triorb_slam_interface::msg::PointArrayStamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromPointArrayStamped(msg, j_msg);
    }
    else if (msg_type == "triorb_slam_interface/msg/PoseDevStamped") {
      triorb_slam_interface::msg::PoseDevStamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromPoseDevStamped(msg, j_msg);
    }
    else if (msg_type == "triorb_slam_interface/msg/UInt32MultiArrayStamped") {
      triorb_slam_interface::msg::UInt32MultiArrayStamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromUInt32MultiArrayStamped(msg, j_msg);
    }
    else if (msg_type == "triorb_slam_interface/msg/XyArrayStamped") {
      triorb_slam_interface::msg::XyArrayStamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromXyArrayStamped(msg, j_msg);
    }
    else if (msg_type == "triorb_static_interface/msg/ClockSync") {
      triorb_static_interface::msg::ClockSync msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromClockSync(msg, j_msg);
    }
    else if (msg_type == "triorb_static_interface/msg/HostStatus") {
      triorb_static_interface::msg::HostStatus msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromHostStatus(msg, j_msg);
    }
    else if (msg_type == "triorb_static_interface/msg/NodeInfo") {
      triorb_static_interface::msg::NodeInfo msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromNodeInfo(msg, j_msg);
    }
    else if (msg_type == "triorb_static_interface/msg/RobotError") {
      triorb_static_interface::msg::RobotError msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromRobotError(msg, j_msg);
    }
    else if (msg_type == "triorb_static_interface/msg/RobotStatus") {
      triorb_static_interface::msg::RobotStatus msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromRobotStatus(msg, j_msg);
    }
    else if (msg_type == "triorb_static_interface/msg/SettingIPv4") {
      triorb_static_interface::msg::SettingIPv4 msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromSettingIPv4(msg, j_msg);
    }
    else if (msg_type == "triorb_static_interface/msg/SettingROS") {
      triorb_static_interface::msg::SettingROS msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromSettingROS(msg, j_msg);
    }
    else if (msg_type == "triorb_static_interface/msg/SettingSSID") {
      triorb_static_interface::msg::SettingSSID msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromSettingSSID(msg, j_msg);
    }
    else if (msg_type == "triorb_static_interface/msg/StringList") {
      triorb_static_interface::msg::StringList msg;
      deserializeRosMessage(*serialized_msg, msg);
      fromStringList(msg, j_msg);
    }

#endif  // HAVE_TRIORB_INTERFACE
    else {
      found_primitive = false;
    }
    if (found_primitive) {
      primitive = j_msg.dump();
    }
  }

  return found_primitive;
}


  MqttClient::MqttClient(const rclcpp::NodeOptions& options)
    : Node("mqtt_client", options) {
      
     this->sub_add_ros2mqtt_ = create_subscription<mqtt_client_interfaces::msg::Ros2MqttInterface>(
        GET_TOPIC_NAME(std::string("/mqtt_bridg/ros2mqtt/add")), rclcpp::ParametersQoS(),
        std::bind(&MqttClient::callback_add_ros2mqtt, this, std::placeholders::_1));
      this->sub_add_mqtt2ros_ = create_subscription<mqtt_client_interfaces::msg::Mqtt2RosInterface>(
        GET_TOPIC_NAME(std::string("/mqtt_bridg/mqtt2ros/add")), rclcpp::ParametersQoS(),
        std::bind(&MqttClient::callback_add_mqtt2ros, this, std::placeholders::_1));
      this->sub_remove_ros2mqtt_ = create_subscription<std_msgs::msg::String>(
        GET_TOPIC_NAME(std::string("/mqtt_bridg/ros2mqtt/remove")), rclcpp::ParametersQoS(),
        std::bind(&MqttClient::callback_remove_ros2mqtt, this, std::placeholders::_1));
      this->sub_remove_mqtt2ros_ = create_subscription<std_msgs::msg::String>(
        GET_TOPIC_NAME(std::string("/mqtt_bridg/mqtt2ros/remove")), rclcpp::ParametersQoS(),
        std::bind(&MqttClient::callback_remove_mqtt2ros, this, std::placeholders::_1));
      this->pub_edit_dynamic_mapping_result_ = create_publisher<std_msgs::msg::String>(
        GET_TOPIC_NAME(std::string("/mqtt_bridg/edynamic_mapping/edit/result")), rclcpp::ParametersQoS());

#ifdef HAVE_TRIORB_INTERFACE
    this->pub_except_node_registration_ = create_publisher<std_msgs::msg::String>(GET_TOPIC_NAME(std::string("/except_handl/node/add")), rclcpp::ParametersQoS());
    this->pub_except_error_str_add_ = create_publisher<std_msgs::msg::String>(GET_TOPIC_NAME(std::string("/triorb/error/str/add")), rclcpp::ParametersQoS());
    this->pub_except_warn_str_add_ = create_publisher<std_msgs::msg::String>(GET_TOPIC_NAME(std::string("/triorb/warn/str/add")), rclcpp::ParametersQoS());
    std_msgs::msg::String msg;
    msg.data = std::string("[instant]") + this->get_name();
    this->pub_except_node_registration_->publish(msg);
#endif
    loadParameters();
    setup();
  }

  void MqttClient::loadParameters() {

    rcl_interfaces::msg::ParameterDescriptor param_desc;

    param_desc.description =
      "IP address or hostname of the machine running the MQTT broker";
    declare_parameter("broker.host", rclcpp::ParameterType::PARAMETER_STRING,
                      param_desc);
    param_desc.description = "port the MQTT broker is listening on";
    declare_parameter("broker.port", rclcpp::ParameterType::PARAMETER_INTEGER,
                      param_desc);
    param_desc.description =
      "username used for authenticating to the broker (if empty, will try to "
      "connect anonymously)";
    declare_parameter("broker.user", rclcpp::ParameterType::PARAMETER_STRING,
                      param_desc);
    param_desc.description = "password used for authenticating to the broker";
    declare_parameter("broker.pass", rclcpp::ParameterType::PARAMETER_STRING,
                      param_desc);
    param_desc.description = "whether to connect via SSL/TLS";
    declare_parameter("broker.tls.enabled",
                      rclcpp::ParameterType::PARAMETER_BOOL, param_desc);
    param_desc.description =
      "CA certificate file trusted by client (relative to ROS_HOME)";
    declare_parameter("broker.tls.ca_certificate",
                      rclcpp::ParameterType::PARAMETER_STRING, param_desc);

    param_desc.description =
      "unique ID used to identify the client (broker may allow empty ID and "
      "automatically generate one)";
    declare_parameter("client.id", rclcpp::ParameterType::PARAMETER_STRING,
                      param_desc);
    param_desc.description =
      "maximum number of messages buffered by the bridge when not connected to "
      "broker (only available if client ID is not empty)";
    declare_parameter("client.buffer.size",
                      rclcpp::ParameterType::PARAMETER_INTEGER, param_desc);
    param_desc.description =
      "directory used to buffer messages when not connected to broker "
      "(relative to ROS_HOME)";
    declare_parameter("client.buffer.directory",
                      rclcpp::ParameterType::PARAMETER_STRING, param_desc);
    param_desc.description =
      "topic used for this client's last-will message (no last will, if not "
      "specified)";
    declare_parameter("client.last_will.topic",
                      rclcpp::ParameterType::PARAMETER_STRING, param_desc);
    param_desc.description = "last-will message";
    declare_parameter("client.last_will.message",
                      rclcpp::ParameterType::PARAMETER_STRING, param_desc);
    param_desc.description = "QoS value for last-will message";
    declare_parameter("client.last_will.qos",
                      rclcpp::ParameterType::PARAMETER_INTEGER, param_desc);
    param_desc.description = "whether to retain last-will message";
    declare_parameter("client.last_will.retained",
                      rclcpp::ParameterType::PARAMETER_BOOL, param_desc);
    param_desc.description = "whether to use a clean session for this client";
    declare_parameter("client.clean_session",
                      rclcpp::ParameterType::PARAMETER_BOOL, param_desc);
    param_desc.description = "keep-alive interval in seconds";
    declare_parameter("client.keep_alive_interval",
                      rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);
    param_desc.description = "maximum number of inflight messages";
    declare_parameter("client.max_inflight",
                      rclcpp::ParameterType::PARAMETER_INTEGER, param_desc);
    param_desc.description =
      "client certificate file (only needed if broker requires client "
      "certificates; relative to ROS_HOME)";
    declare_parameter("client.tls.certificate",
                      rclcpp::ParameterType::PARAMETER_STRING, param_desc);
    param_desc.description = "client private key file (relative to ROS_HOME)";
    declare_parameter("client.tls.key", rclcpp::ParameterType::PARAMETER_STRING,
                      param_desc);
    param_desc.description = "client private key password";
    declare_parameter("client.tls.password",
                      rclcpp::ParameterType::PARAMETER_STRING, param_desc);

    param_desc.description = "Prefix for MQTT topics";
    declare_parameter("prefix.mqtt", rclcpp::ParameterType::PARAMETER_STRING, param_desc);
    param_desc.description = "Prefix for ROS topics";
    declare_parameter("prefix.ros", rclcpp::ParameterType::PARAMETER_STRING, param_desc);

    param_desc.description = "The list of topics to bridge from ROS to MQTT";
    const auto ros2mqtt_ros_topics =
      declare_parameter<std::vector<std::string>>(
        "bridge.ros2mqtt.ros_topics", std::vector<std::string>(), param_desc);
    for (const auto& ros_topic : ros2mqtt_ros_topics) {
      param_desc.description =
        "MQTT topic on which the corresponding ROS messages are sent to the "
        "broker";
      declare_parameter(fmt::format("bridge.ros2mqtt.{}.mqtt_topic", ros_topic),
                        rclcpp::ParameterType::PARAMETER_STRING, param_desc);
      param_desc.description = "whether to publish as primitive message";
      declare_parameter(fmt::format("bridge.ros2mqtt.{}.primitive", ros_topic),
                        rclcpp::ParameterType::PARAMETER_BOOL, param_desc);
      param_desc.description =
        "If set, the ROS msg type provided will be used. If empty, the type is "
        "automatically deduced via the publisher";
      declare_parameter(fmt::format("bridge.ros2mqtt.{}.ros_type", ros_topic),
                        rclcpp::ParameterType::PARAMETER_STRING, param_desc);
      param_desc.description =
        "whether to attach a timestamp to a ROS2MQTT payload (for latency "
        "computation on receiver side)";
      declare_parameter(
        fmt::format("bridge.ros2mqtt.{}.inject_timestamp", ros_topic),
        rclcpp::ParameterType::PARAMETER_BOOL, param_desc);
      param_desc.description = "ROS subscriber queue size";
      declare_parameter(
        fmt::format("bridge.ros2mqtt.{}.advanced.ros.queue_size", ros_topic),
        rclcpp::ParameterType::PARAMETER_INTEGER, param_desc);
      param_desc.description = "ROS subscriber QoS reliability";
      declare_parameter(
        fmt::format("bridge.ros2mqtt.{}.advanced.ros.qos.reliability",
                    ros_topic),
        rclcpp::ParameterType::PARAMETER_STRING, param_desc);
      param_desc.description = "ROS subscriber QoS durability";
      declare_parameter(
        fmt::format("bridge.ros2mqtt.{}.advanced.ros.qos.durability",
                    ros_topic),
        rclcpp::ParameterType::PARAMETER_STRING, param_desc);
      param_desc.description = "MQTT QoS value";
      declare_parameter(
        fmt::format("bridge.ros2mqtt.{}.advanced.mqtt.qos", ros_topic),
        rclcpp::ParameterType::PARAMETER_INTEGER, param_desc);
      param_desc.description = "whether to retain MQTT message";
      declare_parameter(
        fmt::format("bridge.ros2mqtt.{}.advanced.mqtt.retained", ros_topic),
        rclcpp::ParameterType::PARAMETER_BOOL, param_desc);
    }

    param_desc.description = "The list of topics to bridge from MQTT to ROS";
    const auto mqtt2ros_mqtt_topics =
      declare_parameter<std::vector<std::string>>(
        "bridge.mqtt2ros.mqtt_topics", std::vector<std::string>(), param_desc);
    for (const auto& mqtt_topic : mqtt2ros_mqtt_topics) {
      param_desc.description =
        "ROS topic on which corresponding MQTT messages are published";
      declare_parameter(fmt::format("bridge.mqtt2ros.{}.ros_topic", mqtt_topic),
                        rclcpp::ParameterType::PARAMETER_STRING, param_desc);
      param_desc.description =
        "whether to publish as primitive message (if coming from non-ROS MQTT "
        "client)";
      declare_parameter(fmt::format("bridge.mqtt2ros.{}.primitive", mqtt_topic),
                        rclcpp::ParameterType::PARAMETER_BOOL, param_desc);
      param_desc.description =
        "If set, the ROS msg type provided will be used. If empty, the type is "
        "automatically deduced via the MQTT message";
      declare_parameter(fmt::format("bridge.mqtt2ros.{}.ros_type", mqtt_topic),
                        rclcpp::ParameterType::PARAMETER_STRING, param_desc);
      param_desc.description = "MQTT QoS value";
      declare_parameter(
        fmt::format("bridge.mqtt2ros.{}.advanced.mqtt.qos", mqtt_topic),
        rclcpp::ParameterType::PARAMETER_INTEGER, param_desc);
      param_desc.description = "ROS publisher queue size";
      declare_parameter(
        fmt::format("bridge.mqtt2ros.{}.advanced.ros.queue_size", mqtt_topic),
        rclcpp::ParameterType::PARAMETER_INTEGER, param_desc);
      param_desc.description = "ROS publisher QoS durability";
      declare_parameter(
        fmt::format("bridge.mqtt2ros.{}.advanced.ros.qos.durability",
                    mqtt_topic),
        rclcpp::ParameterType::PARAMETER_STRING, param_desc);
      param_desc.description = "ROS publisher QoS reliability";
      declare_parameter(
        fmt::format("bridge.mqtt2ros.{}.advanced.ros.qos.reliability",
                    mqtt_topic),
        rclcpp::ParameterType::PARAMETER_STRING, param_desc);
      param_desc.description = "whether to latch ROS message";
      declare_parameter(
        fmt::format("bridge.mqtt2ros.{}.advanced.ros.latched", mqtt_topic),
        rclcpp::ParameterType::PARAMETER_BOOL, param_desc);
    }

    // load broker parameters from parameter server
    std::string broker_tls_ca_certificate;
    loadParameter("broker.host", broker_config_.host, "localhost");
    loadParameter("broker.port", broker_config_.port, 1883);
    if (loadParameter("broker.user", broker_config_.user)) {
      loadParameter("broker.pass", broker_config_.pass, "");
    }
    if (loadParameter("broker.tls.enabled", broker_config_.tls.enabled,
                      false)) {
      loadParameter("broker.tls.ca_certificate", broker_tls_ca_certificate,
                    "/etc/ssl/certs/ca-certificates.crt");
    }

    // load client parameters from parameter server
    std::string client_buffer_directory, client_tls_certificate, client_tls_key;
    loadParameter("client.id", client_config_.id, "");
    client_config_.buffer.enabled = !client_config_.id.empty();
    if (client_config_.buffer.enabled) {
      loadParameter("client.buffer.size", client_config_.buffer.size, 0);
      loadParameter("client.buffer.directory", client_buffer_directory,
                    "buffer");
    } else {
      RCLCPP_WARN(get_logger(),
                  "Client buffer can not be enabled when client ID is empty");
    }
    if (loadParameter("client.last_will.topic",
                      client_config_.last_will.topic)) {
      loadParameter("client.last_will.message",
                    client_config_.last_will.message, "offline");
      loadParameter("client.last_will.qos", client_config_.last_will.qos, 0);
      loadParameter("client.last_will.retained",
                    client_config_.last_will.retained, false);
    }
    loadParameter("client.clean_session", client_config_.clean_session, true);
    loadParameter("client.keep_alive_interval",
                  client_config_.keep_alive_interval, 60.0);
    loadParameter("client.max_inflight", client_config_.max_inflight, 65535);
    if (broker_config_.tls.enabled) {
      if (loadParameter("client.tls.certificate", client_tls_certificate)) {
        loadParameter("client.tls.key", client_tls_key);
        loadParameter("client.tls.password", client_config_.tls.password);
        loadParameter("client.tls.version", client_config_.tls.version);
        loadParameter("client.tls.verify", client_config_.tls.verify);
        loadParameter("client.tls.alpn_protos", client_config_.tls.alpn_protos);
      }
    }

    // load prefix parameters from parameter server
    loadParameter("prefix.mqtt", this->topic_prefix_mqtt_, "");
    loadParameter("prefix.ros", this->topic_prefix_ros_, "");
    RCLCPP_INFO(get_logger(), "Using MQTT broker %s:%d",
                broker_config_.host.c_str(), broker_config_.port);
    RCLCPP_INFO(get_logger(), "Prefix for MQTT topics: '%s'",
                this->topic_prefix_mqtt_.c_str());
    RCLCPP_INFO(get_logger(), "Prefix for ROS topics: '%s'",
                this->topic_prefix_ros_.c_str());
                
    // resolve filepaths
    broker_config_.tls.ca_certificate = resolvePath(broker_tls_ca_certificate);
    client_config_.buffer.directory = resolvePath(client_buffer_directory);
    client_config_.tls.certificate = resolvePath(client_tls_certificate);
    client_config_.tls.key = resolvePath(client_tls_key);

    // parse bridge parameters

    // ros2mqtt
    for (const auto& ros_topic_raw : ros2mqtt_ros_topics) {

      rclcpp::Parameter mqtt_topic_param;
      if (get_parameter(fmt::format("bridge.ros2mqtt.{}.mqtt_topic", ros_topic_raw),
                        mqtt_topic_param)) {

        const std::string ros_topic = this->topic_prefix_ros_ + ros_topic_raw;

        // ros2mqtt[k]/ros_topic and ros2mqtt[k]/mqtt_topic
        const std::string mqtt_topic = this->topic_prefix_mqtt_ + mqtt_topic_param.as_string();
        Ros2MqttInterface& ros2mqtt = ros2mqtt_[ros_topic];
        ros2mqtt.mqtt.topic = mqtt_topic;

        // ros2mqtt[k]/primitive
        rclcpp::Parameter primitive_param;
        if (get_parameter(
              fmt::format("bridge.ros2mqtt.{}.primitive", ros_topic_raw),
              primitive_param))
          ros2mqtt.primitive = primitive_param.as_bool();

        // ros2mqtt[k]/ros_type
        rclcpp::Parameter ros_type_param;
        if (get_parameter(fmt::format("bridge.ros2mqtt.{}.ros_type", ros_topic_raw),
                          ros_type_param)) {
          ros2mqtt.ros.msg_type = ros_type_param.as_string();
          ros2mqtt.fixed_type = true;
          RCLCPP_DEBUG(get_logger(), "Using explicit ROS message type '%s'",
                       ros2mqtt.ros.msg_type.c_str());
        }

        // ros2mqtt[k]/inject_timestamp
        rclcpp::Parameter stamped_param;
        if (get_parameter(
              fmt::format("bridge.ros2mqtt.{}.inject_timestamp", ros_topic_raw),
              stamped_param))
          ros2mqtt.stamped = stamped_param.as_bool();
        if (ros2mqtt.stamped && ros2mqtt.primitive) {
          RCLCPP_WARN(
            get_logger(),
            "Timestamp will not be injected into primitive messages on ROS "
            "topic '%s'",
            ros_topic.c_str());
          ros2mqtt.stamped = false;
        }

        // ros2mqtt[k]/advanced/ros/queue_size
        rclcpp::Parameter queue_size_param;
        if (get_parameter(
              fmt::format("bridge.ros2mqtt.{}.advanced.ros.queue_size",
                          ros_topic_raw),
              queue_size_param))
          ros2mqtt.ros.queue_size = queue_size_param.as_int();

        rclcpp::Parameter durability_param;
        if (get_parameter(
              fmt::format("bridge.ros2mqtt.{}.advanced.ros.qos.durability",
                          ros_topic_raw),
              durability_param)) {
          const auto p = durability_param.as_string();
          if (p == "system_default") {
            ros2mqtt.ros.qos.durability =
              rclcpp::DurabilityPolicy::SystemDefault;
          } else if (p == "volatile") {
            ros2mqtt.ros.qos.durability = rclcpp::DurabilityPolicy::Volatile;
          } else if (p == "transient_local") {
            ros2mqtt.ros.qos.durability =
              rclcpp::DurabilityPolicy::TransientLocal;
          } else if (p == "auto") {
            ros2mqtt.ros.qos.durability = {};
          } else {
            RCLCPP_ERROR(get_logger(), "Unexpected durability %s", p.c_str());
#ifdef HAVE_TRIORB_INTERFACE
            std_msgs::msg::String _msg; _msg.data = "mqrr_client / Unexpected durability";
            this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
            exit(EXIT_FAILURE);
          }
        }

        rclcpp::Parameter reliability_param;
        if (get_parameter(
              fmt::format("bridge.ros2mqtt.{}.advanced.ros.qos.reliability",
                          ros_topic_raw),
              reliability_param)) {
          const auto p = reliability_param.as_string();
          if (p == "system_default") {
            ros2mqtt.ros.qos.reliability =
              rclcpp::ReliabilityPolicy::SystemDefault;
          } else if (p == "best_effort") {
            ros2mqtt.ros.qos.reliability =
              rclcpp::ReliabilityPolicy::BestEffort;
          } else if (p == "reliable") {
            ros2mqtt.ros.qos.reliability = rclcpp::ReliabilityPolicy::Reliable;
          } else if (p == "auto") {
            ros2mqtt.ros.qos.reliability = {};
          } else {
            RCLCPP_ERROR(get_logger(), "Unexpected reliability %s", p.c_str());
#ifdef HAVE_TRIORB_INTERFACE
            std_msgs::msg::String _msg; _msg.data = "mqrr_client / Unexpected reliability";
            this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
            exit(EXIT_FAILURE);
          }
        }

        // ros2mqtt[k]/advanced/mqtt/qos
        rclcpp::Parameter qos_param;
        if (get_parameter(
              fmt::format("bridge.ros2mqtt.{}.advanced.mqtt.qos", ros_topic_raw),
              qos_param))
          ros2mqtt.mqtt.qos = qos_param.as_int();

        // ros2mqtt[k]/advanced/mqtt/retained
        rclcpp::Parameter retained_param;
        if (get_parameter(
              fmt::format("bridge.ros2mqtt.{}.advanced.mqtt.retained",
                          ros_topic_raw),
              retained_param))
          ros2mqtt.mqtt.retained = retained_param.as_bool();

        printf(
                    "Bridging %sROS topic '%s' to MQTT topic '%s' %s",
                    ros2mqtt.primitive ? "primitive " : "", ros_topic.c_str(),
                    ros2mqtt.mqtt.topic.c_str(),
                    ros2mqtt.stamped ? "and measuring latency" : "");
      } else {
        RCLCPP_WARN(
          get_logger(),
          fmt::format("Parameter 'bridge.ros2mqtt.{}' is missing subparameter "
                      "'mqtt_topic', will be ignored",
                      ros_topic_raw)
            .c_str());
      }
    }

    // mqtt2ros
    for (const auto& mqtt_topic_raw : mqtt2ros_mqtt_topics) {

      rclcpp::Parameter ros_topic_param;
      if (get_parameter(fmt::format("bridge.mqtt2ros.{}.ros_topic", mqtt_topic_raw),
                        ros_topic_param)) {

        const std::string mqtt_topic = this->topic_prefix_mqtt_ + mqtt_topic_raw;

        // mqtt2ros[k]/mqtt_topic and mqtt2ros[k]/ros_topic
        const std::string ros_topic = this->topic_prefix_ros_ + ros_topic_param.as_string();
        Mqtt2RosInterface& mqtt2ros = mqtt2ros_[mqtt_topic];
        mqtt2ros.ros.topic = ros_topic;

        // mqtt2ros[k]/primitive
        rclcpp::Parameter primitive_param;
        if (get_parameter(
              fmt::format("bridge.mqtt2ros.{}.primitive", mqtt_topic_raw),
              primitive_param))
          mqtt2ros.primitive = primitive_param.as_bool();


        rclcpp::Parameter ros_type_param;
        if (get_parameter(
              fmt::format("bridge.mqtt2ros.{}.ros_type", mqtt_topic_raw),
              ros_type_param)) {
          mqtt2ros.ros.msg_type = ros_type_param.as_string();
          mqtt2ros.fixed_type = true;
          RCLCPP_DEBUG(get_logger(),
                       "Using explicit ROS message type '%s' for '%s'",
                       mqtt2ros.ros.msg_type.c_str(), ros_topic.c_str());
        }

        // mqtt2ros[k]/advanced/mqtt/qos
        rclcpp::Parameter qos_param;
        if (get_parameter(
              fmt::format("bridge.mqtt2ros.{}.advanced.mqtt.qos", mqtt_topic_raw),
              qos_param))
          mqtt2ros.mqtt.qos = qos_param.as_int();

        // mqtt2ros[k]/advanced/ros/queue_size
        rclcpp::Parameter queue_size_param;
        if (get_parameter(
              fmt::format("bridge.mqtt2ros.{}.advanced.ros.queue_size",
                          mqtt_topic_raw),
              queue_size_param))
          mqtt2ros.ros.queue_size = queue_size_param.as_int();

        rclcpp::Parameter durability_param;
        if (get_parameter(
              fmt::format("bridge.mqtt2ros.{}.advanced.ros.qos.durability",
                          mqtt_topic_raw),
              durability_param)) {
          const auto p = durability_param.as_string();
          if (p == "system_default") {
            mqtt2ros.ros.qos.durability =
              rclcpp::DurabilityPolicy::SystemDefault;
          } else if (p == "volatile") {
            mqtt2ros.ros.qos.durability = rclcpp::DurabilityPolicy::Volatile;
          } else if (p == "transient_local") {
            mqtt2ros.ros.qos.durability =
              rclcpp::DurabilityPolicy::TransientLocal;
          } else {
            RCLCPP_ERROR(get_logger(), "Unexpected durability %s", p.c_str());
#ifdef HAVE_TRIORB_INTERFACE
            std_msgs::msg::String _msg; _msg.data = "mqrr_client / Unexpected durability";
            this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
            exit(EXIT_FAILURE);
          }
        }

        rclcpp::Parameter reliability_param;
        if (get_parameter(
              fmt::format("bridge.mqtt2ros.{}.advanced.ros.qos.reliability",
                          mqtt_topic_raw),
              reliability_param)) {
          const auto p = reliability_param.as_string();
          if (p == "system_default") {
            mqtt2ros.ros.qos.reliability =
              rclcpp::ReliabilityPolicy::SystemDefault;
          } else if (p == "best_effort") {
            mqtt2ros.ros.qos.reliability =
              rclcpp::ReliabilityPolicy::BestEffort;
          } else if (p == "reliable") {
            mqtt2ros.ros.qos.reliability = rclcpp::ReliabilityPolicy::Reliable;
          } else {
            RCLCPP_ERROR(get_logger(), "Unexpected reliability %s", p.c_str());
#ifdef HAVE_TRIORB_INTERFACE
            std_msgs::msg::String _msg; _msg.data = "mqrr_client / Unexpected reliability";
            this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
            exit(EXIT_FAILURE);
          }
        }

        // mqtt2ros[k]/advanced/ros/latched
        rclcpp::Parameter latched_param;
        if (get_parameter(fmt::format("bridge.mqtt2ros.{}.advanced.ros.latched",
                                      mqtt_topic_raw),
                          latched_param)) {
          mqtt2ros.ros.latched = latched_param.as_bool();
          RCLCPP_WARN(
            get_logger(),
            fmt::format(
              "Parameter 'bridge.mqtt2ros.{}.advanced.ros.latched' is ignored "
              "since ROS 2 does not easily support latched topics.",
              mqtt_topic_raw)
              .c_str());
        }

        printf(
                    "Bridging MQTT topic '%s' to %sROS topic '%s'",
                    mqtt_topic.c_str(), mqtt2ros.primitive ? "primitive " : "",
                    mqtt2ros.ros.topic.c_str());
      } else {
        RCLCPP_WARN(
          get_logger(),
          fmt::format("Parameter 'bridge.ros2mqtt.{}' is missing subparameter "
                      "'ros_topic', will be ignored",
                      mqtt_topic_raw)
            .c_str());
      }
    }

    if (ros2mqtt_.empty() && mqtt2ros_.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "No valid ROS-MQTT bridge found in parameter 'bridge'");
#ifdef HAVE_TRIORB_INTERFACE
      std_msgs::msg::String _msg; _msg.data = "mqtt_client / No valid ROS-MQTT bridge found in parameter 'bridge'";
      this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
      exit(EXIT_FAILURE);
    }
  }


  bool MqttClient::loadParameter(const std::string& key, std::string& value) {
    bool found = get_parameter(key, value);
    if (found)
      RCLCPP_DEBUG(get_logger(), "Retrieved parameter '%s' = '%s'", key.c_str(),
                   value.c_str());
    return found;
  }


  bool MqttClient::loadParameter(const std::string& key, std::string& value,
                                 const std::string& default_value) {
    bool found = get_parameter_or(key, value, default_value);
    if (!found)
      RCLCPP_WARN(get_logger(), "Parameter '%s' not set, defaulting to '%s'",
                  key.c_str(), default_value.c_str());
    if (found)
      RCLCPP_DEBUG(get_logger(), "Retrieved parameter '%s' = '%s'", key.c_str(),
                   value.c_str());
    return found;
  }


  std::filesystem::path MqttClient::resolvePath(
    const std::string& path_string) {

    std::filesystem::path path(path_string);
    if (path_string.empty()) return path;
    if (!path.has_root_path()) {
      std::string ros_home = rcpputils::get_env_var("ROS_HOME");
      if (ros_home.empty())
        ros_home = std::string(std::filesystem::current_path());
      path = std::filesystem::path(ros_home);
      path.append(path_string);
    }
    if (!std::filesystem::exists(path))
      RCLCPP_WARN(get_logger(), "Requested path '%s' does not exist",
                  std::string(path).c_str());
    return path;
  }


  void MqttClient::setup() {

    // pre-compute timestamp length
    builtin_interfaces::msg::Time tmp_stamp;
    rclcpp::SerializedMessage tmp_serialized_stamp;
    serializeRosMessage(tmp_stamp, tmp_serialized_stamp);
    stamp_length_ = tmp_serialized_stamp.size();

    // initialize MQTT client
    setupClient();

    // connect to MQTT broker
    connect();

    // create ROS service server
    is_connected_service_ =
      create_service<mqtt_client_interfaces::srv::IsConnected>(
        "~/is_connected",
        std::bind(&MqttClient::isConnectedService, this, std::placeholders::_1,
                  std::placeholders::_2));

    // create dynamic mappings services
    new_ros2mqtt_bridge_service_ =
      create_service<mqtt_client_interfaces::srv::NewRos2MqttBridge>(
        "~/new_ros2mqtt_bridge",
        std::bind(&MqttClient::newRos2MqttBridge, this, std::placeholders::_1,
                  std::placeholders::_2));
    new_mqtt2ros_bridge_service_ =
      create_service<mqtt_client_interfaces::srv::NewMqtt2RosBridge>(
        "~/new_mqtt2ros_bridge",
        std::bind(&MqttClient::newMqtt2RosBridge, this, std::placeholders::_1,
                  std::placeholders::_2));

    // setup subscribers; check for new types every second
    check_subscriptions_timer_ =
      create_wall_timer(std::chrono::duration<double>(1.0),
                        std::bind(&MqttClient::setupSubscriptions, this));

    setupPublishers();
  }

  std::optional<rclcpp::QoS> MqttClient::getCompatibleQoS(
    const std::string& ros_topic, const rclcpp::TopicEndpointInfo& tei,
    const Ros2MqttInterface& ros2mqtt) const {
    // the basic premise here is that we take the QoS from the publisher,
    // overwrite any parts that are explicitly set via configuration then check
    // if the result is compatible with the publisher
    auto qos = tei.qos_profile();

    if (auto r = ros2mqtt.ros.qos.reliability) qos.reliability(*r);
    if (auto d = ros2mqtt.ros.qos.durability) qos.durability(*d);
    qos.keep_last(ros2mqtt.ros.queue_size);

    const auto qres = rclcpp::qos_check_compatible(tei.qos_profile(), qos);

    switch (qres.compatibility) {
      case rclcpp::QoSCompatibility::Ok:
        return qos;
      case rclcpp::QoSCompatibility::Warning:
        RCLCPP_DEBUG(get_logger(), "QoS compatibility warning on topic %s - %s",
                     ros_topic.c_str(), qres.reason.c_str());
        // presumably this is still compatible
        return qos;
      case rclcpp::QoSCompatibility::Error:
      default:
        return {};
    }
  }

  std::vector<rclcpp::TopicEndpointInfo> MqttClient::getCandidatePublishers(
    const std::string& ros_topic, const Ros2MqttInterface& ros2mqtt) const {
    const auto& pubs = get_publishers_info_by_topic(ros_topic);

    if (pubs.empty()) return {};

    std::vector<rclcpp::TopicEndpointInfo> ret;

    ret.reserve(pubs.size());

    for (const auto& p : pubs) {
      const std::string msg_type = p.topic_type();

      // if the type isn't set, match aginst all t ypes, otherwise only match
      // against mtching types
      if (ros2mqtt.ros.msg_type.empty() || msg_type == ros2mqtt.ros.msg_type)
        ret.push_back(p);
    }

    // If we found any matching types, return those
    if (!ret.empty()) return ret;

    // If we didn't and we aren't fix type, then just return the full set of
    // publishers
    if (!ros2mqtt.fixed_type) return pubs;

    // None of these publishers will work... :sad_panda:
    return {};
  }

  void MqttClient::setupSubscriptions() {

    // For each ros2mqtt interface, check if we need to do a lazy subscription
    // (eg, lazy subscription would be if we are determining either the type of
    // QoS dynamically). If we don't, then just make the fixed subscriber if its
    // not already made.
    //
    // If we do, we check each publisher for a potential match. The first step
    // is to filter down the list of publishers based on the type. If we are
    // fixed type then that list only includes publishers with matching types.
    // If we aren't its a little trickier. For dynamic types, if we already have
    // matched against a type previously, the candidate list will include all
    // publishers which have matching types if any exist. If none of the
    // publishers have matching types, then the list will include all
    // publishers.
    //
    // Then for each candidate publisher, check if their QoS is compatible with
    // the QoS specic in the configuration. This is really just needed because
    // there is the potential for someone to set half of the QoS to auto and
    // half to explicit. Condition for requiring "lazy" subscription where we
    // need to walk the
    const auto requires_lazy_subscription =
      [](const Ros2MqttInterface& ros2mqtt) {
        if (!ros2mqtt.fixed_type) return true;
        if (ros2mqtt.ros.qos.reliability == std::nullopt ||
            ros2mqtt.ros.qos.durability == std::nullopt)
          return true;
        return false;
      };

    for (auto& [ros_topic, ros2mqtt] : ros2mqtt_) {

      std::function<void(const std::shared_ptr<rclcpp::SerializedMessage> msg)>
        bound_callback_func = std::bind(&MqttClient::ros2mqtt, this,
                                        std::placeholders::_1, ros_topic);

      if (!requires_lazy_subscription(ros2mqtt)) {
        try {
          if (ros2mqtt.ros.subscriber) continue;

          auto const qos = rclcpp::QoS(ros2mqtt.ros.queue_size)
                             .reliability(*ros2mqtt.ros.qos.reliability)
                             .durability(*ros2mqtt.ros.qos.durability);

          ros2mqtt.ros.subscriber = create_generic_subscription(
            ros_topic, ros2mqtt.ros.msg_type, qos, bound_callback_func);
          ros2mqtt.ros.is_stale = false;
          printf( "Subscribed ROS topic '%s' of type '%s'",
                      ros_topic.c_str(), ros2mqtt.ros.msg_type.c_str());
        } catch (rclcpp::exceptions::RCLError& e) {
          RCLCPP_ERROR(get_logger(), "Failed to create generic subscriber: %s",
                       e.what());
#ifdef HAVE_TRIORB_INTERFACE
          std_msgs::msg::String _msg; _msg.data = "mqtt_client / Failed to create generic subscriber";
          this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
          continue;
        }
      } else {
        const auto& pubs = getCandidatePublishers(ros_topic, ros2mqtt);

        if (pubs.empty()) continue;

        for (auto endpointInfo : pubs) {
          try {
            // check if message type has changed or if mapping is stale
            const std::string msg_type = endpointInfo.topic_type();

            if (msg_type == ros2mqtt.ros.msg_type && !ros2mqtt.ros.is_stale &&
                ros2mqtt.ros.subscriber)
              continue;

            auto const qos =
              getCompatibleQoS(ros_topic, endpointInfo, ros2mqtt);

            if (!qos) continue;

            ros2mqtt.ros.is_stale = false;
            ros2mqtt.ros.msg_type = msg_type;

            ros2mqtt.ros.subscriber = create_generic_subscription(
              ros_topic, msg_type, *qos, bound_callback_func);

            printf( "Subscribed ROS topic '%s' of type '%s'",
                        ros_topic.c_str(), msg_type.c_str());
          } catch (rclcpp::exceptions::RCLError& e) {
            RCLCPP_ERROR(get_logger(),
                         "Failed to create generic subscriber: %s", e.what());
#ifdef HAVE_TRIORB_INTERFACE
            std_msgs::msg::String _msg; _msg.data = "mqtt_client / Failed to create generic subscriber";
            this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
            continue;
          }
        }
      }
    }
  }

  void MqttClient::setupPublishers() {

    for (auto& [mqtt_topic, mqtt2ros] : mqtt2ros_) {
      if (mqtt2ros.ros.publisher) continue;

      // If the type is not fixed, we require a mqtt message in order to deduce
      // the type
      if (!mqtt2ros.fixed_type) continue;

      try {
        const auto qos = rclcpp::QoS(mqtt2ros.ros.queue_size)
                           .durability(mqtt2ros.ros.qos.durability)
                           .reliability(mqtt2ros.ros.qos.reliability);
        mqtt2ros.ros.publisher = create_generic_publisher(
          mqtt2ros.ros.topic, mqtt2ros.ros.msg_type, qos);

        mqtt2ros.ros.is_stale = false;
      } catch (const rclcpp::exceptions::RCLError& e) {
        RCLCPP_ERROR(get_logger(), "Failed to create generic publisher: %s",
                     e.what());
#ifdef HAVE_TRIORB_INTERFACE
        std_msgs::msg::String _msg; _msg.data = "mqtt_client / Failed to create generic publisher";
        this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
      }
    }
  }

  void MqttClient::setupClient() {

    // basic client connection options
    connect_options_.set_automatic_reconnect(true);
    connect_options_.set_clean_session(client_config_.clean_session);
    connect_options_.set_keep_alive_interval(
      client_config_.keep_alive_interval);
    connect_options_.set_max_inflight(client_config_.max_inflight);

    // user authentication
    if (!broker_config_.user.empty()) {
      connect_options_.set_user_name(broker_config_.user);
      connect_options_.set_password(broker_config_.pass);
    }

    // last will
    if (!client_config_.last_will.topic.empty()) {
      mqtt::will_options will(
        client_config_.last_will.topic, client_config_.last_will.message,
        client_config_.last_will.qos, client_config_.last_will.retained);
      connect_options_.set_will(will);
    }

    // SSL/TLS
    if (broker_config_.tls.enabled) {
      mqtt::ssl_options ssl;
      ssl.set_trust_store(broker_config_.tls.ca_certificate);
      if (!client_config_.tls.certificate.empty() &&
          !client_config_.tls.key.empty()) {
        ssl.set_key_store(client_config_.tls.certificate);
        ssl.set_private_key(client_config_.tls.key);
        if (!client_config_.tls.password.empty())
          ssl.set_private_key_password(client_config_.tls.password);
      }
      ssl.set_ssl_version(client_config_.tls.version);
      ssl.set_verify(client_config_.tls.verify);
      ssl.set_alpn_protos(client_config_.tls.alpn_protos);
      connect_options_.set_ssl(ssl);
    }

    // create MQTT client
    const std::string protocol = broker_config_.tls.enabled ? "ssl" : "tcp";
    const std::string uri = fmt::format(
      "{}://{}:{}", protocol, broker_config_.host, broker_config_.port);
    try {
      if (client_config_.buffer.enabled) {
        client_ = std::shared_ptr<mqtt::async_client>(new mqtt::async_client(
          uri, client_config_.id, client_config_.buffer.size,
          client_config_.buffer.directory));
      } else {
        client_ = std::shared_ptr<mqtt::async_client>(
          new mqtt::async_client(uri, client_config_.id));
      }
    } catch (const mqtt::exception& e) {
      RCLCPP_ERROR(get_logger(), "Client could not be initialized: %s",
                   e.what());
#ifdef HAVE_TRIORB_INTERFACE
      std_msgs::msg::String _msg; _msg.data = "mqtt_client / Client could not be initialized";
      this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
      exit(EXIT_FAILURE);
    }

    // setup MQTT callbacks
    client_->set_callback(*this);
  }


  void MqttClient::connect() {

    std::string as_client =
      client_config_.id.empty()
        ? ""
        : std::string(" as '") + client_config_.id + std::string("'");
    printf( "Connecting to broker at '%s'%s ...",
                client_->get_server_uri().c_str(), as_client.c_str());

    try {
      client_->connect(connect_options_, nullptr, *this);
    } catch (const mqtt::exception& e) {
      RCLCPP_ERROR(get_logger(), "Connection to broker failed: %s", e.what());
#ifdef HAVE_TRIORB_INTERFACE
      std_msgs::msg::String _msg; _msg.data = "mqtt_client / Connection to broker failed";
      this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
      exit(EXIT_FAILURE);
    }
  }


  void MqttClient::ros2mqtt(
    const std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg,
    const std::string& ros_topic) {

    Ros2MqttInterface& ros2mqtt = ros2mqtt_[ros_topic];
    std::string mqtt_topic = ros2mqtt.mqtt.topic;
    std::vector<uint8_t> payload_buffer;

    // gather information on ROS message type in special ROS message
    mqtt_client_interfaces::msg::RosMsgType ros_msg_type;
    ros_msg_type.name = ros2mqtt.ros.msg_type;
    ros_msg_type.stamped = ros2mqtt.stamped;

    RCLCPP_DEBUG(get_logger(),
                 "Received ROS message of type '%s' on topic '%s'",
                 ros_msg_type.name.c_str(), ros_topic.c_str());

    if (ros2mqtt.primitive) {  // publish as primitive (string) message

      // resolve ROS messages to primitive strings if possible
      std::string payload;
      bool found_primitive =
        primitiveRosMessageToString(serialized_msg, ros_msg_type.name, payload);
      if (found_primitive) {
        payload_buffer = std::vector<uint8_t>(payload.begin(), payload.end());
      } else {
        RCLCPP_WARN(
          get_logger(),
          "1. Cannot send ROS message of type '%s' as primitive message, "
          "check supported primitive types",
          ros_msg_type.name.c_str());
        return;
      }

    } else {  // publish as complete ROS message incl. ROS message type

      // serialize ROS message type
      rclcpp::SerializedMessage serialized_ros_msg_type;
      serializeRosMessage(ros_msg_type, serialized_ros_msg_type);
      uint32_t msg_type_length = serialized_ros_msg_type.size();
      std::vector<uint8_t> msg_type_buffer = std::vector<uint8_t>(
        serialized_ros_msg_type.get_rcl_serialized_message().buffer,
        serialized_ros_msg_type.get_rcl_serialized_message().buffer +
          msg_type_length);

      // send ROS message type information to MQTT broker
      mqtt_topic = kRosMsgTypeMqttTopicPrefix + ros2mqtt.mqtt.topic;
      try {
        RCLCPP_DEBUG(
          get_logger(),
          "Sending ROS message type to MQTT broker on topic '%s' ...",
          mqtt_topic.c_str());
        mqtt::message_ptr mqtt_msg =
          mqtt::make_message(mqtt_topic, msg_type_buffer.data(),
                             msg_type_buffer.size(), ros2mqtt.mqtt.qos, true);
        client_->publish(mqtt_msg);
      } catch (const mqtt::exception& e) {
        RCLCPP_WARN(get_logger(),
                    "Publishing ROS message type information to MQTT topic "
                    "'%s' failed: %s",
                    mqtt_topic.c_str(), e.what());
      }

      // build MQTT payload for ROS message (R) as [R]
      // or [S, R] if timestamp (S) is included
      uint32_t msg_length = serialized_msg->size();
      uint32_t payload_length = msg_length;
      uint32_t msg_offset = 0;
      if (ros2mqtt.stamped) {

        // allocate buffer with appropriate size to hold [S, R]
        msg_offset += stamp_length_;
        payload_length += stamp_length_;
        payload_buffer.resize(payload_length);

        // copy serialized ROS message to payload [-, R]
        std::copy(
          serialized_msg->get_rcl_serialized_message().buffer,
          serialized_msg->get_rcl_serialized_message().buffer + msg_length,
          payload_buffer.begin() + msg_offset);
      } else {

        // directly build payload buffer on top of serialized message
        payload_buffer = std::vector<uint8_t>(
          serialized_msg->get_rcl_serialized_message().buffer,
          serialized_msg->get_rcl_serialized_message().buffer + msg_length);
      }

      // inject timestamp as final step
      if (ros2mqtt.stamped) {

        // take current timestamp
        builtin_interfaces::msg::Time stamp(
          rclcpp::Clock(RCL_SYSTEM_TIME).now());

        // copy serialized timestamp to payload [S, R]
        rclcpp::SerializedMessage serialized_stamp;
        serializeRosMessage(stamp, serialized_stamp);
        std::copy(
          serialized_stamp.get_rcl_serialized_message().buffer,
          serialized_stamp.get_rcl_serialized_message().buffer + stamp_length_,
          payload_buffer.begin());
      }
    }

    // send ROS message to MQTT broker
    mqtt_topic = ros2mqtt.mqtt.topic;
    try {
      // ループバック防止ガード
      {
        std::lock_guard<std::mutex> lock(recent_ros_msgs_mutex_);
        auto it = recent_ros_msgs_.find(mqtt_topic);
        std::string new_payload(payload_buffer.begin(), payload_buffer.end());
        auto now = std::chrono::steady_clock::now();
        if (it != recent_ros_msgs_.end()) {
          const auto& [last_payload, timestamp] = it->second;
          auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - timestamp);
          if (elapsed.count() <= MQTT_CLIENT_LOOPBACK_PROTECTION_MS) {
            if (last_payload == new_payload) {
                //RCLCPP_WARN(get_logger(), "Did not publish to '%s' to prevent loopback: consecutive identical messages on the same topic.", mqtt_topic.c_str());
              return;
            }
          }
          it->second = {new_payload, now};
        }else{
          recent_ros_msgs_[mqtt_topic] = {new_payload, now};
        }
      }

      RCLCPP_DEBUG(
        get_logger(),
        "Sending ROS message of type '%s' to MQTT broker on topic '%s' ...",
        ros_msg_type.name.c_str(), mqtt_topic.c_str());
      mqtt::message_ptr mqtt_msg = mqtt::make_message(
        mqtt_topic, payload_buffer.data(), payload_buffer.size(),
        ros2mqtt.mqtt.qos, ros2mqtt.mqtt.retained);
      client_->publish(mqtt_msg);
    } catch (const mqtt::exception& e) {
      RCLCPP_WARN(
        get_logger(),
        "Publishing ROS message type information to MQTT topic '%s' failed: %s",
        mqtt_topic.c_str(), e.what());
    }
  }


  void MqttClient::mqtt2ros(mqtt::const_message_ptr mqtt_msg,
                            const rclcpp::Time& arrival_stamp) {
    std::string mqtt_topic = mqtt_msg->get_topic();
    Mqtt2RosInterface& mqtt2ros = mqtt2ros_[mqtt_topic];
    auto& payload = mqtt_msg->get_payload_ref();
    uint32_t payload_length = static_cast<uint32_t>(payload.size());

    // read MQTT payload for ROS message (R) as [R]
    // or [S, R] if timestamp (S) is included
    uint32_t msg_length = payload_length;
    uint32_t msg_offset = 0;

    // if stamped, compute latency
    if (mqtt2ros.stamped) {

      // copy stamp to generic message buffer
      rclcpp::SerializedMessage serialized_stamp(stamp_length_);
      std::memcpy(serialized_stamp.get_rcl_serialized_message().buffer,
                  &(payload[msg_offset]), stamp_length_);
      serialized_stamp.get_rcl_serialized_message().buffer_length =
        stamp_length_;

      // deserialize stamp
      builtin_interfaces::msg::Time stamp;
      deserializeRosMessage(serialized_stamp, stamp);

      // compute ROS2MQTT latency
      rclcpp::Duration latency = arrival_stamp - stamp;
      std_msgs::msg::Float64 latency_msg;
      latency_msg.data = latency.seconds();

      // publish latency
      if (!mqtt2ros.ros.latency_publisher) {
        std::string latency_topic = kLatencyRosTopicPrefix + mqtt2ros.ros.topic;
        latency_topic.replace(latency_topic.find("//"), 2, "/");
        mqtt2ros.ros.latency_publisher =
          create_publisher<std_msgs::msg::Float64>(latency_topic, 1);
      }
      mqtt2ros.ros.latency_publisher->publish(latency_msg);

      msg_length -= stamp_length_;
      msg_offset += stamp_length_;
    }

    // copy ROS message from MQTT message to generic message buffer
    rclcpp::SerializedMessage serialized_msg(msg_length);
    std::memcpy(serialized_msg.get_rcl_serialized_message().buffer,
                &(payload[msg_offset]), msg_length);
    serialized_msg.get_rcl_serialized_message().buffer_length = msg_length;

    // publish generic ROS message
    RCLCPP_DEBUG(
      get_logger(),
      "Sending ROS message of type '%s' from MQTT broker to ROS topic '%s' ...",
      mqtt2ros.ros.msg_type.c_str(), mqtt2ros.ros.topic.c_str());
    mqtt2ros.ros.publisher->publish(serialized_msg);
  }


  void MqttClient::mqtt2primitive(mqtt::const_message_ptr mqtt_msg) {

    std::string mqtt_topic = mqtt_msg->get_topic();
    Mqtt2RosInterface& mqtt2ros = mqtt2ros_[mqtt_topic];

    const std::string str_msg = mqtt_msg->to_string();
    bool found_primitive = false;
    std::string ros_msg_type = "std_msgs/msg/String";
    rclcpp::SerializedMessage serialized_msg;

    // check for bool
    if (!found_primitive) {
      std::string bool_str = str_msg;
      std::transform(str_msg.cbegin(), str_msg.cend(), bool_str.begin(),
                     ::tolower);
      if (bool_str == "true" || bool_str == "false") {

        bool bool_msg = (bool_str == "true");

        // construct and serialize ROS message
        std_msgs::msg::Bool msg;
        msg.data = bool_msg;
        serializeRosMessage(msg, serialized_msg);

        ros_msg_type = "std_msgs/msg/Bool";
        found_primitive = true;
      }
    }

    // check for int
    if (!found_primitive) {
      std::size_t pos;
      try {
        const int int_msg = std::stoi(str_msg, &pos);
        if (pos == str_msg.size()) {

          // construct and serialize ROS message
          std_msgs::msg::Int32 msg;
          msg.data = int_msg;
          serializeRosMessage(msg, serialized_msg);

          ros_msg_type = "std_msgs/msg/Int32";
          found_primitive = true;
        }
      } catch (const std::invalid_argument& ex) {
      } catch (const std::out_of_range& ex) {
      }
    }

    // check for float
    if (!found_primitive) {
      std::size_t pos;
      try {
        const float float_msg = std::stof(str_msg, &pos);
        if (pos == str_msg.size()) {

          // construct and serialize ROS message
          std_msgs::msg::Float32 msg;
          msg.data = float_msg;
          serializeRosMessage(msg, serialized_msg);

          ros_msg_type = "std_msgs/msg/Float32";
          found_primitive = true;
        }
      } catch (const std::invalid_argument& ex) {
      } catch (const std::out_of_range& ex) {
      }
    }

    // fall back to string
    if (!found_primitive) {

      // construct and serialize ROS message
      std_msgs::msg::String msg;
      msg.data = str_msg;
      serializeRosMessage(msg, serialized_msg);
    }

    // if ROS message type has changed or if mapping is stale
    if (ros_msg_type != mqtt2ros.ros.msg_type || mqtt2ros.ros.is_stale) {

      mqtt2ros.ros.msg_type = ros_msg_type;
      printf(
                  "ROS publisher message type on topic '%s' set to '%s'",
                  mqtt2ros.ros.topic.c_str(), ros_msg_type.c_str());

      // recreate generic publisher
      try {
        const auto qos = rclcpp::QoS(mqtt2ros.ros.queue_size)
                           .durability(mqtt2ros.ros.qos.durability)
                           .reliability(mqtt2ros.ros.qos.reliability);
        mqtt2ros.ros.publisher =
          create_generic_publisher(mqtt2ros.ros.topic, ros_msg_type, qos);
      } catch (rclcpp::exceptions::RCLError& e) {
        RCLCPP_ERROR(get_logger(), "Failed to create generic publisher: %s",
                     e.what());
#ifdef HAVE_TRIORB_INTERFACE
        std_msgs::msg::String _msg; _msg.data = "mqtt_client / Failed to create generic publisher";
        this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
        return;
      }
      mqtt2ros.ros.is_stale = false;
    }

    // publish
    RCLCPP_DEBUG(
      get_logger(),
      "Sending ROS message of type '%s' from MQTT broker to ROS topic '%s' ...",
      mqtt2ros.ros.msg_type.c_str(), mqtt2ros.ros.topic.c_str());
    mqtt2ros.ros.publisher->publish(serialized_msg);
  }

  void MqttClient::mqtt2fixed(mqtt::const_message_ptr mqtt_msg) {

    std::string mqtt_topic = mqtt_msg->get_topic();
    Mqtt2RosInterface& mqtt2ros = mqtt2ros_[mqtt_topic];

    rclcpp::SerializedMessage serialized_msg;

    if (!fixedMqtt2PrimitiveRos(mqtt_msg, mqtt2ros.ros.msg_type,
                                serialized_msg)) {
      RCLCPP_WARN(get_logger(),
                  "Could not convert mqtt message into type %s on topic %s ...",
                  mqtt2ros.ros.msg_type.c_str(), mqtt2ros.ros.topic.c_str());
    } else {

      if (!mqtt2ros.ros.publisher) {
        printf(
                    "ROS publisher message type on topic '%s' set to '%s'",
                    mqtt2ros.ros.topic.c_str(), mqtt2ros.ros.msg_type.c_str());

        // recreate generic publisher
        try {
          const auto qos = rclcpp::QoS(mqtt2ros.ros.queue_size)
                             .durability(mqtt2ros.ros.qos.durability)
                             .reliability(mqtt2ros.ros.qos.reliability);
          mqtt2ros.ros.publisher = create_generic_publisher(
            mqtt2ros.ros.topic, mqtt2ros.ros.msg_type, qos);

          mqtt2ros.ros.is_stale = false;
        } catch (const rclcpp::exceptions::RCLError& e) {
          RCLCPP_ERROR(get_logger(), "Failed to create generic publisher: %s",
                       e.what());
#ifdef HAVE_TRIORB_INTERFACE
          std_msgs::msg::String _msg; _msg.data = "mqtt_client / Failed to create generic publisher";
          this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
          return;
        }
      }

      RCLCPP_DEBUG(get_logger(),
                   "Sending ROS message of type '%s' from MQTT broker to ROS "
                   "topic '%s' ...",
                   mqtt2ros.ros.msg_type.c_str(), mqtt2ros.ros.topic.c_str());
      mqtt2ros.ros.publisher->publish(serialized_msg);
    }
  }


  void MqttClient::connected(const std::string& cause) {

    (void)cause;  // Avoid compiler warning for unused parameter.

    is_connected_ = true;
    std::string as_client =
      client_config_.id.empty()
        ? ""
        : std::string(" as '") + client_config_.id + std::string("'");
    printf( "Connected to broker at '%s'%s",
                client_->get_server_uri().c_str(), as_client.c_str());

    // subscribe MQTT topics
    for (const auto& [mqtt_topic, mqtt2ros] : mqtt2ros_) {
      if (!mqtt2ros.primitive) {
        std::string const mqtt_topic_to_subscribe =
          kRosMsgTypeMqttTopicPrefix + mqtt_topic;
        client_->subscribe(mqtt_topic_to_subscribe, mqtt2ros.mqtt.qos);
        printf( "Subscribed MQTT topic '%s'",
                    mqtt_topic_to_subscribe.c_str());
      }
      // If not primitive and not fixed, we need the message type before we can
      // public. In that case wait for the type to come in before subscribing to
      // the data topic
      if (mqtt2ros.primitive || mqtt2ros.fixed_type) {
        client_->subscribe(mqtt_topic, mqtt2ros.mqtt.qos);
        printf( "Subscribed MQTT topic '%s'",
                    mqtt_topic.c_str());
      }
    }
  }


  void MqttClient::connection_lost(const std::string& cause) {

    (void)cause;  // Avoid compiler warning for unused parameter.

    RCLCPP_ERROR(get_logger(),
                 "Connection to broker lost, will try to reconnect...");
#ifdef HAVE_TRIORB_INTERFACE
    std_msgs::msg::String _msg; _msg.data = "mqtt_client / Connection to broker lost";
    this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
    is_connected_ = false;
    connect();
  }


  bool MqttClient::isConnected() {

    return is_connected_;
  }


  void MqttClient::isConnectedService(
    mqtt_client_interfaces::srv::IsConnected::Request::SharedPtr request,
    mqtt_client_interfaces::srv::IsConnected::Response::SharedPtr response) {

    (void)request;  // Avoid compiler warning for unused parameter.
    response->connected = isConnected();
  }

  void MqttClient::newRos2MqttBridge(
    mqtt_client_interfaces::srv::NewRos2MqttBridge::Request::SharedPtr request,
    mqtt_client_interfaces::srv::NewRos2MqttBridge::Response::SharedPtr
      response) {

    // add mapping definition to ros2mqtt_
    Ros2MqttInterface& ros2mqtt = ros2mqtt_[request->ros_topic];
    ros2mqtt.ros.is_stale = true;
    ros2mqtt.mqtt.topic = request->mqtt_topic;
    ros2mqtt.primitive = request->primitive;
    ros2mqtt.stamped = request->inject_timestamp;
    ros2mqtt.ros.queue_size = request->ros_queue_size;
    ros2mqtt.mqtt.qos = request->mqtt_qos;
    ros2mqtt.mqtt.retained = request->mqtt_retained;

    if (ros2mqtt.stamped && ros2mqtt.primitive) {
      RCLCPP_WARN(
        get_logger(),
        "Timestamp will not be injected into primitive messages on ROS "
        "topic '%s'",
        request->ros_topic.c_str());
      ros2mqtt.stamped = false;
    }

    printf( "Bridging %sROS topic '%s' to MQTT topic '%s' %s",
                ros2mqtt.primitive ? "primitive " : "",
                request->ros_topic.c_str(), ros2mqtt.mqtt.topic.c_str(),
                ros2mqtt.stamped ? "and measuring latency" : "");

    // (re-)setup ROS subscriptions
    setupSubscriptions();

    response->success = true;
  }

  void MqttClient::newMqtt2RosBridge(
    mqtt_client_interfaces::srv::NewMqtt2RosBridge::Request::SharedPtr request,
    mqtt_client_interfaces::srv::NewMqtt2RosBridge::Response::SharedPtr
      response) {

    // add mapping definition to mqtt2ros_
    Mqtt2RosInterface& mqtt2ros = mqtt2ros_[request->mqtt_topic];
    mqtt2ros.ros.is_stale = true;
    mqtt2ros.ros.topic = request->ros_topic;
    mqtt2ros.primitive = request->primitive;
    mqtt2ros.mqtt.qos = request->mqtt_qos;
    mqtt2ros.ros.queue_size = request->ros_queue_size;
    mqtt2ros.ros.latched = request->ros_latched;
    if (mqtt2ros.ros.latched) {
      RCLCPP_WARN(
        get_logger(),
        fmt::format(
          "Parameter 'bridge.mqtt2ros.{}.advanced.ros.latched' is ignored "
          "since ROS 2 does not easily support latched topics.",
          request->mqtt_topic)
          .c_str());
    }

    printf( "Bridging MQTT topic '%s' to %sROS topic '%s'",
                request->mqtt_topic.c_str(),
                mqtt2ros.primitive ? "primitive " : "",
                mqtt2ros.ros.topic.c_str());

    // subscribe to the MQTT topic
    std::string mqtt_topic_to_subscribe = request->mqtt_topic;
    if (!mqtt2ros.primitive)
      mqtt_topic_to_subscribe =
        kRosMsgTypeMqttTopicPrefix + request->mqtt_topic;
    client_->subscribe(mqtt_topic_to_subscribe, mqtt2ros.mqtt.qos);
    printf( "Subscribed MQTT topic '%s'",
                mqtt_topic_to_subscribe.c_str());

    response->success = true;
  }

  rclcpp::DurabilityPolicy convertStr2DurabilityPolicy(const std::string& durability_str) {
    if (durability_str == "system_default") {
      return rclcpp::DurabilityPolicy::SystemDefault;
    } else if (durability_str == "volatile") {
      return rclcpp::DurabilityPolicy::Volatile;
    } else if (durability_str == "transient_local") {
      return rclcpp::DurabilityPolicy::TransientLocal;
    } else if (durability_str == "auto") {
      return {};
    } else {
      throw std::runtime_error("Unexpected reliability " + durability_str);
    }
  }

  rclcpp::ReliabilityPolicy convertStr2ReliabilityPolicy(const std::string& reliability_str) {
    if (reliability_str == "system_default") {
      return rclcpp::ReliabilityPolicy::SystemDefault;
    } else if (reliability_str == "best_effort") {
      return rclcpp::ReliabilityPolicy::BestEffort;
    } else if (reliability_str == "reliable") {
      return rclcpp::ReliabilityPolicy::Reliable;
    } else if (reliability_str == "auto") {
      return {};
    } else {
      throw std::runtime_error("Unexpected reliability " + reliability_str);
    }
  }
  
  
  /**
   * @brief ROS callback that dynamically creates an MQTT -> ROS mapping.
   */
  void MqttClient::callback_add_ros2mqtt(const mqtt_client_interfaces::msg::Ros2MqttInterface::SharedPtr msg) {
    printf( "Add subscribed ROS topic '%s' of type '%s'",
                msg->ros_topic.c_str(), msg->ros_msg_type.c_str());
    std_msgs::msg::String _result_msg;
    _result_msg.data = "add:ros2mqtt:"+msg->ros_topic + ":" + msg->mqtt_topic;
    try {
      if (ros2mqtt_.count(msg->ros_topic)==0){
          Ros2MqttInterface ros2mqtt;
          ros2mqtt.ros.is_stale = true;
          ros2mqtt.ros.queue_size = msg->ros_queue_size;
          ros2mqtt.ros.msg_type = msg->ros_msg_type;
          ros2mqtt.mqtt.topic = msg->mqtt_topic;
          ros2mqtt.mqtt.qos = msg->mqtt_qos;
          ros2mqtt.mqtt.retained = msg->mqtt_retained;
          ros2mqtt.primitive = msg->primitive;
          ros2mqtt.stamped = msg->stamped;
          ros2mqtt.ros.qos.durability = convertStr2DurabilityPolicy(msg->ros_qos_durability);
          ros2mqtt.ros.qos.reliability = convertStr2ReliabilityPolicy(msg->ros_qos_reliability);
          ros2mqtt_[msg->ros_topic] = ros2mqtt;
          setupSubscriptions();
          _result_msg.data += ":success";
      }
      else {
        _result_msg.data += ":exists";
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to add ros2mqtt bridge: %s", e.what());
      _result_msg.data += ":failure:" + std::string(e.what());
    }
    this->pub_edit_dynamic_mapping_result_->publish(_result_msg);
  }

  /**
   * @brief ROS callback that dynamically creates a ROS -> MQTT mapping.
   */
  void MqttClient::callback_add_mqtt2ros(const mqtt_client_interfaces::msg::Mqtt2RosInterface::SharedPtr msg) {
    printf( "Add subscribed MQTT topic '%s' of type '%s'",
                msg->mqtt_topic.c_str(), msg->ros_msg_type.c_str());
    
    std_msgs::msg::String _result_msg;
    _result_msg.data = "add:mqtt2ros:" + msg->mqtt_topic + ":" + msg->ros_topic;
    try {
      if (mqtt2ros_.count(msg->mqtt_topic)==0){
        Mqtt2RosInterface mqtt2ros;
        mqtt2ros.ros.is_stale = true;
        mqtt2ros.ros.topic = msg->ros_topic;
        mqtt2ros.ros.msg_type = msg->ros_msg_type;
        mqtt2ros.ros.queue_size = msg->ros_queue_size;
        mqtt2ros.ros.latched = msg->ros_latched;
        mqtt2ros.mqtt.qos = msg->mqtt_qos;
        mqtt2ros.fixed_type = msg->fixed_type;
        mqtt2ros.primitive = msg->primitive;
        mqtt2ros.stamped = msg->stamped;
        mqtt2ros.ros.qos.durability = convertStr2DurabilityPolicy(msg->ros_qos_durability);
        mqtt2ros.ros.qos.reliability = convertStr2ReliabilityPolicy(msg->ros_qos_reliability);
        mqtt2ros_[msg->mqtt_topic] = mqtt2ros;
        
        if (!mqtt2ros.primitive) {
          std::string const mqtt_topic_to_subscribe =
            kRosMsgTypeMqttTopicPrefix + msg->mqtt_topic;
          client_->subscribe(mqtt_topic_to_subscribe, mqtt2ros.mqtt.qos);
          printf( "Subscribed MQTT topic '%s'",
                      mqtt_topic_to_subscribe.c_str());
        }
        // If not primitive and not fixed, we need the message type before we can
        // public. In that case wait for the type to come in before subscribing to
        // the data topic
        if (mqtt2ros.primitive || mqtt2ros.fixed_type) {
          client_->subscribe(msg->mqtt_topic, mqtt2ros.mqtt.qos);
          printf( "Subscribed MQTT topic '%s'",
                      msg->mqtt_topic.c_str());
        }

        setupPublishers();
        _result_msg.data += ":success";
      }
      else {
        _result_msg.data += ":exists";
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to add mqtt2ros bridge: %s", e.what());
      _result_msg.data += ":failure:" + std::string(e.what());
    }
    this->pub_edit_dynamic_mapping_result_->publish(_result_msg);
  }

  /**
   * @brief ROS callback that removes a ROS -> MQTT mapping.
   */
  void MqttClient::callback_remove_ros2mqtt(const std_msgs::msg::String::SharedPtr msg) {
    printf( "Received remove ros2mqtt bridge request for topic '%s'",
                msg->data.c_str());
    std_msgs::msg::String _result_msg;
    this->pub_edit_dynamic_mapping_result_->publish(_result_msg);
  }

  /**
   * @brief ROS callback that removes a MQTT -> ROS mapping.
   */
  void MqttClient::callback_remove_mqtt2ros(const std_msgs::msg::String::SharedPtr msg) {
    printf( "Received remove mqtt2ros bridge request for topic '%s'",
                msg->data.c_str());
    std_msgs::msg::String _result_msg;
    this->pub_edit_dynamic_mapping_result_->publish(_result_msg);
  }

  void MqttClient::message_arrived(mqtt::const_message_ptr mqtt_msg) {

    // instantly take arrival timestamp
    rclcpp::Time arrival_stamp(
      builtin_interfaces::msg::Time(rclcpp::Clock(RCL_SYSTEM_TIME).now()));

    std::string mqtt_topic = mqtt_msg->get_topic();
    RCLCPP_DEBUG(get_logger(), "Received MQTT message on topic '%s'",
                 mqtt_topic.c_str());

    // publish directly if primitive
    if (mqtt2ros_.count(mqtt_topic) > 0) {
      Mqtt2RosInterface& mqtt2ros = mqtt2ros_[mqtt_topic];

      if (mqtt2ros.primitive) {
        if (mqtt2ros.fixed_type) {
          mqtt2fixed(mqtt_msg);
        } else {
          mqtt2primitive(mqtt_msg);
        }
        return;
      }
    }

    // else first check for ROS message type
    bool msg_contains_ros_msg_type =
      mqtt_topic.find(kRosMsgTypeMqttTopicPrefix) != std::string::npos;
    if (msg_contains_ros_msg_type) {

      // copy message type to generic message buffer
      auto& payload = mqtt_msg->get_payload_ref();
      uint32_t payload_length = static_cast<uint32_t>(payload.size());
      rclcpp::SerializedMessage serialized_ros_msg_type(payload_length);
      std::memcpy(serialized_ros_msg_type.get_rcl_serialized_message().buffer,
                  &(payload[0]), payload_length);
      serialized_ros_msg_type.get_rcl_serialized_message().buffer_length =
        payload_length;

      // deserialize ROS message type
      mqtt_client_interfaces::msg::RosMsgType ros_msg_type;
      deserializeRosMessage(serialized_ros_msg_type, ros_msg_type);

      // reconstruct corresponding MQTT data topic
      std::string mqtt_data_topic = mqtt_topic;
      mqtt_data_topic.erase(mqtt_data_topic.find(kRosMsgTypeMqttTopicPrefix),
                            kRosMsgTypeMqttTopicPrefix.length());
      Mqtt2RosInterface& mqtt2ros = mqtt2ros_[mqtt_data_topic];

      // if ROS message type has changed or if mapping is stale
      if (ros_msg_type.name != mqtt2ros.ros.msg_type || mqtt2ros.ros.is_stale) {

        if (mqtt2ros.fixed_type) {
          // We should never be in this situation if the type has been set
          // explicitly. As fixed_type is not currently supported through the
          // service based bridge creation and the type name not matching means
          // the fixed type specified in the configuration does not match the
          // one we just recieved
          if (ros_msg_type.name != mqtt2ros.ros.msg_type){
            RCLCPP_ERROR(get_logger(),
                         "Unexpected type name received for topic %s (expected "
                         "%s but received %s)",
                         mqtt2ros.ros.topic.c_str(),
                         mqtt2ros.ros.msg_type.c_str(),
                         ros_msg_type.name.c_str());
#ifdef HAVE_TRIORB_INTERFACE
            std_msgs::msg::String _msg; _msg.data = "mqtt_client / Unexpected type name received for topic";
            this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
          }
          if (mqtt2ros.ros.is_stale){
            RCLCPP_ERROR(get_logger(),
                         "Topic %s has been unexpectedly marked stale",
                         mqtt2ros.ros.topic.c_str());
#ifdef HAVE_TRIORB_INTERFACE
            std_msgs::msg::String _msg; _msg.data = "mqtt_client / Topic has been unexpectedly marked stale";
            this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
          }
          return;
        }

        mqtt2ros.ros.msg_type = ros_msg_type.name;
        mqtt2ros.stamped = ros_msg_type.stamped;
        printf(
                    "ROS publisher message type on topic '%s' set to '%s'",
                    mqtt2ros.ros.topic.c_str(), ros_msg_type.name.c_str());

        // recreate generic publisher
        try {
          const auto qos = rclcpp::QoS(mqtt2ros.ros.queue_size)
                             .durability(mqtt2ros.ros.qos.durability)
                             .reliability(mqtt2ros.ros.qos.reliability);
          mqtt2ros.ros.publisher = create_generic_publisher(
            mqtt2ros.ros.topic, ros_msg_type.name, qos);
        } catch (rclcpp::exceptions::RCLError& e) {
          RCLCPP_ERROR(get_logger(), "Failed to create generic publisher: %s",
                       e.what());
#ifdef HAVE_TRIORB_INTERFACE
          std_msgs::msg::String _msg; _msg.data = "mqtt_client / Failed to create generic publisher";
          this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE
          return;
        }
        mqtt2ros.ros.is_stale = false;

        // subscribe to MQTT topic with actual ROS messages
        client_->subscribe(mqtt_data_topic, mqtt2ros.mqtt.qos);
        RCLCPP_DEBUG(get_logger(), "Subscribed MQTT topic '%s'",
                     mqtt_data_topic.c_str());
      }

    } else {

      // publish ROS message, if publisher initialized
      if (!mqtt2ros_[mqtt_topic].ros.msg_type.empty()) {
        mqtt2ros(mqtt_msg, arrival_stamp);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "ROS publisher for data from MQTT topic '%s' is not yet initialized: "
          "ROS message type not yet known",
          mqtt_topic.c_str());
      }
    }
  }


  void MqttClient::delivery_complete(mqtt::delivery_token_ptr token) {

    (void)token;  // Avoid compiler warning for unused parameter.
  }


  void MqttClient::on_success(const mqtt::token& token) {

    (void)token;  // Avoid compiler warning for unused parameter.
    is_connected_ = true;
  }


  void MqttClient::on_failure(const mqtt::token& token) {

    RCLCPP_ERROR(
      get_logger(),
      "Connection to broker failed (return code %d), will automatically "
      "retry...",
      token.get_return_code());
#ifdef HAVE_TRIORB_INTERFACE
    std_msgs::msg::String _msg; _msg.data = "mqtt_client / Connection to broker failed, will automatically retry...";
    this->pub_except_error_str_add_->publish(_msg);
#endif // HAVE_TRIORB_INTERFACE

    is_connected_ = false;
  }
  
}  // namespace mqtt_client
