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
      j_msg["frame_id"] = msg.frame_id;
      j_msg["stamp"]["sec"] = msg.stamp.sec;
      j_msg["stamp"]["nanosec"] = msg.stamp.nanosec;
    } else if (msg_type == "std_msgs/msg/Int8MultiArray") {
      // layout非対応のため1回だけ警告を出す
      RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mqtt_client"),
        "std_msgs/msg/Int8MultiArray layout is not supported");
      std_msgs::msg::Int8MultiArray msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["data"] = json::array();
      for (const auto& data : msg.data) {
        j_msg["data"].push_back(data);
      }
    } else if (msg_type == "std_msgs/msg/UInt8MultiArray") {
      // layout非対応のため1回だけ警告を出す
      RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mqtt_client"),
        "std_msgs/msg/UInt8MultiArray layout is not supported");
      std_msgs::msg::UInt8MultiArray msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["data"] = json::array();
      for (const auto& data : msg.data) {
        j_msg["data"].push_back(data);
      }
    } else if (msg_type == "std_msgs/msg/Int16MultiArray") {
      // layout非対応のため1回だけ警告を出す
      RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mqtt_client"),
        "std_msgs/msg/Int16MultiArray layout is not supported");
      std_msgs::msg::Int16MultiArray msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["data"] = json::array();
      for (const auto& data : msg.data) {
        j_msg["data"].push_back(data);
      }
    } else if (msg_type == "std_msgs/msg/UInt16MultiArray") {
      // layout非対応のため1回だけ警告を出す
      RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mqtt_client"),
        "std_msgs/msg/UInt16MultiArray layout is not supported");
      std_msgs::msg::UInt16MultiArray msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["data"] = json::array();
      for (const auto& data : msg.data) {
        j_msg["data"].push_back(data);
      }
    } else if (msg_type == "std_msgs/msg/Int32MultiArray") {
      // layout非対応のため1回だけ警告を出す
      RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mqtt_client"),
        "std_msgs/msg/Int32MultiArray layout is not supported");
      std_msgs::msg::Int32MultiArray msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["data"] = json::array();
      for (const auto& data : msg.data) {
        j_msg["data"].push_back(data);
      }
    } else if (msg_type == "std_msgs/msg/UInt32MultiArray") {
      // layout非対応のため1回だけ警告を出す
      RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mqtt_client"),
        "std_msgs/msg/UInt32MultiArray layout is not supported");
      std_msgs::msg::UInt32MultiArray msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["data"] = json::array();
      for (const auto& data : msg.data) {
        j_msg["data"].push_back(data);
      }
    } else if (msg_type == "geometry_msgs/msg/Transform") {
      geometry_msgs::msg::Transform msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["translation"]["x"] = msg.translation.x;
      j_msg["translation"]["y"] = msg.translation.y;
      j_msg["translation"]["z"] = msg.translation.z;
      j_msg["rotation"]["x"] = msg.rotation.x;
      j_msg["rotation"]["y"] = msg.rotation.y;
      j_msg["rotation"]["z"] = msg.rotation.z;
      j_msg["rotation"]["w"] = msg.rotation.w;
    } else if (msg_type == "geometry_msgs/msg/TransformStamped") {
      geometry_msgs::msg::TransformStamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["child_frame_id"] = msg.child_frame_id;
      j_msg["transform"]["translation"]["x"] = msg.transform.translation.x;
      j_msg["transform"]["translation"]["y"] = msg.transform.translation.y;
      j_msg["transform"]["translation"]["z"] = msg.transform.translation.z;
      j_msg["transform"]["rotation"]["x"] = msg.transform.rotation.x;
      j_msg["transform"]["rotation"]["y"] = msg.transform.rotation.y;
      j_msg["transform"]["rotation"]["z"] = msg.transform.rotation.z;
      j_msg["transform"]["rotation"]["w"] = msg.transform.rotation.w;

    } else if (msg_type == "geometry_msgs/msg/Vector3") {
      geometry_msgs::msg::Vector3 msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["x"] = msg.x;
      j_msg["y"] = msg.y;
      j_msg["z"] = msg.z;

    } else if (msg_type == "geometry_msgs/msg/Quaternion") {
      geometry_msgs::msg::Quaternion msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["x"] = msg.x;
      j_msg["y"] = msg.y;
      j_msg["z"] = msg.z;
      j_msg["w"] = msg.w;
    } else if (msg_type == "sensor_msgs/msg/Joy"){
      sensor_msgs::msg::Joy msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["axes"] = json::array();
      for (const auto& axis : msg.axes) {
        j_msg["axes"].push_back(axis);
      }
      j_msg["buttons"] = json::array();
      for (const auto& button : msg.buttons) {
        j_msg["buttons"].push_back(button);
      }
    }
#ifdef HAVE_TRIORB_INTERFACE
    /*
    === triorb_collaboration_interface/msg/ParentBind ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    # ==[協調搬送] 仮想（荷物など）原点に対するロボットの相対姿勢==
    std_msgs/Header header      # Header
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    string parent               # Parent name
    string you                  # Your name
    float32 x                   # Relative position of the parent from you [m]
    float32 y                   # Relative position of the parent from you [m]
    float32 deg                 # Relative position of the parent from you [deg]
    */
    else if (msg_type == "triorb_collaboration_interface/msg/ParentBind") {
      triorb_collaboration_interface::msg::ParentBind msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      //j_msg["parent"] = msg.parent;
      j_msg["you"] = msg.you;
      j_msg["x"] = msg.x;
      j_msg["y"] = msg.y;
      j_msg["deg"] = msg.deg;
    }
    /*
    === triorb_cv_interface/msg/BoundingBox ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    # ==バウンディングボックス座標==
    float32[] xtl_ytl_xbr_ybr       # [Left-top-x, Left-top-y, Right-bottom-x,
    Right-bottom-y] [pix]
    */
    else if (msg_type == "triorb_cv_interface/msg/BoundingBox") {
      triorb_cv_interface::msg::BoundingBox msg;
      deserializeRosMessage(*serialized_msg, msg);
      for (const auto& box : msg.xtl_ytl_xbr_ybr) {
        j_msg["xtl_ytl_xbr_ybr"].push_back(box);
      }
    }
    /*
    === triorb_cv_interface/msg/Detection ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    # ==物体検出結果==
    std_msgs/Header header      # Timestamp
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    uint32 det_num              # Number of detections
    BoundingBox[] boxes         # BoundingBoxs
      float32[] xtl_ytl_xbr_ybr       #
    float64[] scores            # Detection scores
    string[] labels             # Object types
    */
    else if (msg_type == "triorb_cv_interface/msg/Detection") {
      triorb_cv_interface::msg::Detection msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["det_num"] = msg.det_num;
      for (const auto& box : msg.boxes) {
        json box_json;
        for (const auto& point : box.xtl_ytl_xbr_ybr) {
          box_json["xtl_ytl_xbr_ybr"].push_back(point);
        }
        j_msg["boxes"].push_back(box_json);
      }
      for (const auto& score : msg.scores) {
        j_msg["scores"].push_back(score);
      }
      for (const auto& label : msg.labels) {
        j_msg["labels"].push_back(label);
      }
    }
    /*
    === triorb_drive_interface/msg/DriveGains ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==自律移動のゲインパラメーター==
    float32 xy_p    # translation P gain
    float32 xy_i    # translation I gain (0 recommended)
    float32 xy_d    # translation D gain
    float32 w_p     # rotation P gain
    float32 w_i     # rotation I gain (0 recommended)
    float32 w_d     # rotation D gain
    */
    else if (msg_type == "triorb_drive_interface/msg/DriveGains") {
      triorb_drive_interface::msg::DriveGains msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["xy_p"] = msg.xy_p;
      j_msg["xy_i"] = msg.xy_i;
      j_msg["xy_d"] = msg.xy_d;
      j_msg["w_p"] = msg.w_p;
      j_msg["w_i"] = msg.w_i;
      j_msg["w_d"] = msg.w_d;
    }
    /*
    === triorb_drive_interface/msg/MotorParams ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==モーター制御パラメーター==
    bool lpf                # Use LPF for driving command filter (False: moving
    average) uint8 filter_t          # Command filter time constant (0-200)[ms]
    uint8 pos_p_gain        # Position loop gain (1-50)[Hz]
    uint16 speed_p_gain     # speed loop gain (1-500) [Hz]
    uint16 speed_i_gain     # speed loop integral time constant (1-10000)
    [0.01ms] uint16 torque_filter    # torque filter (0-4700) [Hz] uint8
    speed_ff          # speed feed-forward (0-100) [%] uint8 stiffness         #
    machine stiffness selection (0-15)
    */
    else if (msg_type == "triorb_drive_interface/msg/MotorParams") {
      triorb_drive_interface::msg::MotorParams msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["lpf"] = msg.lpf;
      j_msg["filter_t"] = msg.filter_t;
      j_msg["pos_p_gain"] = msg.pos_p_gain;
      j_msg["speed_p_gain"] = msg.speed_p_gain;
      j_msg["speed_i_gain"] = msg.speed_i_gain;
      j_msg["torque_filter"] = msg.torque_filter;
      j_msg["speed_ff"] = msg.speed_ff;
      j_msg["stiffness"] = msg.stiffness;
    }
    /*
    === triorb_drive_interface/msg/MotorStatus ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==モーターステータス==
    std_msgs/Header header      # Timestamp
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    uint16 last_error_value     # Last motor alert flag
    uint8 last_error_motor      # Motor ID of the last alert
    float32 voltage             # Mains voltage observed by the motor driver
    uint16 state                # Operating state of each motor (bit flag)
    float32 power               # Power consumption of each motor (W)

    #---Operating state of each motor (bit flag)---
    # 0x8000: Remote control Y button
    # 0x4000: Remote control B button
    # 0x2000: Remote control A button
    # 0x1000: Remote control X button
    # 0x0800: Rotating
    # 0x0400: Position control complete
    # 0x0200: Excitation in progress
    # 0x0100: Motor status acquired successfully
    */
    else if (msg_type == "triorb_drive_interface/msg/MotorStatus") {
      triorb_drive_interface::msg::MotorStatus msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["last_error_value"] = msg.last_error_value;
      j_msg["last_error_motor"] = msg.last_error_motor;
      j_msg["voltage"] = msg.voltage;
      j_msg["state"] = msg.state;
      j_msg["power"] = msg.power;
    }
    /*
    === triorb_drive_interface/msg/Route ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==自律移動経路==
    uint32 id               # ID
    string name             # Name
    TriorbPos3[] waypoint   # Waypoints
      float32 x       # [m]
      float32 y       # [m]
      float32 deg     # [deg]
    */
    else if (msg_type == "triorb_drive_interface/msg/Route") {
      triorb_drive_interface::msg::Route msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["id"] = msg.id;
      j_msg["name"] = msg.name;
      for (const auto& waypoint : msg.waypoint) {
        json waypoint_json;
        waypoint_json["x"] = waypoint.x;
        waypoint_json["y"] = waypoint.y;
        waypoint_json["deg"] = waypoint.deg;
        j_msg["waypoint"].push_back(waypoint_json);
      }
    }
    /*
    === triorb_drive_interface/msg/TriorbAlignPos3 ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    uint16[] marker_id          # 位置合わせ原点とするマーカーのID
    float32[] marker_size       # マーカーのサイズ
    TriorbPos3[] position       # 原点に対する相対位置決め位置姿勢
      float32 x       # [m]
      float32 y       # [m]
      float32 deg     # [deg]
    TriorbSpeed speed           # 移動速度
      uint32 acc  #
      uint32 dec  #
      float32 xy  #
      float32 w   #
    TriorbRunSetting setting    # 走行設定
      float32 tx                  #
      float32 ty                  # Target error in Y-axis direction [±m
      float32 tr                  # Target error in rotation [±deg
      uint8 force                 #
      uint8 gain_no               #
      uint32 timeout_ms           # Timeout (in ms) for the operation to complete (set:0, disable)
      uint8[] disable_camera_idx  #

    */
    else if (msg_type == "triorb_drive_interface/msg/TriorbAlignPos3") {
      triorb_drive_interface::msg::TriorbAlignPos3 msg;
      deserializeRosMessage(*serialized_msg, msg);
      for (const auto& marker_id : msg.marker_id) {
        j_msg["marker_id"].push_back(marker_id);
      }
      for (const auto& marker_size : msg.marker_size) {
        j_msg["marker_size"].push_back(marker_size);
      }
      for (const auto& position : msg.position) {
        json position_json;
        position_json["x"] = position.x;
        position_json["y"] = position.y;
        position_json["deg"] = position.deg;
        j_msg["position"].push_back(position_json);
      }
      j_msg["speed"]["acc"] = msg.speed.acc;
      j_msg["speed"]["dec"] = msg.speed.dec;
      j_msg["speed"]["xy"] = msg.speed.xy;
      j_msg["speed"]["w"] = msg.speed.w;
      j_msg["setting"]["tx"] = msg.setting.tx;
      j_msg["setting"]["ty"] = msg.setting.ty;
      j_msg["setting"]["tr"] = msg.setting.tr;
      j_msg["setting"]["force"] = msg.setting.force;
      j_msg["setting"]["gain_no"] = msg.setting.gain_no;
      j_msg["setting"]["timeout_ms"] = msg.setting.timeout_ms;
      for (const auto& disable_camera_idx : msg.setting.disable_camera_idx) {
        j_msg["setting"]["disable_camera_idx"].push_back(disable_camera_idx);
      }
    }
    /*
    === triorb_drive_interface/msg/TriorbPos3 ===
    #==平面内の位置・姿勢==
    float32 x       # [m]
    float32 y       # [m]
    float32 deg     # [deg]
    */
    else if (msg_type == "triorb_drive_interface/msg/TriorbPos3") {
      triorb_drive_interface::msg::TriorbPos3 msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["x"] = msg.x;
      j_msg["y"] = msg.y;
      j_msg["deg"] = msg.deg;
    }
    /*
    === triorb_drive_interface/msg/TriorbPos3Stamped ===
    #==平面内の位置・姿勢==
    std_msgs/Header header  # Header
            builtin_interfaces/Time stamp
                    int32 sec
                    uint32 nanosec
            string frame_id
    float32 x               # [m]
    float32 y               # [m]
    float32 deg             # [deg]
    */
    else if (msg_type == "triorb_drive_interface/msg/TriorbPos3Stamped") {
      triorb_drive_interface::msg::TriorbPos3Stamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["x"] = msg.x;
      j_msg["y"] = msg.y;
      j_msg["deg"] = msg.deg;
    }
    /*
    === triorb_drive_interface/msg/TriorbRunPos3 ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==相対位置・姿勢指示による移動==
    TriorbSpeed speed       # Configure of moving
      uint32 acc  #
      uint32 dec  #
      float32 xy  #
      float32 w   #
    TriorbPos3 position     # Target position
      float32 x       # [m]
      float32 y       # [m]
      float32 deg     # [deg]
    */
    else if (msg_type == "triorb_drive_interface/msg/TriorbRunPos3") {
      triorb_drive_interface::msg::TriorbRunPos3 msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["speed"]["acc"] = msg.speed.acc;
      j_msg["speed"]["dec"] = msg.speed.dec;
      j_msg["speed"]["xy"] = msg.speed.xy;
      j_msg["speed"]["w"] = msg.speed.w;
      j_msg["position"]["x"] = msg.position.x;
      j_msg["position"]["y"] = msg.position.y;
      j_msg["position"]["deg"] = msg.position.deg;
    }
    /*
    === triorb_drive_interface/msg/TriorbRunResult ===
    #==自律移動結果==
    bool success                # Moving result (true: Compleat, false: Feild)
    uint8 info                  # Moving result info ( substitution NAVIGATE_RESULT )
    TriorbPos3 position         # Last robot position
      float32 x       # [m]
      float32 y       # [m]
      float32 deg     # [deg]
    */
    else if (msg_type == "triorb_drive_interface/msg/TriorbRunResult") {
      triorb_drive_interface::msg::TriorbRunResult msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["success"] = msg.success;
      j_msg["info"] = msg.info;
      j_msg["position"]["x"] = msg.position.x;
      j_msg["position"]["y"] = msg.position.y;
      j_msg["position"]["deg"] = msg.position.deg;
    }
    /*
    === triorb_drive_interface/msg/TriorbRunResultStamped ===
    #==自律移動結果==
    std_msgs/Header header  # Header
            builtin_interfaces/Time stamp
                    int32 sec
                    uint32 nanosec
            string frame_id
    bool success                # Moving result (true: Compleat, false: Feild)
    uint8 info                  # Moving result info ( substitution NAVIGATE_RESULT )
    TriorbPos3 position         # Last robot position
      float32 x       # [m]
      float32 y       # [m]
      float32 deg     # [deg]
    */
    else if (msg_type == "triorb_drive_interface/msg/TriorbRunResultStamped") {
      triorb_drive_interface::msg::TriorbRunResultStamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["success"] = msg.success;
      j_msg["info"] = msg.info;
      j_msg["position"]["x"] = msg.position.x;
      j_msg["position"]["y"] = msg.position.y;
      j_msg["position"]["deg"] = msg.position.deg;
    }

    /*
    === triorb_drive_interface/msg/TriorbRunState ===
    #==自律移動pkgの状態通知トピック==
    TriorbPos3 goal_pos     # Latest goal position
    uint8 state             # Navigate state
    */
    else if (msg_type == "triorb_drive_interface/msg/TriorbRunState") {
      triorb_drive_interface::msg::TriorbRunState msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["goal_pos"]["x"] = msg.goal_pos.x;
      j_msg["goal_pos"]["y"] = msg.goal_pos.y;
      j_msg["goal_pos"]["deg"] = msg.goal_pos.deg;
      j_msg["state"] = msg.state;
    }

    /*
    === triorb_drive_interface/msg/TriorbRunSetting ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==自律移動の位置決め設定==
    float32 tx                  # Target error in X-axis direction [±m]
    float32 ty                  # Target error in Y-axis direction [±m].
    float32 tr                  # Target error in rotation [±deg].
    uint8 force                 # Target force level
    uint8 gain_no               # Number of gain type (not set:0, basic:1)
    uint32 timeout_ms           # Timeout (in ms) for the operation to complete (set:0, disable)
    uint8[] disable_camera_idx  # Camera Index to be excluded from robot pose
    estimation
    */
    else if (msg_type == "triorb_drive_interface/msg/TriorbRunSetting") {
      triorb_drive_interface::msg::TriorbRunSetting msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["tx"] = msg.tx;
      j_msg["ty"] = msg.ty;
      j_msg["tr"] = msg.tr;
      j_msg["force"] = msg.force;
      j_msg["gain_no"] = msg.gain_no;
      j_msg["timeout_ms"] = msg.timeout_ms;
      for (const auto& disable_camera_idx : msg.disable_camera_idx) {
        j_msg["disable_camera_idx"].push_back(disable_camera_idx);
      }
    }
    /*
    === triorb_drive_interface/msg/TriorbRunVel3 ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==速度指示による移動==
    TriorbSpeed speed       # Configure of moving
      uint32 acc  #
      uint32 dec  #
      float32 xy  #
      float32 w   #
    TriorbVel3 velocity     # Target velocities
      float32 vx      #
      float32 vy      #
      float32 vw      #
    */
    else if (msg_type == "triorb_drive_interface/msg/TriorbRunVel3") {
      triorb_drive_interface::msg::TriorbRunVel3 msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["speed"]["acc"] = msg.speed.acc;
      j_msg["speed"]["dec"] = msg.speed.dec;
      j_msg["speed"]["xy"] = msg.speed.xy;
      j_msg["speed"]["w"] = msg.speed.w;
      j_msg["velocity"]["vx"] = msg.velocity.vx;
      j_msg["velocity"]["vy"] = msg.velocity.vy;
      j_msg["velocity"]["vw"] = msg.velocity.vw;
    }
    /*
    #==速度指示による移動==
    std_msgs/Header header  # Header
            builtin_interfaces/Time stamp
                    int32 sec
                    uint32 nanosec
            string frame_id
    TriorbSpeed speed       # Configure of moving
            uint32 acc  #
            uint32 dec  #
            float32 xy  #
            float32 w   #
    TriorbVel3 velocity     # Target velocities
            float32 vx      #
            float32 vy      #
            float32 vw      #
    */
    else if (msg_type == "triorb_drive_interface/msg/TriorbRunVel3Stamped"){
      triorb_drive_interface::msg::TriorbRunVel3Stamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["speed"]["acc"] = msg.speed.acc;
      j_msg["speed"]["dec"] = msg.speed.dec;
      j_msg["speed"]["xy"] = msg.speed.xy;
      j_msg["speed"]["w"] = msg.speed.w;
      j_msg["velocity"]["vx"] = msg.velocity.vx;
      j_msg["velocity"]["vy"] = msg.velocity.vy;
      j_msg["velocity"]["vw"] = msg.velocity.vw;
    }
    /*
    === triorb_drive_interface/msg/TriorbSetPath ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    TriorbSetPos3[] path
      TriorbRunPos3 pos           #
        TriorbSpeed speed       #
          uint32 acc  #
          uint32 dec  #
          float32 xy  #
          float32 w   #
        TriorbPos3 position     #
          float32 x       # [m]
          float32 y       # [m]
          float32 deg     # [deg]
      TriorbRunSetting setting    #
        float32 tx                  #
        float32 ty                  # Target error in Y-axis direction [±m
        float32 tr                  # Target error in rotation [±deg
        uint8 force                 #
        uint8 gain_no               #
        uint32 timeout_ms           # Timeout (in ms) for the operation to complete (set:0, disable)
        uint8[] disable_camera_idx  #
    */
    else if (msg_type == "triorb_drive_interface/msg/TriorbSetPath") {
      triorb_drive_interface::msg::TriorbSetPath msg;
      deserializeRosMessage(*serialized_msg, msg);
      for (const auto& path : msg.path) {
        json path_json;
        path_json["pos"]["speed"]["acc"] = path.pos.speed.acc;
        path_json["pos"]["speed"]["dec"] = path.pos.speed.dec;
        path_json["pos"]["speed"]["xy"] = path.pos.speed.xy;
        path_json["pos"]["speed"]["w"] = path.pos.speed.w;
        path_json["pos"]["position"]["x"] = path.pos.position.x;
        path_json["pos"]["position"]["y"] = path.pos.position.y;
        path_json["pos"]["position"]["deg"] = path.pos.position.deg;
        path_json["setting"]["tx"] = path.setting.tx;
        path_json["setting"]["ty"] = path.setting.ty;
        path_json["setting"]["tr"] = path.setting.tr;
        path_json["setting"]["force"] = path.setting.force;
        path_json["setting"]["gain_no"] = path.setting.gain_no;
        path_json["setting"]["timeout_ms"] = path.setting.timeout_ms;
        for (const auto& disable_camera_idx : path.setting.disable_camera_idx) {
          path_json["setting"]["disable_camera_idx"].push_back(
            disable_camera_idx);
        }
        j_msg["path"].push_back(path_json);
      }
    }
    /*
    === triorb_drive_interface/msg/TriorbSetPos3 ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==目標位置・姿勢指示による移動==
    TriorbRunPos3 pos           # Goal position
      TriorbSpeed speed       #
        uint32 acc  #
        uint32 dec  #
        float32 xy  #
        float32 w   #
      TriorbPos3 position     #
        float32 x       # [m]
        float32 y       # [m]
        float32 deg     # [deg]
    TriorbRunSetting setting    # Configure of navigation
      float32 tx                  #
      float32 ty                  # Target error in Y-axis direction [±m
      float32 tr                  # Target error in rotation [±deg
      uint8 force                 #
      uint8 gain_no               #
      uint32 timeout_ms           # Timeout (in ms) for the operation to complete (set:0, disable)
      uint8[] disable_camera_idx  #

    */
    else if (msg_type == "triorb_drive_interface/msg/TriorbSetPos3") {
      triorb_drive_interface::msg::TriorbSetPos3 msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["pos"]["speed"]["acc"] = msg.pos.speed.acc;
      j_msg["pos"]["speed"]["dec"] = msg.pos.speed.dec;
      j_msg["pos"]["speed"]["xy"] = msg.pos.speed.xy;
      j_msg["pos"]["speed"]["w"] = msg.pos.speed.w;
      j_msg["pos"]["position"]["x"] = msg.pos.position.x;
      j_msg["pos"]["position"]["y"] = msg.pos.position.y;
      j_msg["pos"]["position"]["deg"] = msg.pos.position.deg;
      j_msg["setting"]["tx"] = msg.setting.tx;
      j_msg["setting"]["ty"] = msg.setting.ty;
      j_msg["setting"]["tr"] = msg.setting.tr;
      j_msg["setting"]["force"] = msg.setting.force;
      j_msg["setting"]["gain_no"] = msg.setting.gain_no;
      j_msg["setting"]["timeout_ms"] = msg.setting.timeout_ms;
      for (const auto& disable_camera_idx : msg.setting.disable_camera_idx) {
        j_msg["setting"]["disable_camera_idx"].push_back(disable_camera_idx);
      }
    }
    /*
    === triorb_drive_interface/msg/TriorbSpeed ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==加減速時間・速度の設定==
    uint32 acc  # Acceleration time [ms]
    uint32 dec  # Deceleration time [ms]
    float32 xy  # Translation velocity [m/s]
    float32 w   # Rotation speed [rad/s]
    */
    else if (msg_type == "triorb_drive_interface/msg/TriorbSpeed") {
      triorb_drive_interface::msg::TriorbSpeed msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["acc"] = msg.acc;
      j_msg["dec"] = msg.dec;
      j_msg["xy"] = msg.xy;
      j_msg["w"] = msg.w;
    }
    /*
    === triorb_drive_interface/msg/TriorbVel3 ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==平面内の移動速度設定==
    float32 vx      # Velocity vector along X axis [m/s]
    float32 vy      # Velocity vector along Y axis [m/s]
    float32 vw      # Rotation velocity vector around the Z axis [rad/s]
    */
    else if (msg_type == "triorb_drive_interface/msg/TriorbVel3") {
      triorb_drive_interface::msg::TriorbVel3 msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["vx"] = msg.vx;
      j_msg["vy"] = msg.vy;
      j_msg["vw"] = msg.vw;
    }
    /*
    === triorb_sensor_interface/msg/CameraDevice ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==カメラデバイス==
    std_msgs/Header header      # Timestamp
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    string device               # Path of camera device
    string topic                # Topic name of camera image
    string id                   # Frame ID of the camera image topic
    string state                # Camera device status (sleep | wakeup | awake)
    int16 rotation              # Rotation of the camera image
    int16 exposure              # Camera Exposure
    float32 gamma               # Gamma correction value
    float32 timer               # Data collection cycle [s]
    */
    else if (msg_type == "triorb_sensor_interface/msg/CameraDevice") {
      triorb_sensor_interface::msg::CameraDevice msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["device"] = msg.device;
      j_msg["topic"] = msg.topic;
      j_msg["id"] = msg.id;
      j_msg["state"] = msg.state;
      j_msg["rotation"] = msg.rotation;
      j_msg["exposure"] = msg.exposure;
      j_msg["gamma"] = msg.gamma;
      j_msg["timer"] = msg.timer;
    }
    /*
    === triorb_sensor_interface/msg/DistanceSensor ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==距離センサ==
    std_msgs/Header header      # Timestamp
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    float32 distance      		# Distance to obstacle [m]
    uint8 confidence            # Signal reliability (0-100)
    float32 hfov                # Horizontal detectable angle [deg]
    float32 vfov                # Vertical detectable angle [deg]
    float32 max_dist            # Maximum detectable distance [m]
    float32 min_dist            # Minimum detectable distance [m]
    float32[] mount_xyz         # Mounting location [m]
    float32[] mount_ypr         # Mounting orientation [deg]
    */
    else if (msg_type == "triorb_sensor_interface/msg/DistanceSensor") {
      triorb_sensor_interface::msg::DistanceSensor msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["distance"] = msg.distance;
      j_msg["confidence"] = msg.confidence;
      j_msg["hfov"] = msg.hfov;
      j_msg["vfov"] = msg.vfov;
      j_msg["max_dist"] = msg.max_dist;
      j_msg["min_dist"] = msg.min_dist;
      for (const auto& mount : msg.mount_xyz) {
        j_msg["mount_xyz"].push_back(mount);
      }
      for (const auto& mount : msg.mount_ypr) {
        j_msg["mount_ypr"].push_back(mount);
      }
    }
    /*
    === triorb_sensor_interface/msg/ImuSensor ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==IMUセンサ==
    std_msgs/Header header # Timestamp
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    float32 yaw
    float32 pitch
    float32 roll
    */
    else if (msg_type == "triorb_sensor_interface/msg/ImuSensor") {
      triorb_sensor_interface::msg::ImuSensor msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["yaw"] = msg.yaw;
      j_msg["pitch"] = msg.pitch;
      j_msg["roll"] = msg.roll;
    }
    /*
    === triorb_sensor_interface/msg/Obstacles ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==障害物==
    std_msgs/Header header      # Timestamp
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    float32 forward      		# Distance to obstacle in forward [m]
    float32 left      		    # Distance to obstacle in left [m]
    float32 right      		    # Distance to obstacle in right [m]
    float32 back      		    # Distance to obstacle in back [m]
    */
    else if (msg_type == "triorb_sensor_interface/msg/Obstacles") {
      triorb_sensor_interface::msg::Obstacles msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["forward"] = msg.forward;
      j_msg["left"] = msg.left;
      j_msg["right"] = msg.right;
      j_msg["back"] = msg.back;
    }
    /*
    === triorb_slam_interface/msg/CamerasLandmarkInfo ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    std_msgs/Header header            # header
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    PointArrayStamped[] camera        # points array per camera
      std_msgs/Header
        builtin_interfaces/Time stamp
          int32 sec
          uint32 nanosec
        string frame_id
      geometry_msgs/Point[] points        #
        float64 x
        float64 y
        float64 z
    */
    else if (msg_type == "triorb_slam_interface/msg/CamerasLandmarkInfo") {
      triorb_slam_interface::msg::CamerasLandmarkInfo msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      for (const auto& camera : msg.camera) {
        json camera_json;
        camera_json["header"]["frame_id"] = camera.header.frame_id;
        camera_json["header"]["stamp"]["sec"] = camera.header.stamp.sec;
        camera_json["header"]["stamp"]["nanosec"] = camera.header.stamp.nanosec;
        for (const auto& point : camera.points) {
          json point_json;
          point_json["x"] = point.x;
          point_json["y"] = point.y;
          point_json["z"] = point.z;
          camera_json["points"].push_back(point_json);
        }
        j_msg["camera"].push_back(camera_json);
      }
    }
    /*
    === triorb_slam_interface/msg/CamerasPose ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    std_msgs/Header header         # header
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    PoseDevStamped[] camera        # pose info
      std_msgs/Header
        builtin_interfaces/Time stamp
          int32 sec
          uint32 nanosec
        string frame_id
      geometry_msgs/Pose pose             #
        Point position
          float64 x
          float64 y
          float64 z
        Quaternion orientation
          float64 x 0
          float64 y 0
          float64 z 0
          float64 w 1
      bool
    */
    else if (msg_type == "triorb_slam_interface/msg/CamerasPose") {
      triorb_slam_interface::msg::CamerasPose msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      for (const auto& camera : msg.camera) {
        json camera_json;
        camera_json["header"]["frame_id"] = camera.header.frame_id;
        camera_json["header"]["stamp"]["sec"] = camera.header.stamp.sec;
        camera_json["header"]["stamp"]["nanosec"] = camera.header.stamp.nanosec;
        camera_json["pose"]["position"]["x"] = camera.pose.position.x;
        camera_json["pose"]["position"]["y"] = camera.pose.position.y;
        camera_json["pose"]["position"]["z"] = camera.pose.position.z;
        camera_json["pose"]["orientation"]["x"] = camera.pose.orientation.x;
        camera_json["pose"]["orientation"]["y"] = camera.pose.orientation.y;
        camera_json["pose"]["orientation"]["z"] = camera.pose.orientation.z;
        camera_json["pose"]["orientation"]["w"] = camera.pose.orientation.w;
        j_msg["camera"].push_back(camera_json);
      }
    }
    /*
    === triorb_slam_interface/msg/PointArrayStamped ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    std_msgs/Header header              # header
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    geometry_msgs/Point[] points        # points array
      float64 x
      float64 y
      float64 z
    */
    else if (msg_type == "triorb_slam_interface/msg/PointArrayStamped") {
      triorb_slam_interface::msg::PointArrayStamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      for (const auto& point : msg.points) {
        json point_json;
        point_json["x"] = point.x;
        point_json["y"] = point.y;
        point_json["z"] = point.z;
        j_msg["points"].push_back(point_json);
      }
    }
    /*
    === triorb_slam_interface/msg/PoseDevStamped ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    std_msgs/Header header              # header
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    geometry_msgs/Pose pose             # pose array
      Point position
        float64 x
        float64 y
        float64 z
      Quaternion orientation
        float64 x 0
        float64 y 0
        float64 z 0
        float64 w 1
    bool valid                          # valid
    */
    else if (msg_type == "triorb_slam_interface/msg/PoseDevStamped") {
      triorb_slam_interface::msg::PoseDevStamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["pose"]["position"]["x"] = msg.pose.position.x;
      j_msg["pose"]["position"]["y"] = msg.pose.position.y;
      j_msg["pose"]["position"]["z"] = msg.pose.position.z;
      j_msg["pose"]["orientation"]["x"] = msg.pose.orientation.x;
      j_msg["pose"]["orientation"]["y"] = msg.pose.orientation.y;
      j_msg["pose"]["orientation"]["z"] = msg.pose.orientation.z;
      j_msg["pose"]["orientation"]["w"] = msg.pose.orientation.w;
      j_msg["valid"] = msg.valid;
    }
    /*
    === triorb_slam_interface/msg/UInt32MultiArrayStamped ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    std_msgs/Header header  # header
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    uint32[] data           # data array
    */
    else if (msg_type == "triorb_slam_interface/msg/UInt32MultiArrayStamped") {
      triorb_slam_interface::msg::UInt32MultiArrayStamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      for (const auto& data : msg.data) {
        j_msg["data"].push_back(data);
      }
    }
    /*
    === triorb_slam_interface/msg/XyArrayStamped ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    std_msgs/Header header  # header
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    uint16[] x              # x array
    uint16[] y              # y array
    */
    else if (msg_type == "triorb_slam_interface/msg/XyArrayStamped") {
      triorb_slam_interface::msg::XyArrayStamped msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      for (const auto& x : msg.x) {
        j_msg["x"].push_back(x);
      }
      for (const auto& y : msg.y) {
        j_msg["y"].push_back(y);
      }
    }
    /*
    === triorb_static_interface/msg/ClockSync ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==時計同期のためのメッセージ==
    std_msgs/Header header1     # Header 1
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    std_msgs/Header header2     # Header 2
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    */
    else if (msg_type == "triorb_static_interface/msg/ClockSync") {
      triorb_static_interface::msg::ClockSync msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header1"]["frame_id"] = msg.header1.frame_id;
      j_msg["header1"]["stamp"]["sec"] = msg.header1.stamp.sec;
      j_msg["header1"]["stamp"]["nanosec"] = msg.header1.stamp.nanosec;
      j_msg["header2"]["frame_id"] = msg.header2.frame_id;
      j_msg["header2"]["stamp"]["sec"] = msg.header2.stamp.sec;
      j_msg["header2"]["stamp"]["nanosec"] = msg.header2.stamp.nanosec;
    }
    /*
    === triorb_static_interface/msg/HostStatus ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
    implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==ホストコンピューターのモニター==
    std_msgs/Header header      # Timestamp
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    float32 memory_percent      # Memory usage
    float32 cpu_percent         # CPU usage
    float32 host_temperature    # Temperature of the host computer
    string wlan_ssid            # SSID of the access point
    uint8 wlan_signal           # Signal strength of the access point
    uint32 wlan_freq            # Communication speed of the access point
    float32 ping                # Ping speed to the default gateway
    uint8[] gateway             # Address of the default gateway
    */
    else if (msg_type == "triorb_static_interface/msg/HostStatus") {
      triorb_static_interface::msg::HostStatus msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["memory_percent"] = msg.memory_percent;
      j_msg["cpu_percent"] = msg.cpu_percent;
      j_msg["host_temperature"] = msg.host_temperature;
      j_msg["wlan_ssid"] = msg.wlan_ssid;
      j_msg["wlan_signal"] = msg.wlan_signal;
      j_msg["wlan_freq"] = msg.wlan_freq;
      j_msg["ping"] = msg.ping;
      for (const auto& gateway : msg.gateway) {
        j_msg["gateway"].push_back(gateway);
      }
    }
    /*
    === triorb_static_interface/msg/NodeInfo ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
    implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==ROS2ノードの状態==
    string name # Node name
    string state # Node state ( sleep | wakeup | awake )
    */
    else if (msg_type == "triorb_static_interface/msg/NodeInfo") {
      triorb_static_interface::msg::NodeInfo msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["name"] = msg.name;
      j_msg["state"] = msg.state;
    }
    /*
    === triorb_static_interface/msg/RobotError ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
    implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    std_msgs/Header header      # Timestamp
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    uint8 error                 # error code
    */
    else if (msg_type == "triorb_static_interface/msg/RobotError") {
      triorb_static_interface::msg::RobotError msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["error"] = msg.error;
    }
    /*
    === triorb_static_interface/msg/RobotStatus ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
    implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==ロボットの状態==
    std_msgs/Header header  # timestamp
      builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
      string frame_id
    float32 voltage         # main power supply voltage
    uint16 btns             # Remote control operation status (bit flag)
    uint16 state            # Robot operation state (bit flag)
    uint16 error            # Error status of the robot (bit flag)
    float32 battery         # Battery level (0.0 - 1.0)

    #---Remote control operation status (bit flag)---
    # 0x8000: Remote control Y button
    # 0x4000: Remote control B button
    # 0x2000: Remote control A button
    # 0x1000: Remote control X button

    #---Robot operation state (bit flag)---
    # 0x8000: Motor is being excited
    # 0x4000: Accepting move instruction
    # 0x2000: Moving
    # 0x1000: Self-position recognition in progress
    # 0x0800: Generating map
    # 0x0400: During anti-collision control
    # 0x0200: Position control move completed
    # 0x0010: Emergency stop is working
    # 0x0001: Status obtained successfully

    #---Error status of the robot (bit flag)---
    # 0x8000: Motor connection error
    # 0x4000: IMU and distance sensor connection error
    # 0x2000: Camera connection error
    # 0x1000: Main power supply voltage abnormal
    # 0x0001: Control ECU connection error
    */
    else if (msg_type == "triorb_static_interface/msg/RobotStatus") {
      triorb_static_interface::msg::RobotStatus msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["header"]["frame_id"] = msg.header.frame_id;
      j_msg["header"]["stamp"]["sec"] = msg.header.stamp.sec;
      j_msg["header"]["stamp"]["nanosec"] = msg.header.stamp.nanosec;
      j_msg["voltage"] = msg.voltage;
      j_msg["btns"] = msg.btns;
      j_msg["state"] = msg.state;
      j_msg["error"] = msg.error;
      j_msg["battery"] = msg.battery;
    }
    /*
    === triorb_static_interface/msg/SettingIPv4 ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
    implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==TCP/IPv4==
    string device # device name
    string method # device mode: auto | manual | shared | disabled
    uint8[] adress # IP adress
    uint8 mask # Subnet mask
    uint8[] gateway # Default gateway adress
    uint8[] mac # Hardware adress
    */
    else if (msg_type == "triorb_static_interface/msg/SettingIPv4") {
      triorb_static_interface::msg::SettingIPv4 msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["device"] = msg.device;
      j_msg["method"] = msg.method;
      for (const auto& address : msg.adress) {
        j_msg["adress"].push_back(address);
      }
      j_msg["mask"] = msg.mask;
      for (const auto& gateway : msg.gateway) {
        j_msg["gateway"].push_back(gateway);
      }
      for (const auto& mac : msg.mac) {
        j_msg["mac"].push_back(mac);
      }
    }
    /*
    === triorb_static_interface/msg/SettingROS ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
    implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==ROS2環境==
    bool ros_localhost_only # ROS_LOCALHOST_ONLY
    uint16 ros_domain_id # ROS_DOMAIN_ID
    string ros_prefix # ROS_PREFIX
    */
    else if (msg_type == "triorb_static_interface/msg/SettingROS") {
      triorb_static_interface::msg::SettingROS msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["ros_localhost_only"] = msg.ros_localhost_only;
      j_msg["ros_domain_id"] = msg.ros_domain_id;
      j_msg["ros_prefix"] = msg.ros_prefix;
    }
    /*
    === triorb_static_interface/msg/SettingSSID ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
    implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    #==無線LAN設定==
    string ssid # Wi-Fi SSID name
    string passphrase # Wi-Fi passphrase
    string security # Wi-Fi security type
    uint8 signal # Signal strength (0-100)
    */
    else if (msg_type == "triorb_static_interface/msg/SettingSSID") {
      triorb_static_interface::msg::SettingSSID msg;
      deserializeRosMessage(*serialized_msg, msg);
      j_msg["ssid"] = msg.ssid;
      j_msg["passphrase"] = msg.passphrase;
      j_msg["security"] = msg.security;
      j_msg["signal"] = msg.signal;
    }
    /*
    === triorb_static_interface/msg/StringList ===
    #**
    #* Copyright 2023 TriOrb Inc.
    #*
    #* Licensed under the Apache License, Version 2.0 (the "License");
    #* you may not use this file except in compliance with the License.
    #* You may obtain a copy of the License at
    #*
    #*     http://www.apache.org/licenses/LICENSE-2.0
    #*
    #* Unless required by applicable law or agreed to in writing, software
    #* distributed under the License is distributed on an "AS IS" BASIS,
    #* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
    implied.
    #* See the License for the specific language governing permissions and
    #* limitations under the License.
    #**

    string[] strings
    */
    else if (msg_type == "triorb_static_interface/msg/StringList") {
      triorb_static_interface::msg::StringList msg;
      deserializeRosMessage(*serialized_msg, msg);
      for (const auto& str : msg.strings) {
        j_msg["strings"].push_back(str);
      }
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

    // resolve filepaths
    broker_config_.tls.ca_certificate = resolvePath(broker_tls_ca_certificate);
    client_config_.buffer.directory = resolvePath(client_buffer_directory);
    client_config_.tls.certificate = resolvePath(client_tls_certificate);
    client_config_.tls.key = resolvePath(client_tls_key);

    // parse bridge parameters

    // ros2mqtt
    for (const auto& ros_topic : ros2mqtt_ros_topics) {

      rclcpp::Parameter mqtt_topic_param;
      if (get_parameter(fmt::format("bridge.ros2mqtt.{}.mqtt_topic", ros_topic),
                        mqtt_topic_param)) {

        // ros2mqtt[k]/ros_topic and ros2mqtt[k]/mqtt_topic
        const std::string mqtt_topic = mqtt_topic_param.as_string();
        Ros2MqttInterface& ros2mqtt = ros2mqtt_[ros_topic];
        ros2mqtt.mqtt.topic = mqtt_topic;

        // ros2mqtt[k]/primitive
        rclcpp::Parameter primitive_param;
        if (get_parameter(
              fmt::format("bridge.ros2mqtt.{}.primitive", ros_topic),
              primitive_param))
          ros2mqtt.primitive = primitive_param.as_bool();

        // ros2mqtt[k]/ros_type
        rclcpp::Parameter ros_type_param;
        if (get_parameter(fmt::format("bridge.ros2mqtt.{}.ros_type", ros_topic),
                          ros_type_param)) {
          ros2mqtt.ros.msg_type = ros_type_param.as_string();
          ros2mqtt.fixed_type = true;
          RCLCPP_DEBUG(get_logger(), "Using explicit ROS message type '%s'",
                       ros2mqtt.ros.msg_type.c_str());
        }

        // ros2mqtt[k]/inject_timestamp
        rclcpp::Parameter stamped_param;
        if (get_parameter(
              fmt::format("bridge.ros2mqtt.{}.inject_timestamp", ros_topic),
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
                          ros_topic),
              queue_size_param))
          ros2mqtt.ros.queue_size = queue_size_param.as_int();

        rclcpp::Parameter durability_param;
        if (get_parameter(
              fmt::format("bridge.ros2mqtt.{}.advanced.ros.qos.durability",
                          ros_topic),
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
                          ros_topic),
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
              fmt::format("bridge.ros2mqtt.{}.advanced.mqtt.qos", ros_topic),
              qos_param))
          ros2mqtt.mqtt.qos = qos_param.as_int();

        // ros2mqtt[k]/advanced/mqtt/retained
        rclcpp::Parameter retained_param;
        if (get_parameter(
              fmt::format("bridge.ros2mqtt.{}.advanced.mqtt.retained",
                          ros_topic),
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
                      ros_topic)
            .c_str());
      }
    }

    // mqtt2ros
    for (const auto& mqtt_topic : mqtt2ros_mqtt_topics) {

      rclcpp::Parameter ros_topic_param;
      if (get_parameter(fmt::format("bridge.mqtt2ros.{}.ros_topic", mqtt_topic),
                        ros_topic_param)) {

        // mqtt2ros[k]/mqtt_topic and mqtt2ros[k]/ros_topic
        const std::string ros_topic = ros_topic_param.as_string();
        Mqtt2RosInterface& mqtt2ros = mqtt2ros_[mqtt_topic];
        mqtt2ros.ros.topic = ros_topic;

        // mqtt2ros[k]/primitive
        rclcpp::Parameter primitive_param;
        if (get_parameter(
              fmt::format("bridge.mqtt2ros.{}.primitive", mqtt_topic),
              primitive_param))
          mqtt2ros.primitive = primitive_param.as_bool();


        rclcpp::Parameter ros_type_param;
        if (get_parameter(
              fmt::format("bridge.mqtt2ros.{}.ros_type", mqtt_topic),
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
              fmt::format("bridge.mqtt2ros.{}.advanced.mqtt.qos", mqtt_topic),
              qos_param))
          mqtt2ros.mqtt.qos = qos_param.as_int();

        // mqtt2ros[k]/advanced/ros/queue_size
        rclcpp::Parameter queue_size_param;
        if (get_parameter(
              fmt::format("bridge.mqtt2ros.{}.advanced.ros.queue_size",
                          mqtt_topic),
              queue_size_param))
          mqtt2ros.ros.queue_size = queue_size_param.as_int();

        rclcpp::Parameter durability_param;
        if (get_parameter(
              fmt::format("bridge.mqtt2ros.{}.advanced.ros.qos.durability",
                          mqtt_topic),
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
                          mqtt_topic),
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
                                      mqtt_topic),
                          latched_param)) {
          mqtt2ros.ros.latched = latched_param.as_bool();
          RCLCPP_WARN(
            get_logger(),
            fmt::format(
              "Parameter 'bridge.mqtt2ros.{}.advanced.ros.latched' is ignored "
              "since ROS 2 does not easily support latched topics.",
              mqtt_topic)
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
                      mqtt_topic)
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
