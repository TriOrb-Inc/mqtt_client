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

#include <mqtt_client/MqttConvert.hpp>

namespace mqtt_client {

    void toHeader(const json& j_msg, std_msgs::msg::Header &msg) {
        // 各Keyが存在しない場合はデフォルト値を代入する
        msg.frame_id = j_msg["frame_id"].get<std::string>();
        if (j_msg.contains("stamp")) {
            msg.stamp.sec = j_msg["stamp"]["sec"].get<int32_t>();
            msg.stamp.nanosec = j_msg["stamp"]["nanosec"].get<uint32_t>();
        }
    }
    void fromHeader(const std_msgs::msg::Header &msg, json &j_msg) {
        j_msg["frame_id"] = msg.frame_id;
        j_msg["stamp"]["sec"] = msg.stamp.sec;
        j_msg["stamp"]["nanosec"] = msg.stamp.nanosec;
    }

    void toInt8MultiArray(const json& j_msg, std_msgs::msg::Int8MultiArray &msg) {
        for (const auto& data : j_msg.contains("data") ? j_msg["data"] : j_msg) {
            msg.data.push_back(data.get<int8_t>());
        }
    }
    void fromInt8MultiArray(const std_msgs::msg::Int8MultiArray &msg, json &j_msg) {
        j_msg["data"] = json::array();
        for (const auto& data : msg.data) {
            j_msg["data"].push_back(data);
        }
    }

    void toUInt8MultiArray(const json& j_msg, std_msgs::msg::UInt8MultiArray &msg) {
        for (const auto& data : j_msg.contains("data") ? j_msg["data"] : j_msg) {
            msg.data.push_back(data.get<uint8_t>());
        }
    }
    void fromUInt8MultiArray(const std_msgs::msg::UInt8MultiArray &msg, json &j_msg) {
        j_msg["data"] = json::array();
        for (const auto& data : msg.data) {
            j_msg["data"].push_back(data);
        }
    }

    void toInt16MultiArray(const json& j_msg, std_msgs::msg::Int16MultiArray &msg) {
        for (const auto& data : j_msg.contains("data") ? j_msg["data"] : j_msg) {
            msg.data.push_back(data.get<int16_t>());
        }
    }
    void fromInt16MultiArray(const std_msgs::msg::Int16MultiArray &msg, json &j_msg) {
        j_msg["data"] = json::array();
        for (const auto& data : msg.data) {
            j_msg["data"].push_back(data);
        }
    }

    void toUInt16MultiArray(const json& j_msg, std_msgs::msg::UInt16MultiArray &msg) {
        for (const auto& data : j_msg.contains("data") ? j_msg["data"] : j_msg) {
            msg.data.push_back(data.get<uint16_t>());
        }
    }
    void fromUInt16MultiArray(const std_msgs::msg::UInt16MultiArray &msg, json &j_msg) {
        j_msg["data"] = json::array();
        for (const auto& data : msg.data) {
            j_msg["data"].push_back(data);
        }
    }

    void toInt32MultiArray(const json& j_msg, std_msgs::msg::Int32MultiArray &msg) {
        for (const auto& data : j_msg.contains("data") ? j_msg["data"] : j_msg) {
            msg.data.push_back(data.get<int32_t>());
        }
    }
    void fromInt32MultiArray(const std_msgs::msg::Int32MultiArray &msg, json &j_msg) {
        j_msg["data"] = json::array();
        for (const auto& data : msg.data) {
            j_msg["data"].push_back(data);
        }
    }

    void toUInt32MultiArray(const json& j_msg, std_msgs::msg::UInt32MultiArray &msg) {
        for (const auto& data : j_msg.contains("data") ? j_msg["data"] : j_msg) {
            msg.data.push_back(data.get<uint32_t>());
        }
    }
    void fromUInt32MultiArray(const std_msgs::msg::UInt32MultiArray &msg, json &j_msg) {
        j_msg["data"] = json::array();
        for (const auto& data : msg.data) {
            j_msg["data"].push_back(data);
        }
    }


    void toTransform(const json& j_msg, geometry_msgs::msg::Transform &msg) {
        if (j_msg.contains("translation")) {
            msg.translation.x = j_msg["translation"]["x"].get<float>();
            msg.translation.y = j_msg["translation"]["y"].get<float>();
            msg.translation.z = j_msg["translation"]["z"].get<float>();
        }
        if (j_msg.contains("rotation")) {
            msg.rotation.x = j_msg["rotation"]["x"].get<float>();
            msg.rotation.y = j_msg["rotation"]["y"].get<float>();
            msg.rotation.z = j_msg["rotation"]["z"].get<float>();
            msg.rotation.w = j_msg["rotation"]["w"].get<float>();
        }
    }
    void fromTransform(const geometry_msgs::msg::Transform &msg, json &j_msg) {
        j_msg["translation"]["x"] = msg.translation.x;
        j_msg["translation"]["y"] = msg.translation.y;
        j_msg["translation"]["z"] = msg.translation.z;
        j_msg["rotation"]["x"] = msg.rotation.x;
        j_msg["rotation"]["y"] = msg.rotation.y;
        j_msg["rotation"]["z"] = msg.rotation.z;
        j_msg["rotation"]["w"] = msg.rotation.w;
    }

    void toTransformStamped(const json& j_msg, geometry_msgs::msg::TransformStamped &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        msg.child_frame_id = j_msg["child_frame_id"].get<std::string>();
        if (j_msg.contains("transform")) {
            toTransform(j_msg["transform"], msg.transform);
        }
    }
    void fromTransformStamped(const geometry_msgs::msg::TransformStamped &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["child_frame_id"] = msg.child_frame_id;
        fromTransform(msg.transform, j_msg["transform"]);
    }

    void toVector3(const json& j_msg, geometry_msgs::msg::Vector3 &msg) {
        msg.x = j_msg["x"].get<float>();
        msg.y = j_msg["y"].get<float>();
        msg.z = j_msg["z"].get<float>();
    }
    void fromVector3(const geometry_msgs::msg::Vector3 &msg, json &j_msg) {
        j_msg["x"] = msg.x;
        j_msg["y"] = msg.y;
        j_msg["z"] = msg.z;
    }

    void toQuaternion(const json& j_msg, geometry_msgs::msg::Quaternion &msg) {
        msg.x = j_msg["x"].get<float>();
        msg.y = j_msg["y"].get<float>();
        msg.z = j_msg["z"].get<float>();
        msg.w = j_msg["w"].get<float>();
    }
    void fromQuaternion(const geometry_msgs::msg::Quaternion &msg, json &j_msg) {
        j_msg["x"] = msg.x;
        j_msg["y"] = msg.y;
        j_msg["z"] = msg.z;
        j_msg["w"] = msg.w;
    }

    void toJoy(const json& j_msg, sensor_msgs::msg::Joy &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        if (j_msg.contains("axes")) {
            for (const auto& axis : j_msg["axes"]) {
                msg.axes.push_back(axis.get<float>());
            }
        }
        if (j_msg.contains("buttons")) {
            for (const auto& button : j_msg["buttons"]) {
                msg.buttons.push_back(button.get<uint8_t>());
            }
        }
    }
    void fromJoy(const sensor_msgs::msg::Joy &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
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
    "header:
        stamp:
            sec: 0
            nanosec: 0
        frame_id: ''
    parent: ''
    you: ''
    x: 0.0
    y: 0.0
    deg: 0.0
    "
    */
    void toParentBind(const json& j_msg, triorb_collaboration_interface::msg::ParentBind &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        //msg.parent = j_msg["parent"].get<std::string>();
        msg.you = j_msg["you"].get<std::string>();
        msg.x = j_msg["x"].get<float>();
        msg.y = j_msg["y"].get<float>();
        msg.deg = j_msg["deg"].get<float>();
    }
    void fromParentBind(const triorb_collaboration_interface::msg::ParentBind &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        //j_msg["parent"] = msg.parent;
        j_msg["you"] = msg.you;
        j_msg["x"] = msg.x;
        j_msg["y"] = msg.y;
        j_msg["deg"] = msg.deg;
    }
    /*
    === triorb_cv_interface/msg/BoundingBox ===
    # ==バウンディングボックス座標==
    float32[] xtl_ytl_xbr_ybr       # [Left-top-x, Left-top-y, Right-bottom-x,
    Right-bottom-y] [pix]
    */
    void toBoundingBox(const json& j_msg, triorb_cv_interface::msg::BoundingBox &msg) {
        for (const auto& point : j_msg["xtl_ytl_xbr_ybr"]) {
            msg.xtl_ytl_xbr_ybr.push_back(point.get<float>());
        }
    }
    void fromBoundingBox(const triorb_cv_interface::msg::BoundingBox &msg, json &j_msg) {
        j_msg["xtl_ytl_xbr_ybr"] = json::array();      // 元実装だとなかったけど多分必要？
        for (const auto& box : msg.xtl_ytl_xbr_ybr) {
            j_msg["xtl_ytl_xbr_ybr"].push_back(box);
        }
    }
    /*
    === triorb_cv_interface/msg/Detection ===
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
    void toDetection(const json& j_msg, triorb_cv_interface::msg::Detection &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        msg.det_num = j_msg["det_num"].get<uint32_t>();
        for (const auto& box : j_msg["boxes"]) {
            triorb_cv_interface::msg::BoundingBox box_msg;
            toBoundingBox(box, box_msg);
            msg.boxes.push_back(box_msg);
        }
        for (const auto& score : j_msg["scores"]) {
            msg.scores.push_back(score.get<float>());
        }
        for (const auto& label : j_msg["labels"]) {
            msg.labels.push_back(label.get<std::string>());
        }
    }
    void fromDetection(const triorb_cv_interface::msg::Detection &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["det_num"] = msg.det_num;
        j_msg["boxes"] = json::array();
        for (const auto& box : msg.boxes) {
            json box_json;
            fromBoundingBox(box, box_json);
            j_msg["boxes"].push_back(box_json);
        }
        j_msg["scores"] = json::array();
        for (const auto& score : msg.scores) {
            j_msg["scores"].push_back(score);
        }
        j_msg["labels"] = json::array();
        for (const auto& label : msg.labels) {
            j_msg["labels"].push_back(label);
        }
    }

    /*
    === triorb_drive_interface/msg/DriveGains ===
    #==自律移動のゲインパラメーター==
    float32 xy_p    # translation P gain
    float32 xy_i    # translation I gain (0 recommended)
    float32 xy_d    # translation D gain
    float32 w_p     # rotation P gain
    float32 w_i     # rotation I gain (0 recommended)
    float32 w_d     # rotation D gain
    */
    void toDriveGains(const json& j_msg, triorb_drive_interface::msg::DriveGains &msg) {
        msg.xy_p = j_msg["xy_p"].get<float>();
        msg.xy_i = j_msg["xy_i"].get<float>();
        msg.xy_d = j_msg["xy_d"].get<float>();
        msg.w_p = j_msg["w_p"].get<float>();
        msg.w_i = j_msg["w_i"].get<float>();
        msg.w_d = j_msg["w_d"].get<float>();
    }
    void fromDriveGains(const triorb_drive_interface::msg::DriveGains &msg, json &j_msg) {
        j_msg["xy_p"] = msg.xy_p;
        j_msg["xy_i"] = msg.xy_i;
        j_msg["xy_d"] = msg.xy_d;
        j_msg["w_p"] = msg.w_p;
        j_msg["w_i"] = msg.w_i;
        j_msg["w_d"] = msg.w_d;
    }
    /*
    === triorb_drive_interface/msg/MotorParams ===
    #==モーター制御パラメーター==
    bool lpf                # Use LPF for driving command filter (False:
    moving average) uint8 filter_t          # Command filter time constant
    (0-200)[ms] uint8 pos_p_gain        # Position loop gain (1-50)[Hz] uint16
    speed_p_gain     # speed loop gain (1-500) [Hz] uint16 speed_i_gain     #
    speed loop integral time constant (1-10000) [0.01ms] uint16 torque_filter
    # torque filter (0-4700) [Hz] uint8 speed_ff          # speed feed-forward
    (0-100) [%] uint8 stiffness         # machine stiffness selection (0-15)
    */
    void toMotorParams(const json& j_msg, triorb_drive_interface::msg::MotorParams &msg) {
        msg.lpf = j_msg["lpf"].get<bool>();
        msg.filter_t = j_msg["filter_t"].get<uint8_t>();
        msg.pos_p_gain = j_msg["pos_p_gain"].get<uint8_t>();
        msg.speed_p_gain = j_msg["speed_p_gain"].get<uint16_t>();
        msg.speed_i_gain = j_msg["speed_i_gain"].get<uint16_t>();
        msg.torque_filter = j_msg["torque_filter"].get<uint16_t>();
        msg.speed_ff = j_msg["speed_ff"].get<uint8_t>();
        msg.stiffness = j_msg["stiffness"].get<uint8_t>();
    }
    void fromMotorParams(const triorb_drive_interface::msg::MotorParams &msg, json &j_msg) {
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
    */
    void toMotorStatus(const json& j_msg, triorb_drive_interface::msg::MotorStatus &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        msg.last_error_value = j_msg["last_error_value"].get<uint16_t>();
        msg.last_error_motor = j_msg["last_error_motor"].get<uint8_t>();
        msg.voltage = j_msg["voltage"].get<float>();
        msg.state = j_msg["state"].get<uint16_t>();
        msg.power = j_msg["power"].get<float>();
    }
    void fromMotorStatus(const triorb_drive_interface::msg::MotorStatus &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["last_error_value"] = msg.last_error_value;
        j_msg["last_error_motor"] = msg.last_error_motor;
        j_msg["voltage"] = msg.voltage;
        j_msg["state"] = msg.state;
        j_msg["power"] = msg.power;
    }
    /*
    === triorb_drive_interface/msg/Route ===
    #==自律移動経路==
    uint32 id               # ID
    string name             # Name
    TriorbPos3[] waypoint   # Waypoints
        float32 x       # [m]
        float32 y       # [m]
        float32 deg     # [deg]
    */
    void toRoute(const json& j_msg, triorb_drive_interface::msg::Route &msg) {
        msg.id = j_msg["id"].get<uint32_t>();
        msg.name = j_msg["name"].get<std::string>();
        for (const auto& waypoint : j_msg["waypoint"]) {
            triorb_drive_interface::msg::TriorbPos3 pos;
            toTriorbPos3(waypoint, pos);
            msg.waypoint.push_back(pos);
        }
    }
    void fromRoute(const triorb_drive_interface::msg::Route &msg, json &j_msg) {
        j_msg["id"] = msg.id;
        j_msg["name"] = msg.name;
        j_msg["waypoint"] = json::array();
        for (const auto& waypoint : msg.waypoint) {
            json waypoint_json;
            fromTriorbPos3(waypoint, waypoint_json);
            j_msg["waypoint"].push_back(waypoint_json);
        }
    }
    /*
    === triorb_drive_interface/msg/TriorbAlignPos3 ===
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
    void toTriorbAlignPos3(const json& j_msg, triorb_drive_interface::msg::TriorbAlignPos3 &msg) {
        for (const auto& marker_id : j_msg["marker_id"]) {
            msg.marker_id.push_back(marker_id.get<uint16_t>());
        }
        for (const auto& marker_size : j_msg["marker_size"]) {
            msg.marker_size.push_back(marker_size.get<float>());
        }
        for (const auto& position : j_msg["position"]) {
            triorb_drive_interface::msg::TriorbPos3 pos;
            toTriorbPos3(position, pos);
            msg.position.push_back(pos);
        }
        if (j_msg.contains("speed")) {
            toTriorbSpeed(j_msg["speed"], msg.speed);
        }
        if (j_msg.contains("setting")) {
            toTriorbRunSetting(j_msg["setting"], msg.setting);
        }
    }
    void fromTriorbAlignPos3(const triorb_drive_interface::msg::TriorbAlignPos3 &msg, json &j_msg) {
        j_msg["marker_id"] = json::array(); // 元実装だとなかった
        j_msg["marker_size"] = json::array();
        j_msg["position"] = json::array();
        for (const auto& marker_id : msg.marker_id) {
            j_msg["marker_id"].push_back(marker_id);
        }
        for (const auto& marker_size : msg.marker_size) {
            j_msg["marker_size"].push_back(marker_size);
        }
        for (const auto& position : msg.position) {
            json position_json;
            fromTriorbPos3(position, position_json);
            j_msg["position"].push_back(position_json);
        }
        fromTriorbSpeed(msg.speed, j_msg["speed"]);
        fromTriorbRunSetting(msg.setting, j_msg["setting"]);
    }
    /*
    === triorb_drive_interface/msg/TriorbPos3 ===
    #==平面内の位置・姿勢==
    float32 x       # [m]
    float32 y       # [m]
    float32 deg     # [deg]
    */
    void toTriorbPos3(const json& j_msg, triorb_drive_interface::msg::TriorbPos3 &msg) {
        msg.x = j_msg["x"].get<float>();
        msg.y = j_msg["y"].get<float>();
        msg.deg = j_msg["deg"].get<float>();
    }
    void fromTriorbPos3(const triorb_drive_interface::msg::TriorbPos3 &msg, json &j_msg) {
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
    void toTriorbPos3Stamped(const json& j_msg, triorb_drive_interface::msg::TriorbPos3Stamped &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        msg.x = j_msg["x"].get<float>();
        msg.y = j_msg["y"].get<float>();
        msg.deg = j_msg["deg"].get<float>();
    }
    void fromTriorbPos3Stamped(const triorb_drive_interface::msg::TriorbPos3Stamped &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["x"] = msg.x;
        j_msg["y"] = msg.y;
        j_msg["deg"] = msg.deg;
    }
    /*
    === triorb_drive_interface/msg/TriorbRunPos3 ===
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
    void toTriorbRunPos3(const json& j_msg, triorb_drive_interface::msg::TriorbRunPos3 &msg) {
        if (j_msg.contains("speed")) {
            toTriorbSpeed(j_msg["speed"], msg.speed);
        }
        if (j_msg.contains("position")) {
            toTriorbPos3(j_msg["position"], msg.position);
        }
    }
    void fromTriorbRunPos3(const triorb_drive_interface::msg::TriorbRunPos3 &msg, json &j_msg) {
        fromTriorbSpeed(msg.speed, j_msg["speed"]);
        fromTriorbPos3(msg.position, j_msg["position"]);
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
    void toTriorbRunResult(const json& j_msg, triorb_drive_interface::msg::TriorbRunResult &msg) {
        msg.success = j_msg["success"].get<bool>();
        msg.info = j_msg["info"].get<uint8_t>();
        if (j_msg.contains("position")) {
            toTriorbPos3(j_msg["position"], msg.position);
        }
    }
    void fromTriorbRunResult(const triorb_drive_interface::msg::TriorbRunResult &msg, json &j_msg) {
        j_msg["success"] = msg.success;
        j_msg["info"] = msg.info;
        fromTriorbPos3(msg.position, j_msg["position"]);
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
    void toTriorbRunResultStamped(const json& j_msg, triorb_drive_interface::msg::TriorbRunResultStamped &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        msg.success = j_msg["success"].get<bool>();
        msg.info = j_msg["info"].get<uint8_t>();
        if (j_msg.contains("position")) {
            toTriorbPos3(j_msg["position"], msg.position);
        }
    }
    void fromTriorbRunResultStamped(const triorb_drive_interface::msg::TriorbRunResultStamped &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["success"] = msg.success;
        j_msg["info"] = msg.info;
        fromTriorbPos3(msg.position, j_msg["position"]);
    }
    /*
    === triorb_drive_interface/msg/TriorbRunState ===
    #==自律移動pkgの状態通知トピック==
    TriorbPos3 goal_pos     # Latest goal position
    uint8 state             # Navigate state
    */
    void toTriorbRunState(const json& j_msg, triorb_drive_interface::msg::TriorbRunState &msg) {
        if (j_msg.contains("goal_pos")) {
            toTriorbPos3(j_msg["goal_pos"], msg.goal_pos);
        }
        msg.state = j_msg["state"].get<uint8_t>();
        msg.cap_vxy = j_msg["cap_vxy"].get<float>();
        msg.cap_vw  = j_msg["cap_vw"].get<float>();
    }
    void fromTriorbRunState(const triorb_drive_interface::msg::TriorbRunState &msg, json &j_msg) {
        fromTriorbPos3(msg.goal_pos, j_msg["goal_pos"]);
        j_msg["state"] = msg.state;
        j_msg["cap_vxy"] = msg.cap_vxy;
        j_msg["cap_vw"] = msg.cap_vw;
    }
    /*
    === triorb_drive_interface/msg/TriorbRunSetting ===
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
    void toTriorbRunSetting(const json& j_msg, triorb_drive_interface::msg::TriorbRunSetting &msg) {
        msg.tx = j_msg["tx"].get<float>();
        msg.ty = j_msg["ty"].get<float>();
        msg.tr = j_msg["tr"].get<float>();
        msg.force = j_msg["force"].get<uint8_t>();
        msg.gain_no = j_msg["gain_no"].get<uint8_t>();
        if (j_msg.contains("timeout_ms")){
            msg.timeout_ms = j_msg["timeout_ms"].get<uint32_t>();
        }
        if (j_msg.contains("disable_camera_idx")) {
            for (const auto& disable_camera_idx : j_msg["disable_camera_idx"]) {
                msg.disable_camera_idx.push_back(disable_camera_idx.get<uint8_t>());
            }
        }
    }
    void fromTriorbRunSetting(const triorb_drive_interface::msg::TriorbRunSetting &msg, json &j_msg) {
        j_msg["tx"] = msg.tx;
        j_msg["ty"] = msg.ty;
        j_msg["tr"] = msg.tr;
        j_msg["force"] = msg.force;
        j_msg["gain_no"] = msg.gain_no;
        j_msg["timeout_ms"] = msg.timeout_ms;
        j_msg["disable_camera_idx"] = json::array();
        for (const auto& disable_camera_idx : msg.disable_camera_idx) {
            j_msg["disable_camera_idx"].push_back(disable_camera_idx);
        }
    }
    /*
    === triorb_drive_interface/msg/TriorbRunVel3 ===
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
    void toTriorbRunVel3(const json& j_msg, triorb_drive_interface::msg::TriorbRunVel3 &msg) {
        if (j_msg.contains("speed")) {
            toTriorbSpeed(j_msg["speed"], msg.speed);
        }
        if (j_msg.contains("velocity")) {
            toTriorbVel3(j_msg["velocity"], msg.velocity);
        }
    }
    void fromTriorbRunVel3(const triorb_drive_interface::msg::TriorbRunVel3 &msg, json &j_msg) {
        fromTriorbSpeed(msg.speed, j_msg["speed"]);
        fromTriorbVel3(msg.velocity, j_msg["velocity"]);
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
    void toTriorbRunVel3Stamped(const json& j_msg, triorb_drive_interface::msg::TriorbRunVel3Stamped &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        if (j_msg.contains("speed")) {
            toTriorbSpeed(j_msg["speed"], msg.speed);
        }
        if (j_msg.contains("velocity")) {
            toTriorbVel3(j_msg["velocity"], msg.velocity);
        }
    }
    void fromTriorbRunVel3Stamped(const triorb_drive_interface::msg::TriorbRunVel3Stamped &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        fromTriorbSpeed(msg.speed, j_msg["speed"]);
        fromTriorbVel3(msg.velocity, j_msg["velocity"]);
    }
    /*
    === triorb_drive_interface/msg/TriorbSetPath ===
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
    void toTriorbSetPath(const json& j_msg, triorb_drive_interface::msg::TriorbSetPath &msg) {
        for (const auto& path : j_msg["path"]) {
            triorb_drive_interface::msg::TriorbSetPos3 path_msg;
            toTriorbSetPos3(path, path_msg);
            msg.path.push_back(path_msg);
        }
    }
    void fromTriorbSetPath(const triorb_drive_interface::msg::TriorbSetPath &msg, json &j_msg) {
        j_msg["path"] = json::array();
        for (const auto& path : msg.path) {
            json path_json;
            fromTriorbSetPos3(path, path_json);
            j_msg["path"].push_back(path_json);
        }
    }
    /*
    === triorb_drive_interface/msg/TriorbSetPos3 ===
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
    void toTriorbSetPos3(const json& j_msg, triorb_drive_interface::msg::TriorbSetPos3 &msg) {
        if (j_msg.contains("pos")) {
            toTriorbRunPos3(j_msg["pos"], msg.pos);
        }
        if (j_msg.contains("setting")) {
            toTriorbRunSetting(j_msg["setting"], msg.setting);
        }
    }
    void fromTriorbSetPos3(const triorb_drive_interface::msg::TriorbSetPos3 &msg, json &j_msg) {
        fromTriorbRunPos3(msg.pos, j_msg["pos"]);
        fromTriorbRunSetting(msg.setting, j_msg["setting"]);
    }
    /*
    === triorb_drive_interface/msg/TriorbSpeed ===
    #==加減速時間・速度の設定==
    uint32 acc  # Acceleration time [ms]
    uint32 dec  # Deceleration time [ms]
    float32 xy  # Translation velocity [m/s]
    float32 w   # Rotation speed [rad/s]
    */
    void toTriorbSpeed(const json& j_msg, triorb_drive_interface::msg::TriorbSpeed &msg) {
        msg.acc = j_msg["acc"].get<uint32_t>();
        msg.dec = j_msg["dec"].get<uint32_t>();
        msg.xy = j_msg["xy"].get<float>();
        msg.w = j_msg["w"].get<float>();
    }
    void fromTriorbSpeed(const triorb_drive_interface::msg::TriorbSpeed &msg, json &j_msg) {
        j_msg["acc"] = msg.acc;
        j_msg["dec"] = msg.dec;
        j_msg["xy"] = msg.xy;
        j_msg["w"] = msg.w;
    }
    /*
    === triorb_drive_interface/msg/TriorbVel3 ===
    #==平面内の移動速度設定==
    float32 vx      # Velocity vector along X axis [m/s]
    float32 vy      # Velocity vector along Y axis [m/s]
    float32 vw      # Rotation velocity vector around the Z axis [rad/s]
    */
    void toTriorbVel3(const json& j_msg, triorb_drive_interface::msg::TriorbVel3 &msg) {
        msg.vx = j_msg["vx"].get<float>();
        msg.vy = j_msg["vy"].get<float>();
        msg.vw = j_msg["vw"].get<float>();
    }
    void fromTriorbVel3(const triorb_drive_interface::msg::TriorbVel3 &msg, json &j_msg) {
        j_msg["vx"] = msg.vx;
        j_msg["vy"] = msg.vy;
        j_msg["vw"] = msg.vw;
    }

    /*
    === triorb_sensor_interface/msg/CameraDevice ===
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
    float32 timer               # Data collection cycle[s]
    */
    void toCameraDevice(const json& j_msg, triorb_sensor_interface::msg::CameraDevice &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        msg.device = j_msg["device"].get<std::string>();
        msg.topic = j_msg["topic"].get<std::string>();
        msg.id = j_msg["id"].get<std::string>();
        msg.state = j_msg["state"].get<std::string>();
        msg.rotation = j_msg["rotation"].get<int16_t>();
        msg.exposure = j_msg["exposure"].get<int16_t>();
        msg.gamma = j_msg["gamma"].get<float>();
        msg.timer = j_msg["timer"].get<float>();
    }
    void fromCameraDevice(const triorb_sensor_interface::msg::CameraDevice &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
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
    void toDistanceSensor(const json& j_msg, triorb_sensor_interface::msg::DistanceSensor &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        msg.distance = j_msg["distance"].get<float>();
        msg.confidence = j_msg["confidence"].get<uint8_t>();
        msg.hfov = j_msg["hfov"].get<float>();
        msg.vfov = j_msg["vfov"].get<float>();
        msg.max_dist = j_msg["max_dist"].get<float>();
        msg.min_dist = j_msg["min_dist"].get<float>();
        for (const auto& mount_xyz : j_msg["mount_xyz"]) {
            msg.mount_xyz.push_back(mount_xyz.get<float>());
        }
        for (const auto& mount_ypr : j_msg["mount_ypr"]) {
            msg.mount_ypr.push_back(mount_ypr.get<float>());
        }
    }
    void fromDistanceSensor(const triorb_sensor_interface::msg::DistanceSensor &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["distance"] = msg.distance;
        j_msg["confidence"] = msg.confidence;
        j_msg["hfov"] = msg.hfov;
        j_msg["vfov"] = msg.vfov;
        j_msg["max_dist"] = msg.max_dist;
        j_msg["min_dist"] = msg.min_dist;
        j_msg["mount_xyz"] = json::array(); // 元実装だとなかった
        j_msg["mount_ypr"] = json::array();
        for (const auto& mount_xyz : msg.mount_xyz) {
            j_msg["mount_xyz"].push_back(mount_xyz);
        }
        for (const auto& mount_ypr : msg.mount_ypr) {
            j_msg["mount_ypr"].push_back(mount_ypr);
        }
    }
    /*
    === triorb_sensor_interface/msg/ImuSensor ===
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
    void toImuSensor(const json& j_msg, triorb_sensor_interface::msg::ImuSensor &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        msg.yaw = j_msg["yaw"].get<float>();
        msg.pitch = j_msg["pitch"].get<float>();
        msg.roll = j_msg["roll"].get<float>();
    }
    void fromImuSensor(const triorb_sensor_interface::msg::ImuSensor &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["yaw"] = msg.yaw;
        j_msg["pitch"] = msg.pitch;
        j_msg["roll"] = msg.roll;
    }
    /*
    === triorb_sensor_interface/msg/Obstacles ===
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
    void toObstacles(const json& j_msg, triorb_sensor_interface::msg::Obstacles &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        msg.forward = j_msg["forward"].get<float>();
        msg.left = j_msg["left"].get<float>();
        msg.right = j_msg["right"].get<float>();
        msg.back = j_msg["back"].get<float>();
    }
    void fromObstacles(const triorb_sensor_interface::msg::Obstacles &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["forward"] = msg.forward;
        j_msg["left"] = msg.left;
        j_msg["right"] = msg.right;
        j_msg["back"] = msg.back;
    }

    /*
    === triorb_slam_interface/msg/CamerasLandmarkInfo ===
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
    void toCamerasLandmarkInfo(const json& j_msg, triorb_slam_interface::msg::CamerasLandmarkInfo &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        for (const auto& camera : j_msg["camera"]) {
            triorb_slam_interface::msg::PointArrayStamped camera_msg;
            toPointArrayStamped(camera, camera_msg);
            msg.camera.push_back(camera_msg);
        }
    }
    void fromCamerasLandmarkInfo(const triorb_slam_interface::msg::CamerasLandmarkInfo &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["camera"] = json::array(); // 元実装だとなかった
        for (const auto& camera : msg.camera) {
            json camera_json;
            fromPointArrayStamped(camera, camera_json);
            j_msg["camera"].push_back(camera_json);
        }
    }
    /*
    === triorb_slam_interface/msg/CamerasPose ===
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
    void toCamerasPose(const json& j_msg, triorb_slam_interface::msg::CamerasPose &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        for (const auto& camera : j_msg["camera"]) {
            triorb_slam_interface::msg::PoseDevStamped camera_msg;
            toPoseDevStamped(camera, camera_msg);
            msg.camera.push_back(camera_msg);
        }
    }
    void fromCamerasPose(const triorb_slam_interface::msg::CamerasPose &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["camera"] = json::array(); // 元実装だとなかった
        for (const auto& camera : msg.camera) {
            json camera_json;
            fromPoseDevStamped(camera, camera_json);
            j_msg["camera"].push_back(camera_json);
        }
    }
    /*
    === triorb_slam_interface/msg/PointArrayStamped ===
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
    void toPointArrayStamped(const json& j_msg, triorb_slam_interface::msg::PointArrayStamped &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        for (const auto& point : j_msg["points"]) {
            geometry_msgs::msg::Point point_msg; // toPointは作っていないのでベタ書き
            point_msg.x = point["x"].get<double>();
            point_msg.y = point["y"].get<double>();
            point_msg.z = point["z"].get<double>();
            msg.points.push_back(point_msg);
        }
    }
    void fromPointArrayStamped(const triorb_slam_interface::msg::PointArrayStamped &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["points"] = json::array(); // 元実装だとなかった
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
    void toPoseDevStamped(const json& j_msg, triorb_slam_interface::msg::PoseDevStamped &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        if (j_msg.contains("pose")) {
            // toPoseは作っていないのでベタ書き
            if (j_msg["pose"].contains("position")) {
              msg.pose.position.x = j_msg["pose"]["position"]["x"].get<double>();
              msg.pose.position.y = j_msg["pose"]["position"]["y"].get<double>();
              msg.pose.position.z = j_msg["pose"]["position"]["z"].get<double>();
            }
            if (j_msg["pose"].contains("orientation")) {
              msg.pose.orientation.x = j_msg["pose"]["orientation"]["x"].get<double>();
              msg.pose.orientation.y = j_msg["pose"]["orientation"]["y"].get<double>();
              msg.pose.orientation.z = j_msg["pose"]["orientation"]["z"].get<double>();
              msg.pose.orientation.w = j_msg["pose"]["orientation"]["w"].get<double>();
            }
        }
        msg.valid = j_msg["valid"].get<bool>();
    }
    void fromPoseDevStamped(const triorb_slam_interface::msg::PoseDevStamped &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
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
    std_msgs/Header header  # header
        builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
        string frame_id
    uint32[] data           # data array
    */
    void toUInt32MultiArrayStamped(const json& j_msg, triorb_slam_interface::msg::UInt32MultiArrayStamped &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        for (const auto& data : j_msg["data"]) {
            msg.data.push_back(data.get<uint32_t>());
        }
    }
    void fromUInt32MultiArrayStamped(const triorb_slam_interface::msg::UInt32MultiArrayStamped &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["data"] = json::array(); // 元実装だとなかった
        for (const auto& data : msg.data) {
            j_msg["data"].push_back(data);
        }
    }
    /*
    === triorb_slam_interface/msg/XyArrayStamped ===
    std_msgs/Header header  # header
        builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
        string frame_id
    uint16[] x              # x array
    uint16[] y              # y array
    */
    void toXyArrayStamped(const json& j_msg, triorb_slam_interface::msg::XyArrayStamped &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        for (const auto& x : j_msg["x"]) {
            msg.x.push_back(x.get<uint16_t>());
        }
        for (const auto& y : j_msg["y"]) {
            msg.y.push_back(y.get<uint16_t>());
        }
    }
    void fromXyArrayStamped(const triorb_slam_interface::msg::XyArrayStamped &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["x"] = json::array(); // 元実装だとなかった
        j_msg["y"] = json::array();
        for (const auto& x : msg.x) {
            j_msg["x"].push_back(x);
        }
        for (const auto& y : msg.y) {
            j_msg["y"].push_back(y);
        }
    }

    /*
    === triorb_static_interface/msg/ClockSync ===
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
    void toClockSync(const json& j_msg, triorb_static_interface::msg::ClockSync &msg) {
        if (j_msg.contains("header1")) {
            toHeader(j_msg["header1"], msg.header1);
        }
        if (j_msg.contains("header2")) {
            toHeader(j_msg["header2"], msg.header2);
        }
    }
    void fromClockSync(const triorb_static_interface::msg::ClockSync &msg, json &j_msg) {
        fromHeader(msg.header1, j_msg["header1"]);
        fromHeader(msg.header2, j_msg["header2"]);
    }
    /*
    === triorb_static_interface/msg/HostStatus ===
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
    void toHostStatus(const json& j_msg, triorb_static_interface::msg::HostStatus &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        msg.memory_percent = j_msg["memory_percent"].get<float>();
        msg.cpu_percent = j_msg["cpu_percent"].get<float>();
        msg.host_temperature = j_msg["host_temperature"].get<float>();
        msg.wlan_ssid = j_msg["wlan_ssid"].get<std::string>();
        msg.wlan_signal = j_msg["wlan_signal"].get<uint8_t>();
        msg.wlan_freq = j_msg["wlan_freq"].get<uint32_t>();
        msg.ping = j_msg["ping"].get<float>();
        for (const auto& gateway : j_msg["gateway"]) {
            msg.gateway.push_back(gateway.get<uint8_t>());
        }
    }
    void fromHostStatus(const triorb_static_interface::msg::HostStatus &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
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
    #==ROS2ノードの状態==
    string name # Node name
    string state # Node state ( sleep | wakeup | awake )
    */
    void toNodeInfo(const json& j_msg, triorb_static_interface::msg::NodeInfo &msg) {
        msg.name = j_msg["name"].get<std::string>();
        msg.state = j_msg["state"].get<std::string>();
    }
    void fromNodeInfo(const triorb_static_interface::msg::NodeInfo &msg, json &j_msg) {
        j_msg["name"] = msg.name;
        j_msg["state"] = msg.state;
    }
    /*
    === triorb_static_interface/msg/RobotError ===
    std_msgs/Header header      # Timestamp
        builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
        string frame_id
    uint8 error                 # error code
    */
    void toRobotError(const json& j_msg, triorb_static_interface::msg::RobotError &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        msg.error = j_msg["error"].get<uint8_t>();
    }
    void fromRobotError(const triorb_static_interface::msg::RobotError &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["error"] = msg.error;
    }
    /*
    === triorb_static_interface/msg/RobotStatus ===
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
    */
    void toRobotStatus(const json& j_msg, triorb_static_interface::msg::RobotStatus &msg) {
        if (j_msg.contains("header")) {
            toHeader(j_msg["header"], msg.header);
        }
        msg.voltage = j_msg["voltage"].get<float>();
        msg.btns = j_msg["btns"].get<uint16_t>();
        msg.state = j_msg["state"].get<uint16_t>();
        msg.error = j_msg["error"].get<uint16_t>();
        msg.battery = j_msg["battery"].get<float>();
    }
    void fromRobotStatus(const triorb_static_interface::msg::RobotStatus &msg, json &j_msg) {
        fromHeader(msg.header, j_msg["header"]);
        j_msg["voltage"] = msg.voltage;
        j_msg["btns"] = msg.btns;
        j_msg["state"] = msg.state;
        j_msg["error"] = msg.error;
        j_msg["battery"] = msg.battery;
    }
    /*
    === triorb_static_interface/msg/SettingIPv4 ===
    #==TCP/IPv4==
    string device # device name
    string method # device mode: auto | manual | shared | disabled
    uint8[] adress # IP adress
    uint8 mask # Subnet mask
    uint8[] gateway # Default gateway adress
    uint8[] mac # Hardware adress
    */
    void toSettingIPv4(const json& j_msg, triorb_static_interface::msg::SettingIPv4 &msg) {
        msg.device = j_msg["device"].get<std::string>();
        msg.method = j_msg["method"].get<std::string>();
        for (const auto& address : j_msg["adress"]) {
            msg.adress.push_back(address.get<uint8_t>());
        }
        msg.mask = j_msg["mask"].get<uint8_t>();
        for (const auto& gateway : j_msg["gateway"]) {
            msg.gateway.push_back(gateway.get<uint8_t>());
        }
        for (const auto& mac : j_msg["mac"]) {
            msg.mac.push_back(mac.get<uint8_t>());
        }
    }
    void fromSettingIPv4(const triorb_static_interface::msg::SettingIPv4 &msg, json &j_msg) {
        j_msg["device"] = msg.device;
        j_msg["method"] = msg.method;
        j_msg["mask"] = msg.mask;
        j_msg["adress"] = json::array(); // 元実装だとなかった
        j_msg["gateway"] = json::array();
        j_msg["mac"] = json::array();
        for (const auto& address : msg.adress) {
            j_msg["adress"].push_back(address);
        }
        for (const auto& gateway : msg.gateway) {
            j_msg["gateway"].push_back(gateway);
        }
        for (const auto& mac : msg.mac) {
            j_msg["mac"].push_back(mac);
        }
    }
    /*
    === triorb_static_interface/msg/SettingROS ===
    #==ROS2環境==
    bool ros_localhost_only # ROS_LOCALHOST_ONLY
    uint16 ros_domain_id # ROS_DOMAIN_ID
    string ros_prefix # ROS_PREFIX
    */
    void toSettingROS(const json& j_msg, triorb_static_interface::msg::SettingROS &msg) {
        msg.ros_localhost_only = j_msg["ros_localhost_only"].get<bool>();
        msg.ros_domain_id = j_msg["ros_domain_id"].get<uint16_t>();
        msg.ros_prefix = j_msg["ros_prefix"].get<std::string>();
    }
    void fromSettingROS(const triorb_static_interface::msg::SettingROS &msg, json &j_msg) {
        j_msg["ros_localhost_only"] = msg.ros_localhost_only;
        j_msg["ros_domain_id"] = msg.ros_domain_id;
        j_msg["ros_prefix"] = msg.ros_prefix;
    }
    /*
    === triorb_static_interface/msg/SettingSSID ===
    #==無線LAN設定==
    string ssid # Wi-Fi SSID name
    string passphrase # Wi-Fi passphrase
    string security # Wi-Fi security type
    uint8 signal # Signal strength (0-100)
    */
    void toSettingSSID(const json& j_msg, triorb_static_interface::msg::SettingSSID &msg) {
        msg.ssid = j_msg["ssid"].get<std::string>();
        msg.passphrase = j_msg["passphrase"].get<std::string>();
        msg.security = j_msg["security"].get<std::string>();
        msg.signal = j_msg["signal"].get<uint8_t>();
    }
    void fromSettingSSID(const triorb_static_interface::msg::SettingSSID &msg, json &j_msg) {
        j_msg["ssid"] = msg.ssid;
        j_msg["passphrase"] = msg.passphrase;
        j_msg["security"] = msg.security;
        j_msg["signal"] = msg.signal;
    }
    /*
    === triorb_static_interface/msg/StringList ===
    string[] strings
    */
    void toStringList(const json& j_msg, triorb_static_interface::msg::StringList &msg) {
        for (const auto& str : j_msg["strings"]) {
            msg.strings.push_back(str.get<std::string>());
        }
    }
    void fromStringList(const triorb_static_interface::msg::StringList &msg, json &j_msg) {
        j_msg["strings"] = json::array(); // 元実装だとなかった
        for (const auto& str : msg.strings) {
            j_msg["strings"].push_back(str);
        }
    }
#endif // HAVE_TRIORB_INTERFACE
}  // namespace mqtt_client
