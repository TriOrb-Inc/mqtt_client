/**
 * Copyright 2023 TriOrb Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __pingpong_HPP__
#define __pingpong_HPP__

// clang-format off
#define NODE_NAME "pingpong"
#define NODE_VERSION {1,0,0}
#define UNIQUE_NODE
// clang-format on

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

#include <string>
#include <regex>
#define GET_NODE_NAME(s) std::regex_replace(std::string(getenv("ROS_PREFIX")) + std::string("_") + std::string(s), std::regex("^_"), "")
#define GET_TOPIC_NAME(s) std::regex_replace(std::string(getenv("ROS_PREFIX")) + std::string(s), std::regex("//"), "/")

// 環境変数を取得し、nullならデフォルト値を返す関数
std::string getEnvOrDefault(const std::string& varName, const std::string& defaultValue) {
    const char* value = std::getenv(varName.c_str());
    if (value == nullptr) {
        return defaultValue;
    } else {
        return std::string(value);
    }
}

class _Node : public rclcpp::Node
{
public:
    _Node();
    ~_Node()
    {
    }

private:
    std::string node_name_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_my_pong_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ping_;
    void callback_ping(const std_msgs::msg::String::SharedPtr msg);

#ifdef UNIQUE_NODE
    uint8_t unique_ok_count_;
    rclcpp::TimerBase::SharedPtr unique_check_timer_;
    void callback_unique_check();
#endif

    /**
     * @brief TriOrb ExceptHandler
     */
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_except_node_registration_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_except_error_str_add_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_except_warn_str_add_;
    
};

#endif //__pingpong_HPP__