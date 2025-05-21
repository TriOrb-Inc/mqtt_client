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

#include "mqtt_pingpong.hpp"

_Node::_Node() : Node(GET_NODE_NAME(NODE_NAME)), node_name_(GET_NODE_NAME(NODE_NAME))
{
    
    /**
     * Ping-pong topic for the ROS2
     * This is used to check if the connection is still alive
     */
    this->pub_my_pong_ = this->create_publisher<std_msgs::msg::String>(GET_TOPIC_NAME(std::string("/ros2/pong")), rclcpp::SensorDataQoS());
    this->sub_ping_ = this->create_subscription<std_msgs::msg::String>(GET_TOPIC_NAME(std::string("/ros2/ping")), rclcpp::SensorDataQoS(), std::bind(&_Node::callback_ping, this, std::placeholders::_1));
#ifdef UNIQUE_NODE
    this->unique_check_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&_Node::callback_unique_check, this));
#endif

    this->pub_except_node_registration_ = create_publisher<std_msgs::msg::String>(GET_TOPIC_NAME(std::string("/except_handl/node/add")), rclcpp::ParametersQoS());
    this->pub_except_error_str_add_ = create_publisher<std_msgs::msg::String>(GET_TOPIC_NAME(std::string("/triorb/error/str/add")), rclcpp::ParametersQoS());
    this->pub_except_warn_str_add_ = create_publisher<std_msgs::msg::String>(GET_TOPIC_NAME(std::string("/triorb/warn/str/add")), rclcpp::ParametersQoS());
    std_msgs::msg::String msg;
    msg.data = std::string("[instant]") + this->get_name();
    this->pub_except_node_registration_->publish(msg);
}

void _Node::callback_ping(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "callback_ping() %s", msg->data.c_str());
    auto pong_msg = std_msgs::msg::String();
    pong_msg.data = msg->data;
    this->pub_my_pong_->publish(pong_msg);
  }

#ifdef UNIQUE_NODE
void _Node::callback_unique_check()
{
    RCLCPP_INFO(this->get_logger(), "callback_unique_check()");
    std::vector<std::string> _available_nodes = rclcpp::Node::get_node_names();
    if (std::count(_available_nodes.begin(), _available_nodes.end(), std::string("/") + GET_NODE_NAME(NODE_NAME)) > 1)
    {
        std::cerr << "[Error] Multiple unique node (" << GET_NODE_NAME(NODE_NAME) << ") cannot be activated" << std::endl;
        std_msgs::msg::String _msg; _msg.data = this->get_name() + std::string(" / ") + GET_NODE_NAME(NODE_NAME) + std::string(" is already activated");
        this->pub_except_error_str_add_->publish(_msg);
        exit(-1);
    }
    this->unique_ok_count_++;
    if (this->unique_ok_count_ > 7)
    {
        this->unique_check_timer_->cancel();
        this->unique_check_timer_ = nullptr;
    }
}
#endif

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<_Node>());
    rclcpp::shutdown();
    printf("EOF");
    return 0;
}
