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

string ros_topic                            # ROS topic
string mqtt_topic                           # MQTT topic
string ros_msg_type std_msgs/msg/String     # message type of subscriber
uint16 ros_queue_size 1                     # ROS subscriber queue size
string ros_qos_reliability system_default   # system_default / best_effort / reliable / auto
string ros_qos_durability system_default    # system_default / volatile / transient_local / auto
int8 mqtt_qos 0                             # MQTT QoS value
bool mqtt_retained false                    # whether to retain MQTT message
bool fixed_type false                       # whether the published message type is specified explicitly
bool primitive false                        # whether to publish as primitive message
bool stamped false                          # whether to inject timestamp in MQTT message