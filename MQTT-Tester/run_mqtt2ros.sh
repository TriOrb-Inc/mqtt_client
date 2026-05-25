#!/bin/bash
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: TriOrb Inc.
#

sudo mkdir -p /triorb/mqtt/data > /dev/null 2>&1
sudo mkdir -p /triorb/mqtt/log > /dev/null 2>&1
sudo chmod 777 -R /triorb/mqtt

# Check if EMQX broker is running
# If not, start it
docker ps | grep emqx_local > /dev/null 2>&1
if [ $? -ne 0 ]; then
  echo "Starting EMQX broker..."
  docker run -it -d --rm --name emqx_local \
    -p 1883:1883 -p 8083:8083 \
    -p 8084:8084 -p 8883:8883 \
    -p 18083:18083 \
    -e EMQX_DASHBOARD__DEFAULT_USERNAME="triorb" \
    -e EMQX_DASHBOARD__DEFAULT_PASSWORD="triorb" \
    -e EMQX_LISTENER__WS__DEFAULT__IDLE_TIMEOUT=30s \
    -e EMQX_LISTENER__WS__DEFAULT__PING_INTERVAL=20s \
    -e EMQX_LISTENER__WS__DEFAULT__PING_TIMEOUT=10s \
    -e EMQX_LISTENER__WS__DEFAULT__MAX_CONN_RATE=1000 \
    -e EMQX_LISTENER__WS__DEFAULT__MAX_CONNECTIONS=100 \
    -e EMQX_MQTT__MAX_PACKET_SIZE=50MB \
    -v /triorb/mqtt/data:/opt/emqx/data \
    -v /triorb/mqtt/log:/opt/emqx/log \
    emqx/emqx:5.8
    
  sleep 5
else
  echo "EMQX broker is already running."
fi

# Check if MQTT client is running
# If not, start it
docker ps | grep mqtt-ros > /dev/null 2>&1
if [ $? -ne 0 ]; then
  echo "Starting MQTT client..."
  docker run -it --rm -d --name mqtt-ros --privileged --net=host --ipc=host \
          --add-host=localhost:127.0.1.1 \
          -e ROS_LOCALHOST_ONLY=$(cat /triorb/params/ROS_LOCALHOST_ONLY) \
          -e ROS_DOMAIN_ID=$(cat /triorb/params/ROS_DOMAIN_ID) \
          -e ROS_PREFIX=$(cat /triorb/params/ROS_PREFIX) \
          -e MQTT_PREFIX=$(cat /triorb/params/ROS_PREFIX) \
          -v /dev:/dev \
          -v /sys/devices/:/sys/devices/ \
          -v /triorb/log:/log \
          -v /triorb/install:/install \
          -v /triorb/params:/params \
          -v /triorb/data:/data \
          -v /triorb/data/map:/map \
          -v /etc/NetworkManager/system-connections:/etc/NetworkManager/system-connections \
          -v /var/run/dbus:/var/run/dbus \
          -v "$(pwd)":/ws \
          -w /ws \
          $(cat /triorb/params/DOCKER_IMAGE_ROS) /bin/bash -c '
          export PATH=/install/vslam:$PATH &&\
          export LD_LIBRARY_PATH=/install/vslam/lib:$LD_LIBRARY_PATH &&\
          export PATH=/install/cuda_efficient_features:$PATH &&\
          export LD_LIBRARY_PATH=/install/cuda_efficient_features/lib:$LD_LIBRARY_PATH &&\
          export LD_LIBRARY_PATH=/tmp/ros/install/cv_bridge/lib:$LD_LIBRARY_PATH &&\
          tmux new-session -s mqtt -d "while true; do . ./ros2_run_mqtt_client.sh; sleep 0.2; done" &&\
          /bin/bash'
else
  echo "Restarting MQTT client..."
  docker exec mqtt-ros /bin/bash -c "tmux send-keys -t mqtt C-c"
fi