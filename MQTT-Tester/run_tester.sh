#!/bin/bash

## 起動中のコンテナは全て一旦停止
docker stop -t 1 gui vslam except_handle tester >> /dev/null
    
# MQTTブローカー & ROS2ブリッジ起動
bash -c 'sh ./run_mqtt2ros.sh'

docker run -it --rm -d --name tester --shm-size=1gb --privileged --net=host --ipc=host $(cat /triorb/params/DOCKER_USE_GPU) \
    --add-host=localhost:127.0.1.1 \
    -e ROS_LOCALHOST_ONLY=$(cat /triorb/params/ROS_LOCALHOST_ONLY) \
    -e ROS_DOMAIN_ID=$(cat /triorb/params/ROS_DOMAIN_ID) \
    -e ROS_PREFIX=$(cat /triorb/params/ROS_PREFIX) \
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
    $(cat /triorb/params/DOCKER_IMAGE_ROS) /bin/bash -c '\
        cd src; \
        python node.py; \
        /bin/bash'

# --- GUI起動 ---
ROBOT_CTRL_GUI_DIR=./html
DOCKER_NAME_GUI=gui
    
# prefixの置換
sudo sed -i "s/\".*\"/\""$(cat /triorb/params/ROS_PREFIX)"\"/" ${ROBOT_CTRL_GUI_DIR}/prefix.js

docker run -it --rm -d --name ${DOCKER_NAME_GUI} --privileged \
            --net=host \
            --ipc=host \
            --add-host=localhost:127.0.1.1 \
            -e ROS_LOCALHOST_ONLY=$(cat /triorb/params/ROS_LOCALHOST_ONLY) \
            -e ROS_DOMAIN_ID=$(cat /triorb/params/ROS_DOMAIN_ID) \
            -e ROS_PREFIX=$(cat /triorb/params/ROS_PREFIX) \
            -v /triorb/install:/install \
            -v /triorb/params:/params \
            -v /triorb/data:/data \
            -v /triorb/data/map:/map \
            -v $(pwd):/ws \
            -w /ws \
            $(cat /triorb/params/DOCKER_IMAGE_ROS) /bin/bash -c 'cd html; python3 -m http.server 8080 --cgi'
