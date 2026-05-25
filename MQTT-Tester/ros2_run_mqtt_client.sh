#/bin/bash

export ROS_LOCALHOST_ONLY=$(cat /params/ROS_LOCALHOST_ONLY)
export ROS_DOMAIN_ID=$(cat /params/ROS_DOMAIN_ID)
export ROS_PREFIX=$(cat /params/ROS_PREFIX)
export MQTT_PREFIX=$(cat /params/ROS_PREFIX)

PARAM_FILE=./params/params.ros2.yaml
TEMP_FILE=./params/tmp_params.ros2.yaml
TEMP_FILE_INJECTED=./params/tmp_params.injected.ros2.yaml
MQTT_CLIENT_BUFFER_SIZE=${MQTT_CLIENT_BUFFER_SIZE:-2000}

cp ${PARAM_FILE} ${TEMP_FILE}
ros_prefix=${ROS_PREFIX}
mqtt_prefix=${MQTT_PREFIX}
if [ ${#ros_prefix} -gt 0 ]; then
    if [[ "${ros_prefix}" != /* ]]; then
        ros_prefix="/${ros_prefix}"
    fi
fi
if [ ${#mqtt_prefix} -gt 0 ]; then
    if [[ "${mqtt_prefix}" != */ ]]; then
        mqtt_prefix="${mqtt_prefix}/"
    fi
fi
sed -i "s@/ROS_PREFIX@${ros_prefix}@g" ${TEMP_FILE}
sed -i "s@MQTT_PREFIX/@${mqtt_prefix}@g" ${TEMP_FILE}

mqtt_client_uuid=$(tr -d '-' < /proc/sys/kernel/random/uuid)
mqtt_client_id="triorb_mqtt_${mqtt_client_uuid}"
mqtt_client_buffer_dir="/tmp/mqtt_client/${mqtt_client_id}"
mkdir -p "${mqtt_client_buffer_dir}"

awk -v id="${mqtt_client_id}" \
    -v size="${MQTT_CLIENT_BUFFER_SIZE}" \
    -v dir="${mqtt_client_buffer_dir}" '
    /^    bridge:/ {
        print "    client:"
        print "      id: " id
        print "      clean_session: false"
        print "      buffer:"
        print "        size: " size
        print "        directory: " dir
    }
    { print }
' "${TEMP_FILE}" > "${TEMP_FILE_INJECTED}"
mv "${TEMP_FILE_INJECTED}" "${TEMP_FILE}"

. /install/${ROS_DISTRO}/setup.sh
ros2 launch mqtt_client standalone.launch.ros2.xml params_file:=${TEMP_FILE}
