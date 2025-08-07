import yaml

def getTopicType(fname):
    with open(fname) as f:
        data = f.read()

    spl = data.split("\n")
    if spl[-1] == "":
        spl = spl[:-1] 
    
    return spl


def makeROS2MQTT(topic, topic_name):
    dic = {
        "mqtt_topic": "MQTT_PREFIX/" + topic_name,
        "primitive": True,
        "ros_type": topic,
        "advanced": {
            "ros": {
                "queue_size": 1,
                "qos":{
                    "reliability": "best_effort",
                    "durability": "volatile",
                }
            },
            "mqtt": {
                "qos": 0,
                "retained": False,
            }
        },
    }
    return dic

def makeMQTT2ROS(topic, topic_name):
    dic = {
        "ros_topic": "/ROS_PREFIX/" + topic_name,
        "primitive": True,
        "ros_type": topic,
        "advanced": {
            "mqtt": {
                "qos": 2,
            },
            "ros": {
                "qos": {
                    "reliability": "reliable",
                    "durability": "volatile",
                }
            }
        },
    }
    return dic


def topicToYaml(topics):
    ros_topics = []
    mqtt_topics = []
    bridge = {
        "ros2mqtt": { "ros_topics": ros_topics },
        "mqtt2ros": {"mqtt_topics": mqtt_topics}
    }
    yaml_dict = {
        "/**/*": {
            "ros__parameters": {
                "broker": {
                    "host": "localhost",
                    "port": 1883,
                },
                "bridge": bridge
            }
        }
    }

    ### Generate topic_definition.js
    topic_definition_str = ""

    ### Generate subpub.js
    subpub_str = ""

    for topic in topics:
        topic_name = topic.split("/")[-1]
        to_topic_name   =   "to/" + topic_name
        from_topic_name = "from/" + topic_name

        ros_topics.append(  "/ROS_PREFIX/" + to_topic_name)
        mqtt_topics.append("MQTT_PREFIX/" + from_topic_name)

        bridge["ros2mqtt"]["/ROS_PREFIX/" +   to_topic_name] = makeROS2MQTT(topic, to_topic_name)
        bridge["mqtt2ros"]["MQTT_PREFIX/" + from_topic_name] = makeMQTT2ROS(topic, from_topic_name)

        base_txt  = "const " + topic_name + "_sub = new ROS2MQTT.Topic({ ros: ros2mqtt, name: ros_topic_prefix + '/" +   to_topic_name + "', messageType: '" + topic + "'});\n"
        base_txt += "const " + topic_name + "_pub = new ROS2MQTT.Topic({ ros: ros2mqtt, name: ros_topic_prefix + '/" + from_topic_name + "', messageType: '" + topic + "'});\n"
        topic_definition_str += base_txt

        base_txt = topic_name + "_sub.subscribe(function(msg) {\n"
        base_txt +=             "    let elm = document.getElementById('mqtt-sub');\n"
        base_txt +=             "    elm.innerHTML = JSON.stringify(msg, null, \"\\t\");\n"
        base_txt +=             "    " + topic_name + "_pub.publish(msg);\n"
        base_txt +=             "});\n\n"
        subpub_str += base_txt

    print(yaml_dict)

    with open("params/params.ros2.yaml", "w") as f:
        yaml.dump(yaml_dict, f, default_flow_style=False, sort_keys=False)
    
    with open("html/topic_definition.js", "w") as f:
        f.write(topic_definition_str)

    with open("html/subpub.js", "w") as f:
        f.write(subpub_str)

if __name__ == "__main__":
    topics = getTopicType("topics.txt")
    topicToYaml(topics)

