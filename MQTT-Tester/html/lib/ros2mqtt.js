/*
 * Copyright 2023 TriOrb
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

class ROS2MQTT_ROS {
    constructor(kwargs = {}) {
        this.mqttUrl = kwargs.mqttUrl || 'ws://localhost:8083/mqtt';
        this.clientId = kwargs.clientId || 'mqttjs_' + Math.random().toString(16).substr(2, 8);
        this.options = kwargs.options || {
            keepalive: 0,
            clientId: this.clientId,
            protocolId: 'MQTT',
            protocolVersion: 5,
            clean: true,
            reconnectPeriod: 1000,
            connectTimeout: 30 * 1000,
            will: {
                topic: 'WillMsg',
                payload: 'Connection Closed abnormally..!',
                qos: 0,
                retain: false
            },
        }
        this.mqttState = kwargs.mqttState || null;
        console.log("MQTT State: ", this.mqttState);
        this.subscribers = {};
        this.mqttClient = null;
        this.connectToMQTT();
    }

    connectToMQTT() {
        let state_msg = `Connecting mqtt client to: ${this.mqttUrl}`;
        console.log(state_msg);
        if (this.mqttState) {
            this.mqttState.innerHTML = state_msg;
        }
        try {
            this.mqttClient = mqtt.connect(this.mqttUrl, this.options);
            this.mqttClient.on('connect', () => { state_msg = 'Connected to MQTT'; console.log(state_msg); if (this.mqttState) { this.mqttState.innerHTML = state_msg; } });
            this.mqttClient.on('close', () => { state_msg = 'MQTT connection closed'; console.log(state_msg); if (this.mqttState) { this.mqttState.innerHTML = state_msg; } });
            this.mqttClient.on('reconnect', () => { state_msg = 'Reconnecting to MQTT...'; console.log(state_msg); if (this.mqttState) { this.mqttState.innerHTML = state_msg; } });
            this.mqttClient.on('offline', () => { state_msg = 'MQTT is offline'; console.log(state_msg); if (this.mqttState) { this.mqttState.innerHTML = state_msg; } });
            this.mqttClient.on('message', (topic, message, packet) => {
                //console.debug('Received message:', topic, message.toString());
                if(this.subscribers[topic]){
                    message = message.toString();
                    // メッセージタイプを判定
                    // 1. '{'で始まり'}'で終わる場合はJSONとみなす
                    if (message.startsWith('{') && message.endsWith('}')) {
                        try {
                            message = JSON.parse(message);
                        } catch (e) {
                            console.error('Failed to parse message:', e);
                        }
                    }
                    // 2. '['で始まり']'で終わる場合は配列とみなす
                    else if (message.startsWith('[') && message.endsWith(']')) {
                        try {
                            message = JSON.parse(message);
                        } catch (e) {
                            console.error('Failed to parse message:', e);
                        }
                    }
                    // 3. 数値の場合は数値型に変換
                    else if (!isNaN(message)) {
                        message = { data: parseFloat(message) };
                    }
                    // 4. それ以外は文字列型として扱う
                    else {
                        message = { data: message };
                    }
                    //console.debug(message);

                    // コールバック関数を呼び出す
                    for (let i = 0; i< this.subscribers[topic].length; i++) {
                        this.subscribers[topic][i](message);
                    }
                }
            });
            this.mqttClient.on('error', (error) => {
                state_msg = `MQTT connection error: ${error}`;
                console.error(state_msg);
                if (this.mqttState) { this.mqttState.innerHTML = state_msg; }
                /*
                if (this.mqttClient) {
                    this.mqttClient.end(true, () => {
                        setTimeout(() => {
                            console.log('MQTT client connection ended. Reconnecting...');
                            this.connectToMQTT();
                        }, 1000);
                    });
                } 
                */
            });
            this.mqttClient.on('disconnect', () => {
                state_msg = 'MQTT disconnected';
                console.log(state_msg);
                if (this.mqttState) { this.mqttState.innerHTML = state_msg; }
            });
        } catch (error) {
            console.error(`Failed to connect to MQTT: ${error}`);
        }
    }

    publish(topic, type, message) {
        if (!this.mqttClient) {
            console.error('MQTT client is not connected');
            return;
        }
        if (type == 'std_msgs/msg/Bool') {
            if (typeof message == "object" && message.hasOwnProperty('data')) {
                console.log('message.data', message.data);
                message = message.data ? 1 : 0;
            }else if (typeof message == 'string') {
                message = message.toLowerCase() === 'true' ? 1 : 0;
            }
            else if (typeof message == 'boolean') {
                message = message ? 1 : 0;
            }else{
                message = message;
            }
        }
        if (!(typeof message == 'string')) {
            if (Object.keys(message).length == 1 && message.hasOwnProperty('data')) {
                message = message.data;
            } else {
                message = JSON.stringify(message);
            }
        }
        if (typeof message == 'object') {
            message = JSON.stringify(message);
        }
        if (typeof message == 'number') {
            message = "" + message;
        }

        // console.log('Publishing message to topic:', topic, 'with type:', type, 'and message:', message);
        this.mqttClient.publish(topic, message);
        //console.debug('Publishing message to topic:', topic, 'with type:', type);
    }

    subscribe(topic, callback) {
        if (!(topic in this.subscribers)){
            this.subscribers[topic] = [];
            this.mqttClient.subscribe(topic, (err) => {
                if (err) {
                    console.error('Failed to subscribe to topic:', topic, err);
                } else {
                    console.log('Add subscribed to topic:', topic);
                }
            });
        }
        this.subscribers[topic].push(callback);
    }

    unsubscribe(topic, callback) {
        if (topic in this.subscribers) {
            const index = this.subscribers[topic].indexOf(callback);
            if (index !== -1) {
                this.subscribers[topic].splice(index, 1);
                if (this.subscribers[topic].length === 0) {
                    delete this.subscribers[topic];
                    this.mqttClient.unsubscribe(topic, (err) => {
                        if (err) {
                            console.error('Failed to unsubscribe from topic:', topic, err);
                        } else {
                            console.log('Unsubscribed from topic:', topic);
                        }
                    });
                }
            }
        }
    }
    
}

class ROS2MQTT_Topic {
    constructor(params={ros:null, name:null, messageType:null}) {
        this.ros2mqtt = params.ros;
        this.name = params.name.startsWith('/') ? params.name.substring(1) : params.name;
        this.type = params.messageType;
    }

    subscribe(callback) {
        this.ros2mqtt.subscribe(this.name, callback);
    }

    unsubscribe(callback) {
        this.ros2mqtt.unsubscribe(this.name, callback);
    }

    publish(message = {}) {
        this.ros2mqtt.publish(this.name, this.type, message);
    }
}

window.ROS2MQTT = window.ROS2MQTT || {};
window.ROS2MQTT.Ros = ROS2MQTT_ROS;
window.ROS2MQTT.Topic = ROS2MQTT_Topic;