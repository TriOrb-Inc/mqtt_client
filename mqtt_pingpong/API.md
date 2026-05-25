# mqtt_pingpong

ROS2 通信疎通を確認する Ping/Pong ノード。`ROS_PREFIX` と `PING_PREFIX` で
トピックプレフィクスを調整できます。

## Active API

### ROS2 Ping受信
- Topic：(prefix)(ping_prefix)/ros2/ping
- Node：(prefix)_mqtt_pingpong
- Type： std_msgs/msg/String
- Note：SensorDataQoS で購読し、受信データをそのまま Pong に転送します
- Usage：
```
ros2 topic pub /ros2/ping std_msgs/msg/String "{data: 'alive?'}" --once
```

### ROS2 Pong応答
- Topic：(prefix)(ping_prefix)/ros2/pong
- Node：(prefix)_mqtt_pingpong
- Type： std_msgs/msg/String
- Note：Ping を受信すると同じペイロードで即時 Publish されます
- Usage：
```
ros2 topic echo /ros2/pong
```
