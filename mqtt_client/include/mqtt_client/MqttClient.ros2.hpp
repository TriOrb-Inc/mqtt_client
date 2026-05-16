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


#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <string>

#define FMT_HEADER_ONLY
#include <fmt/format.h>
#include <mqtt/async_client.h>
#include <mqtt_client_interfaces/srv/is_connected.hpp>
#include <mqtt_client_interfaces/srv/new_mqtt2_ros_bridge.hpp>
#include <mqtt_client_interfaces/srv/new_ros2_mqtt_bridge.hpp>
#include <mqtt_client_interfaces/msg/ros2_mqtt_interface.hpp>
#include <mqtt_client_interfaces/msg/mqtt2_ros_interface.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

// ループバック防止
#define MQTT_CLIENT_LOOPBACK_PROTECTION_MS 90
#include <unordered_map>
#include <mutex>
#include <chrono>
// ガード用のキャッシュ
std::unordered_map<std::string, std::pair<std::string, std::chrono::steady_clock::time_point>> recent_ros_msgs_;
std::mutex recent_ros_msgs_mutex_;


/**
 * @brief Namespace for the mqtt_client package
 */
namespace mqtt_client {


/**
 * @brief ROS Nodelet for sending and receiving ROS messages via MQTT
 *
 * The MqttClient enables connected ROS-based devices or robots to
 * exchange ROS messages via an MQTT broker using the MQTT protocol.
 * This works generically for any ROS message, i.e. there is no need
 * to specify the ROS message type for ROS messages you wish to
 * exchange via the MQTT broker.
 */
class MqttClient : public rclcpp::Node,
                   public virtual mqtt::callback,
                   public virtual mqtt::iaction_listener {

 public:
  /**
   * @brief Destructor closes optional QUIC resources loaded outside Paho.
   */
  ~MqttClient() override;

  /**
   * @brief Initializes node.
   *
   * @param[in]   options   ROS node options
   */
  explicit MqttClient(const rclcpp::NodeOptions& options);

 protected:
   struct Ros2MqttInterface;
   struct Mqtt2RosInterface;

  /**
   * @brief Loads ROS parameters from parameter server.
   */
  void loadParameters();

  /**
   * @brief Loads requested ROS parameter from parameter server.
   *
   * @param[in]   key      parameter name
   * @param[out]  value    variable where to store the retrieved parameter
   *
   * @return  true         if parameter was successfully retrieved
   * @return  false        if parameter was not found
   */
  bool loadParameter(const std::string& key, std::string& value);

  /**
   * @brief Loads requested ROS parameter from parameter server, allows default
   * value.
   *
   * @param[in]   key            parameter name
   * @param[out]  value          variable where to store the retrieved parameter
   * @param[in]   default_value  default value
   *
   * @return  true         if parameter was successfully retrieved
   * @return  false        if parameter was not found or default was used
   */
  bool loadParameter(const std::string& key, std::string& value, const std::string& default_value);

  /**
   * @brief Loads requested ROS parameter from parameter server.
   *
   * @tparam  T            type (one of int, double, bool)
   *
   * @param[in]   key      parameter name
   * @param[out]  value    variable where to store the retrieved parameter
   *
   * @return  true         if parameter was successfully retrieved
   * @return  false        if parameter was not found
   */
  template <typename T>
  bool loadParameter(const std::string& key, T& value);

  /**
   * @brief Loads requested ROS parameter from parameter server, allows default
   * value.
   *
   * @tparam  T            type (one of int, double, bool)
   *
   * @param[in]   key            parameter name
   * @param[out]  value          variable where to store the retrieved parameter
   * @param[in]   default_value  default value
   *
   * @return  true         if parameter was successfully retrieved
   * @return  false        if parameter was not found or default was used
   */
  template <typename T>
  bool loadParameter(const std::string& key, T& value, const T& default_value);

  /**
   * @brief Loads requested ROS parameter from parameter server.
   *
   * @tparam  T            type (one of int, double, bool)
   *
   * @param[in]   key      parameter name
   * @param[out]  value    variable where to store the retrieved parameter
   *
   * @return  true         if parameter was successfully retrieved
   * @return  false        if parameter was not found
   */
  template <typename T>
  bool loadParameter(const std::string& key, std::vector<T>& value);

  /**
   * @brief Loads requested ROS parameter from parameter server, allows default
   * value.
   *
   * @tparam  T            type (one of int, double, bool)
   *
   * @param[in]   key            parameter name
   * @param[out]  value          variable where to store the retrieved parameter
   * @param[in]   default_value  default value
   *
   * @return  true         if parameter was successfully retrieved
   * @return  false        if parameter was not found or default was used
   */
  template <typename T>
  bool loadParameter(const std::string& key, std::vector<T>& value, const std::vector<T>& default_value);

  /**
   * @brief Converts a string to a path object resolving paths relative to
   * ROS_HOME.
   *
   * Resolves relative to CWD, if ROS_HOME is not set.
   * Returns empty path, if argument is empty.
   *
   * @param   path_string  (relative) path as string
   *
   * @return  std::filesystem::path  path variable
   */
  std::filesystem::path resolvePath(const std::string& path_string,
                                    bool warn_if_missing = true);

  /**
   * @brief Initializes broker connection and subscriptions.
   */
  void setup();

  /**
   * @brief Get the resolved compatible QOS from the interface and the endpoint
   *
   * This uses the two endpoints to decide upon a compatible QoS, resolving any "auto" QoS settings
   *
   * @param ros_topic the ROS topic we are looking on
   * @param tei Topic endpoint info
   * @param ros2mqtt the ROS to MQTT interface spec
   *
   * @returns The compatible QoS or nullopt if no compatible combination is found
   */
   std::optional<rclcpp::QoS> getCompatibleQoS(
     const std::string& ros_topic, const rclcpp::TopicEndpointInfo& tei,
     const Ros2MqttInterface& ros2mqtt) const;

  /**
   * @brief Get the candiate topic endpoints for subscription matching
   *
   * @param ros2mqtt the ROS to MQTT interface spec
   *
   * @returns The compatible QoS or nullopt if no compatible combination is found
   */
   std::vector<rclcpp::TopicEndpointInfo> getCandidatePublishers(
     const std::string& ros_topic, const Ros2MqttInterface& ros2mqtt) const;

  /**
   * @brief Setup any subscriptions we can.
   *
   * These may be fixed type/QoS, or dynamically matched against active publisher
   */
  void setupSubscriptions();

  /**
   * @brief Setup any publishers that we can
   */
  void setupPublishers();

  /**
   * @brief Sets up the client connection options and initializes the client
   * object.
   */
  void setupClient();

  /**
   * @brief Connects to the broker using the member client and options.
   */
  void connect();

  /**
   * @brief Records an MQTT health issue that should trigger restart if it persists.
   */
  void markMqttUnhealthy(const std::string& reason);

  /**
   * @brief Clears the MQTT health issue state.
   */
  void clearMqttUnhealthy();

  /**
   * @brief Returns whether an MQTT return code should be watched by the watchdog.
   */
  bool shouldWatchdogMqttRc(int rc) const;

  /**
   * @brief Returns the number of pending MQTT delivery tokens.
   */
  size_t pendingDeliveryTokenCount() const;

  /**
   * @brief Restarts the process if MQTT stays unhealthy for too long.
   */
  void mqttRecoveryWatchdog();

  /**
   * @brief Returns true when the bridge should use the direct QUIC backend.
   */
  bool usingQuicTransport() const;

  /**
   * @brief Human-readable broker URI for the active MQTT transport.
   */
  std::string mqttServerUri() const;

  /**
   * @brief Returns connection state for the active MQTT transport.
   */
  bool isMqttTransportConnected();

  /**
   * @brief Publishes one MQTT payload using the active MQTT transport.
   */
  void publishMqtt(const std::string& topic, const void* payload,
                   size_t payload_size, int qos, bool retained);

  /**
   * @brief Subscribes one MQTT topic using the active MQTT transport.
   */
  void subscribeMqtt(const std::string& topic, int qos);

  /**
   * @brief Common connected handler used by Paho and QUIC backends.
   */
  void handleMqttConnected();

  /**
   * @brief Common disconnected handler used by Paho and QUIC backends.
   */
  void handleMqttDisconnected(const std::string& cause,
                              bool request_reconnect);

  /**
   * @brief Publishes a generic serialized ROS message to the MQTT broker.
   *
   * Before publishing the ROS message to the MQTT broker, the ROS message type
   * is extracted. This type information is also sent to the MQTT broker on a
   * separate topic.
   *
   * The MQTT payload for the actual ROS message carries the following:
   * - 0 or 1 (indicating if timestamp is injected (=1))
   * - serialized timestamp (optional)
   * - serialized ROS message
   *
   * @param   serialized_msg  generic serialized ROS message
   * @param   ros_topic       ROS topic where the message was published
   */
  void ros2mqtt(
    const std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg,
    const std::string& ros_topic);

  /**
   * @brief Publishes a ROS message received via MQTT to ROS.
   *
   * This utilizes the generic publisher stored for the MQTT topic on which the
   * message was received. The publisher has to be configured to the ROS message
   * type of the message. If the message carries an injected timestamp, the
   * latency is computed and published.
   *
   * The MQTT payload is expected to carry the following:
   * - 0 or 1 (indicating if timestamp is injected (=1))
   * - serialized timestamp (optional)
   * - serialized ROS message
   *
   * @param   mqtt_msg       MQTT message
   * @param   arrival_stamp  arrival timestamp used for latency computation
   */
  void mqtt2ros(mqtt::const_message_ptr mqtt_msg,
                const rclcpp::Time& arrival_stamp);

  /**
   * @brief Publishes a primitive message received via MQTT to ROS.
   *
   * @param   mqtt_msg     MQTT message
   */
  void mqtt2primitive(mqtt::const_message_ptr mqtt_msg);

  /**
   * @brief Publishes a primitive message received via MQTT to ROS.
   *
   * @param   mqtt_msg     MQTT message
   */
  void mqtt2fixed(mqtt::const_message_ptr mqtt_msg);

  /**
   * @brief Callback for when the client has successfully connected to the
   * broker.
   *
   * Overrides mqtt::callback::connected(const std::string&).
   *
   * @param   cause
   */
  void connected(const std::string& cause) override;

  /**
   * @brief Callback for when the client has lost connection to the broker.
   *
   * Overrides mqtt::callback::connection_lost(const std::string&).
   *
   * @param   cause
   */
  void connection_lost(const std::string& cause) override;

  /**
   * @brief Returns whether the client is connected to the broker.
   *
   * @return true if client is connected to the broker
   * @return false if client is not connected to the broker
   */
  bool isConnected();

  /**
   * @brief ROS service returning whether the client is connected to the broker.
   *
   * @param request  service request
   * @param response service response
   */
  void isConnectedService(
    mqtt_client_interfaces::srv::IsConnected::Request::SharedPtr request,
    mqtt_client_interfaces::srv::IsConnected::Response::SharedPtr response);

  /**
   * @brief ROS service that dynamically creates a ROS -> MQTT mapping.
   *
   * @param request  service request
   * @param response service response
   */
  void newRos2MqttBridge(
    mqtt_client_interfaces::srv::NewRos2MqttBridge::Request::SharedPtr request,
    mqtt_client_interfaces::srv::NewRos2MqttBridge::Response::SharedPtr response);

  /**
   * @brief ROS service that dynamically creates an MQTT -> ROS mapping.
   *
   * @param request  service request
   * @param response service response
   */
  void newMqtt2RosBridge(
    mqtt_client_interfaces::srv::NewMqtt2RosBridge::Request::SharedPtr request,
    mqtt_client_interfaces::srv::NewMqtt2RosBridge::Response::SharedPtr response);

  /**
   * @brief ROS callback that dynamically creates an MQTT -> ROS mapping.
   */
  void callback_add_ros2mqtt(const mqtt_client_interfaces::msg::Ros2MqttInterface::SharedPtr msg);

  /**
   * @brief ROS callback that dynamically creates a ROS -> MQTT mapping.
   */
  void callback_add_mqtt2ros(const mqtt_client_interfaces::msg::Mqtt2RosInterface::SharedPtr msg);

  /**
   * @brief ROS callback that removes a ROS -> MQTT mapping.
   */
  void callback_remove_ros2mqtt(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief ROS callback that removes a MQTT -> ROS mapping.
   */
  void callback_remove_mqtt2ros(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Callback for when the client receives a MQTT message from the
   * broker.
   *
   * Overrides mqtt::callback::message_arrived(mqtt::const_message_ptr).
   * If the received MQTT message contains information about a ROS message type,
   * the corresponding ROS publisher is configured. If the received MQTT message
   * is a ROS message, the mqtt2ros conversion is called.
   *
   * @param   mqtt_msg     MQTT message
   */
  void message_arrived(mqtt::const_message_ptr mqtt_msg) override;

  /**
   * @brief Callback for when delivery for a MQTT message has been completed.
   *
   * Overrides mqtt::callback::delivery_complete(mqtt::delivery_token_ptr).
   *
   * @param   token        token tracking the message delivery
   */
  void delivery_complete(mqtt::delivery_token_ptr token) override;

  /**
   * @brief Callback for when a MQTT action succeeds.
   *
   * Overrides mqtt::iaction_listener::on_success(const mqtt::token&).
   * Does nothing.
   *
   * @param   token        token tracking the action
   */
  void on_success(const mqtt::token& token) override;

  /**
   * @brief Callback for when a MQTT action fails.
   *
   * Overrides mqtt::iaction_listener::on_failure(const mqtt::token&).
   * Logs error.
   *
   * @param   token        token tracking the action
   */
  void on_failure(const mqtt::token& token) override;

 protected:
  /**
   * @brief Minimal ABI-compatible handle for NanoSDK's nng_socket.
   */
  struct NngSocket {
    uint32_t id = 0;
  };

  /**
   * @brief Opaque NanoSDK MQTT message handle used through dlopen.
   */
  struct NngMsg;

  /**
   * @brief NanoSDK topic buffer layout used when composing SUBSCRIBE packets.
   */
  struct NngMqttTopic {
    uint32_t length = 0;
    uint8_t* buf = nullptr;
  };

  /**
   * @brief NanoSDK topic/QoS layout used when composing SUBSCRIBE packets.
   */
  struct NngMqttTopicQos {
    NngMqttTopic topic;
    uint8_t qos = 0;
    uint8_t nolocal = 0;
    uint8_t rap = 0;
    uint8_t retain_handling = 0;
  };

  /**
   * @brief NanoSDK QUIC TLS configuration layout passed to open_conf().
   */
  struct NngConfTls {
    bool enable = false;
    char* url = nullptr;
    char* cafile = nullptr;
    char* certfile = nullptr;
    char* keyfile = nullptr;
    char* ca = nullptr;
    char* cert = nullptr;
    char* key = nullptr;
    char* key_password = nullptr;
    bool verify_peer = false;
    bool set_fail = false;
  };

  /**
   * @brief NanoSDK QUIC configuration layout passed to open_conf().
   */
  struct NngConfQuic {
    NngConfTls tls;
    bool qos_first = true;
    bool multi_stream = false;
    uint64_t qkeepalive = 30;
    uint64_t qconnect_timeout = 60;
    uint32_t qdiscon_timeout = 30;
    uint32_t qidle_timeout = 30;
    uint8_t qcongestion_control = 0;
  };

  /**
   * @brief NanoSDK symbols loaded at runtime for optional QUIC transport.
   */
  struct NngQuicApi {
    void* handle = nullptr;
    int (*nng_mqtt_quic_client_open_conf)(NngSocket*, const char*, NngConfQuic*) = nullptr;
    int (*nng_mqtt_quic_set_connect_cb)(NngSocket*, int (*)(void*, void*), void*) = nullptr;
    int (*nng_mqtt_quic_set_disconnect_cb)(NngSocket*, int (*)(void*, void*), void*) = nullptr;
    int (*nng_mqtt_quic_set_msg_recv_cb)(NngSocket*, int (*)(void*, void*), void*) = nullptr;
    int (*nng_mqtt_msg_alloc)(NngMsg**, size_t) = nullptr;
    void (*nng_mqtt_msg_set_packet_type)(NngMsg*, int) = nullptr;
    void (*nng_mqtt_msg_set_connect_proto_version)(NngMsg*, uint8_t) = nullptr;
    void (*nng_mqtt_msg_set_connect_keep_alive)(NngMsg*, uint16_t) = nullptr;
    void (*nng_mqtt_msg_set_connect_client_id)(NngMsg*, const char*) = nullptr;
    void (*nng_mqtt_msg_set_connect_user_name)(NngMsg*, const char*) = nullptr;
    void (*nng_mqtt_msg_set_connect_password)(NngMsg*, const char*) = nullptr;
    void (*nng_mqtt_msg_set_connect_clean_session)(NngMsg*, bool) = nullptr;
    void (*nng_mqtt_msg_set_connect_will_topic)(NngMsg*, const char*) = nullptr;
    void (*nng_mqtt_msg_set_connect_will_msg)(NngMsg*, uint8_t*, uint32_t) = nullptr;
    void (*nng_mqtt_msg_set_connect_will_retain)(NngMsg*, bool) = nullptr;
    void (*nng_mqtt_msg_set_connect_will_qos)(NngMsg*, uint8_t) = nullptr;
    int (*nng_mqtt_msg_set_publish_topic)(NngMsg*, const char*) = nullptr;
    void (*nng_mqtt_msg_set_publish_payload)(NngMsg*, uint8_t*, uint32_t) = nullptr;
    void (*nng_mqtt_msg_set_publish_qos)(NngMsg*, uint8_t) = nullptr;
    void (*nng_mqtt_msg_set_publish_retain)(NngMsg*, bool) = nullptr;
    void (*nng_mqtt_msg_set_publish_dup)(NngMsg*, bool) = nullptr;
    const char* (*nng_mqtt_msg_get_publish_topic)(NngMsg*, uint32_t*) = nullptr;
    uint8_t* (*nng_mqtt_msg_get_publish_payload)(NngMsg*, uint32_t*) = nullptr;
    void (*nng_mqtt_msg_set_subscribe_topics)(NngMsg*, NngMqttTopicQos*, uint32_t) = nullptr;
    void (*nng_msg_free)(NngMsg*) = nullptr;
    int (*nng_sendmsg)(NngSocket, NngMsg*, int) = nullptr;
    int (*nng_close)(NngSocket) = nullptr;
    const char* (*nng_strerror)(int) = nullptr;
    int (*nng_socket_set_ptr)(NngSocket, const char*, void*) = nullptr;
    int (*nng_mqtt_alloc_sqlite_opt)(void**) = nullptr;
    int (*nng_mqtt_free_sqlite_opt)(void*) = nullptr;
    void (*nng_mqtt_set_sqlite_enable)(void*, bool) = nullptr;
    void (*nng_mqtt_set_sqlite_flush_threshold)(void*, size_t) = nullptr;
    void (*nng_mqtt_set_sqlite_max_rows)(void*, size_t) = nullptr;
    void (*nng_mqtt_set_sqlite_db_dir)(void*, const char*) = nullptr;
    void (*nng_mqtt_sqlite_db_init)(void*, const char*, uint8_t) = nullptr;
  };

  /**
   * @brief Loads NanoSDK/NNG symbols when QUIC transport is requested.
   */
  bool loadNngQuicApi();

  /**
   * @brief Initializes the optional NanoSDK persistent buffer.
   */
  void setupQuicPersistentBuffer();

  /**
   * @brief Opens the QUIC MQTT socket and registers callbacks.
   */
  void setupQuicClient();

  /**
   * @brief Sends the MQTT CONNECT packet through NanoSDK QUIC.
   */
  void connectQuic();

  /**
   * @brief Static NanoSDK callback wrappers.
   */
  static int quicConnectCallback(void* rmsg, void* arg);
  static int quicDisconnectCallback(void* rmsg, void* arg);
  static int quicMessageCallback(void* rmsg, void* arg);

  /**
   * @brief Handles one NanoSDK PUBLISH message.
   */
  void handleQuicMessage(void* rmsg);

  /**
   * @brief Struct containing broker parameters
   */
  struct BrokerConfig {
    std::string transport;  ///< mqtt transport: tcp, ssl, or quic
    std::string host;  ///< broker host
    int port;          ///< broker port
    std::string user;  ///< username
    std::string pass;  ///< password
    struct {
      bool enabled;                          ///< whether to connect via SSL/TLS
      std::filesystem::path ca_certificate;  ///< public CA certificate trusted by client
    } tls;                                   ///< SSL/TLS-related variables
    struct {
      std::string library;       ///< NanoSDK/NNG shared library path
      bool qos_first;            ///< prioritize QoS packets on QUIC
      bool multi_stream;         ///< use NanoSDK multi-stream mode
      int keep_alive_sec;        ///< QUIC keepalive timeout
      int connect_timeout_sec;   ///< QUIC handshake idle timeout
      int disconnect_timeout_sec;///< QUIC disconnect timeout
      int idle_timeout_sec;      ///< QUIC idle timeout
      int congestion_control;    ///< 0=cubic, 1=bbr in NanoSDK
      bool tls_enabled;          ///< enable explicit client TLS material
      bool verify_peer;          ///< verify broker certificate
      bool fail_if_no_peer_cert; ///< fail when peer cert is missing
      std::string ca_certificate;///< QUIC CA certificate path
      std::string certificate;   ///< QUIC client certificate path
      std::string key;           ///< QUIC client private key path
      std::string key_password;  ///< QUIC client key password
    } quic;                      ///< QUIC-related variables
  };

  /**
   * @brief Struct containing client parameters
   */
  struct ClientConfig {
    std::string id;  ///< client unique ID
    struct {
      bool enabled;                     ///< whether client buffer is enabled
      int size;                         ///< client buffer size
      std::filesystem::path directory;  ///< client buffer directory
    } buffer;                           ///< client buffer-related variables
    struct {
      std::string topic;         ///< last-will topic
      std::string message;       ///< last-will message
      int qos;                   ///< last-will QoS value
      bool retained;             ///< whether last-will is retained
    } last_will;                 ///< last-will-related variables
    bool clean_session;          ///< whether client requests clean session
    double keep_alive_interval;  ///< keep-alive interval
    int max_inflight;            ///< maximum number of inflight messages
    struct {
      std::filesystem::path certificate;     ///< client certificate
      std::filesystem::path key;             ///< client private keyfile
      std::string password;                  ///< decryption password for private key
      int version;                           ///< TLS version (https://github.com/eclipse/paho.mqtt.cpp/blob/master/src/mqtt/ssl_options.h#L305)
      bool verify;                           ///< Verify the client should conduct
                                             ///< post-connect checks
      std::vector<std::string> alpn_protos;  ///< list of ALPN protocols
    } tls;                                   ///< SSL/TLS-related variables
  };

  /**
   * @brief Struct containing variables related to a ROS2MQTT connection.
   */
  struct Ros2MqttInterface {
    struct {
      rclcpp::GenericSubscription::SharedPtr
        subscriber;          ///< generic ROS subscriber
      std::string msg_type;  ///< message type of subscriber
      int queue_size = 1;    ///< ROS subscriber queue size
      bool is_stale = false; ///< whether a new generic publisher/subscriber is required
      struct {
        // If these are set to nullopt then that part of the QoS is determine automatically based on discovery
        std::optional<rclcpp::ReliabilityPolicy> reliability;
        std::optional<rclcpp::DurabilityPolicy> durability;
      } qos;
    } ros;                   ///< ROS-related variables
    struct {
      std::string topic;      ///< MQTT topic
      int qos = 0;            ///< MQTT QoS value
      bool retained = false;  ///< whether to retain MQTT message
    } mqtt;                   ///< MQTT-related variables
    bool fixed_type = false;  ///< whether the published message type is specified explicitly
    bool primitive = false;   ///< whether to publish as primitive message
    bool stamped = false;     ///< whether to inject timestamp in MQTT message
  };

  /**
   * @brief Struct containing variables related to a MQTT2ROS connection.
   */
  struct Mqtt2RosInterface {
    struct {
      int qos = 0;      ///< MQTT QoS value
    } mqtt;             ///< MQTT-related variables
    struct {
      std::string topic;     ///< ROS topic
      std::string msg_type;  ///< message type of publisher
      rclcpp::GenericPublisher::SharedPtr publisher;  ///< generic ROS publisher
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        latency_publisher;   ///< ROS publisher for latency
      int queue_size = 1;    ///< ROS publisher queue size
      struct {
        rclcpp::ReliabilityPolicy reliability = rclcpp::ReliabilityPolicy::SystemDefault;
        rclcpp::DurabilityPolicy durability = rclcpp::DurabilityPolicy::SystemDefault;
      } qos;
      bool latched = false;  ///< whether to latch ROS message
      bool is_stale = false; ///< whether a new generic publisher/subscriber is required
    } ros;      ///< ROS-related variables
    bool fixed_type = false; ///< whether the published ros message type is specified explicitly
    bool primitive = false;  ///< whether to publish as primitive message (if
                             ///< coming from non-ROS MQTT client)
    bool stamped = false;    ///< whether timestamp is injected
  };

 protected:
  /**
   * @brief MQTT topic prefix under which ROS message type information is
   * published
   *
   * Must contain trailing '/'.
   */
  static const std::string kRosMsgTypeMqttTopicPrefix;

  /**
   * @brief ROS topic prefix under which ROS2MQTT2ROS latencies are published
   *
   * Must contain trailing '/'.
   */
  static const std::string kLatencyRosTopicPrefix;

  /**
   * @brief Timer to repeatedly check active ROS topics for topics to subscribe
   */
  rclcpp::TimerBase::SharedPtr check_subscriptions_timer_;

  /**
   * @brief Timer to restart the process when MQTT stays unhealthy.
   */
  rclcpp::TimerBase::SharedPtr mqtt_recovery_watchdog_timer_;

  /**
   * @brief ROS Service server for providing connection status
   */
  rclcpp::Service<mqtt_client_interfaces::srv::IsConnected>::SharedPtr
    is_connected_service_;

  /**
   * @brief ROS Service server for providing dynamic ROS to MQTT mappings.
   */
  rclcpp::Service<mqtt_client_interfaces::srv::NewRos2MqttBridge>::SharedPtr
    new_ros2mqtt_bridge_service_;

  /**
   * @brief ROS Service server for providing dynamic MQTT to ROS mappings.
   */
  rclcpp::Service<mqtt_client_interfaces::srv::NewMqtt2RosBridge>::SharedPtr
    new_mqtt2ros_bridge_service_;

    
  /**
   * @brief ROS Topic subscriber for providing dynamic ROS to MQTT mappings.
   */
  rclcpp::Subscription<mqtt_client_interfaces::msg::Ros2MqttInterface>::SharedPtr sub_add_ros2mqtt_;

  /**
   * @brief ROS Topic subscriber for providing dynamic MQTT to ROS mappings.
   */
  rclcpp::Subscription<mqtt_client_interfaces::msg::Mqtt2RosInterface>::SharedPtr sub_add_mqtt2ros_;

  /**
   * @brief ROS Topic subscriber for removing dynamic ROS to MQTT mappings.
   */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_remove_ros2mqtt_;

  /**
   * @brief ROS Topic subscriber for removing dynamic MQTT to ROS mappings.
   */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_remove_mqtt2ros_;

  /**
   * @brief ROS Topic publisher for the result of dynamic mappings.
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_edit_dynamic_mapping_result_;

  /**
   * @brief Status variable keeping track of connection status to broker
   */
  bool is_connected_ = false;

  /**
   * @brief Mutex protecting MQTT health state shared across ROS/MQTT threads.
   */
  mutable std::mutex mqtt_health_mutex_;

  /**
   * @brief Timestamp from when MQTT first entered an unhealthy state.
   */
  std::optional<std::chrono::steady_clock::time_point> mqtt_unhealthy_since_;

  /**
   * @brief Human-readable reason for the current unhealthy state.
   */
  std::string mqtt_unhealthy_reason_;

  /**
   * @brief Broker parameters
   */
  BrokerConfig broker_config_;

  /**
   * @brief Client parameters
   */
  ClientConfig client_config_;

  /**
   * @brief MQTT client variable
   */
  std::shared_ptr<mqtt::async_client> client_;

  /**
   * @brief Runtime-loaded NanoSDK/NNG QUIC symbols.
   */
  NngQuicApi nng_;

  /**
   * @brief NanoSDK QUIC socket and related state.
   */
  NngSocket quic_socket_;
  bool quic_socket_opened_ = false;
  std::string quic_uri_;
  void* quic_sqlite_option_ = nullptr;

  /**
   * @brief MQTT client connection options
   */
  mqtt::connect_options connect_options_;

  /**
   * @brief ROS2MQTT connection variables sorted by ROS topic
   */
  std::map<std::string, Ros2MqttInterface> ros2mqtt_;

  /**
   * @brief MQTT2ROS connection variables sorted by MQTT topic
   */
  std::map<std::string, Mqtt2RosInterface> mqtt2ros_;

  /**
   * Message length of a serialized `builtin_interfaces::msg::Time` message
   */
  uint32_t stamp_length_;

  /**
   * @brief Prefix for MQTT topics
   */
  std::string topic_prefix_mqtt_;

  /**
   * @brief Prefix for ROS topics
   */
  std::string topic_prefix_ros_;
  
#ifdef HAVE_TRIORB_INTERFACE
  /**
   * @brief TriOrb ExceptHandler
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_except_node_registration_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_except_error_str_add_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_except_warn_str_add_;
#endif // HAVE_TRIORB_INTERFACE
};


template <typename T>
bool MqttClient::loadParameter(const std::string& key, T& value) {
  rclcpp::Parameter param;
  bool found = false;
  try {
    found = get_parameter(key, param);
  } catch (const rclcpp::exceptions::InvalidParameterValueException&) {
    return false;
  }
  if (!found || param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    return false;
  try {
    found = get_parameter(key, value);
  } catch (const rclcpp::exceptions::InvalidParameterValueException&) {
    return false;
  }
  if (found)
    RCLCPP_DEBUG(get_logger(), "Retrieved parameter '%s' = '%s'", key.c_str(),
                 std::to_string(value).c_str());
  return found;
}


template <typename T>
bool MqttClient::loadParameter(const std::string& key, T& value,
                               const T& default_value) {
  rclcpp::Parameter param;
  bool found = false;
  try {
    found = get_parameter(key, param);
  } catch (const rclcpp::exceptions::InvalidParameterValueException&) {
    found = false;
  }
  if (found && param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    try {
      found = get_parameter(key, value);
    } catch (const rclcpp::exceptions::InvalidParameterValueException&) {
      value = default_value;
      found = false;
    }
  } else {
    value = default_value;
    found = false;
  }
  if (!found)
    RCLCPP_WARN(get_logger(), "Parameter '%s' not set, defaulting to '%s'",
                key.c_str(), std::to_string(default_value).c_str());
  if (found)
    RCLCPP_DEBUG(get_logger(), "Retrieved parameter '%s' = '%s'", key.c_str(),
                 std::to_string(value).c_str());
  return found;
}


template <typename T>
bool MqttClient::loadParameter(const std::string& key, std::vector<T>& value) {
  rclcpp::Parameter param;
  bool found = false;
  try {
    found = get_parameter(key, param);
  } catch (const rclcpp::exceptions::InvalidParameterValueException&) {
    return false;
  }
  if (!found || param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    return false;
  try {
    found = get_parameter(key, value);
  } catch (const rclcpp::exceptions::InvalidParameterValueException&) {
    return false;
  }
  if (found)
    RCLCPP_WARN(get_logger(), "Retrieved parameter '%s' = '[%s]'", key.c_str(),
                fmt::format("{}", fmt::join(value, ", ")).c_str());
  return found;
}


template <typename T>
bool MqttClient::loadParameter(const std::string& key, std::vector<T>& value,
                               const std::vector<T>& default_value) {
  rclcpp::Parameter param;
  bool found = false;
  try {
    found = get_parameter(key, param);
  } catch (const rclcpp::exceptions::InvalidParameterValueException&) {
    found = false;
  }
  if (found && param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    try {
      found = get_parameter(key, value);
    } catch (const rclcpp::exceptions::InvalidParameterValueException&) {
      value = default_value;
      found = false;
    }
  } else {
    value = default_value;
    found = false;
  }
  if (!found)
    RCLCPP_WARN(get_logger(), "Parameter '%s' not set, defaulting to '%s'",
                key.c_str(), fmt::format("{}", fmt::join(value, ", ")).c_str());
  if (found)
    RCLCPP_DEBUG(get_logger(), "Retrieved parameter '%s' = '%s'", key.c_str(),
                 fmt::format("{}", fmt::join(value, ", ")).c_str());
  return found;
}


/**
 * Serializes a ROS message.
 *
 * @tparam  T                    ROS message type
 *
 * @param[in]   msg              ROS message
 * @param[out]  serialized_msg   serialized message
 */
template <typename T>
void serializeRosMessage(const T& msg,
                         rclcpp::SerializedMessage& serialized_msg) {

  rclcpp::Serialization<T> serializer;
  serializer.serialize_message(&msg, &serialized_msg);
}


/**
 * Deserializes a ROS message.
 *
 * @tparam  T                   ROS message type
 *
 * @param[in]   serialized_msg  serialized message
 * @param[out]  msg             ROS message
 */
template <typename T>
void deserializeRosMessage(const rclcpp::SerializedMessage& serialized_msg,
                           T& msg) {

  rclcpp::Serialization<T> serializer;
  serializer.deserialize_message(&serialized_msg, &msg);
}

}  // namespace mqtt_client
