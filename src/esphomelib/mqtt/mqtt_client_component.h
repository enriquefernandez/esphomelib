#ifndef ESPHOMELIB_MQTT_MQTT_CLIENT_COMPONENT_H
#define ESPHOMELIB_MQTT_MQTT_CLIENT_COMPONENT_H

#include <string>
#include <functional>
#include <vector>
#include <ArduinoJson.h>
#include <AsyncMqttClient.h>

#include "esphomelib/component.h"
#include "esphomelib/helpers.h"
#include "esphomelib/automation.h"
#include "esphomelib/log.h"
#include "esphomelib/defines.h"

ESPHOMELIB_NAMESPACE_BEGIN

namespace mqtt {

/** Callback for MQTT subscriptions.
 *
 * First parameter is the topic, the second one is the payload.
 */
using mqtt_callback_t = std::function<void(const std::string &, const std::string &)>;
using mqtt_json_callback_t = std::function<void(const std::string &, JsonObject &)>;

/// internal struct for MQTT messages.
struct MQTTMessage {
  std::string topic;
  std::string payload;
  uint8_t qos; ///< QoS. Only for last will testaments.
  bool retain;
};

/// internal struct for MQTT subscriptions.
struct MQTTSubscription {
  std::string topic;
  uint8_t qos;
  mqtt_callback_t callback;
};

/// internal struct for MQTT credentials.
struct MQTTCredentials {
  std::string address; ///< The address of the server without port number
  uint16_t port; ///< The port number of the server.
  std::string username;
  std::string password;
  std::string client_id; ///< The client ID. Will automatically be truncated to 23 characters.
};

/// Simple data struct for Home Assistant component availability.
struct Availability {
  std::string topic; ///< Empty means disabled
  std::string payload_available;
  std::string payload_not_available;
};

class MQTTMessageTrigger;
class MQTTJsonMessageTrigger;

template<typename T>
class MQTTPublishAction;

template<typename T>
class MQTTPublishJsonAction;

/** Internal struct for MQTT Home Assistant discovery
 *
 * See <a href="https://home-assistant.io/docs/mqtt/discovery/">MQTT Discovery</a>.
 */
struct MQTTDiscoveryInfo {
  std::string prefix; ///< The Home Assistant discovery prefix. Empty means disabled.
  bool retain; ///< Whether to retain discovery messages.
};

enum MQTTClientState {
  MQTT_CLIENT_DISCONNECTED = 0,
  MQTT_CLIENT_CONNECTING,
  MQTT_CLIENT_CONNECTED,
};

class MQTTComponent;

class MQTTClientComponent : public Component {
 public:
  explicit MQTTClientComponent(const MQTTCredentials &credentials, const std::string &topic_prefix);

  /// Set the last will testament message.
  void set_last_will(MQTTMessage &&message);
  /// Remove the last will testament message.
  void disable_last_will();

  /// Set the birth message.
  void set_birth_message(MQTTMessage &&message);
  /// Remove the birth message.
  void disable_birth_message();

  void set_shutdown_message(MQTTMessage &&message);
  void disable_shutdown_message();

  /// Set the keep alive time in seconds, every 0.7*keep_alive a ping will be sent.
  void set_keep_alive(uint16_t keep_alive_s);

  /** Set the Home Assistant discovery info
   *
   * See <a href="https://home-assistant.io/docs/mqtt/discovery/">MQTT Discovery</a>.
   * @param prefix The Home Assistant discovery prefix.
   * @param retain Whether to retain discovery messages.
   */
  void set_discovery_info(std::string &&prefix, bool retain);
  /// Get Home Assistant discovery info.
  const MQTTDiscoveryInfo &get_discovery_info() const;
  /// Globally disable Home Assistant discovery.
  void disable_discovery();
  bool is_discovery_enabled() const;

  /// Manually set the client id, by default it's <name>-<MAC>, it's automatically truncated to 23 chars.
  void set_client_id(std::string client_id);

#if ASYNC_TCP_SSL_ENABLED
  /** Add a SSL fingerprint to use for TCP SSL connections to the MQTT broker.
   *
   * To use this feature you first have to globally enable the `ASYNC_TCP_SSL_ENABLED` define flag.
   * This function can be called multiple times and any certificate that matches any of the provided fingerprints
   * will match. Calling this method will also automatically disable all non-ssl connections.
   *
   * @warning This is *not* secure and *not* how SSL is usually done. You'll have to add
   *          a separate fingerprint for every certificate you use. Additionally, the hashing
   *          algorithm used here due to the constraints of the MCU, SHA1, is known to be insecure.
   *
   * @param fingerprint The SSL fingerprint as a 20 value long std::array.
   */
  void add_ssl_fingerprint(const std::array<uint8_t, SHA1_SIZE> &fingerprint);
#endif

  const Availability &get_availability();

  /** Set the topic prefix that will be prepended to all topics together with "/". This will, in most cases,
   * be the name of your Application.
   *
   * For example, if "livingroom" is passed to this method, all state topics will, by default, look like
   * "livingroom/.../state"
   *
   * @param topic_prefix The topic prefix. The last "/" is appended automatically.
   */
  void set_topic_prefix(std::string topic_prefix);
  /// Get the topic prefix of this device, using default if necessary
  const std::string &get_topic_prefix() const;

  /// Manually set the topic used for logging.
  void set_log_message_template(MQTTMessage &&message);
  void set_log_level(int level);
  /// Get the topic used for logging. Defaults to "<topic_prefix>/debug" and the value is cached for speed.
  void disable_log_message();
  bool is_log_message_enabled() const;

  /** Subscribe to an MQTT topic and call callback when a message is received.
   *
   * @param topic The topic. Wildcards are currently not supported.
   * @param callback The callback function.
   * @param qos The QoS of this subscription.
   */
  void subscribe(const std::string &topic, mqtt_callback_t callback, uint8_t qos = 0);

  /** Subscribe to a MQTT topic and automatically parse JSON payload.
   *
   * If an invalid JSON payload is received, the callback will not be called.
   *
   * @param topic The topic. Wildcards are currently not supported.
   * @param callback The callback with a parsed JsonObject that will be called when a message with matching topic is received.
   * @param qos The QoS of this subscription.
   */
  void subscribe_json(const std::string &topic, mqtt_json_callback_t callback, uint8_t qos = 0);

  /** Publish a MQTTMessage
   *
   * @param message The message.
   */
  void publish(const MQTTMessage &message);

  /** Publish a MQTT message
   *
   * @param topic The topic.
   * @param payload The payload.
   * @param retain Whether to retain the message.
   */
  void publish(const std::string &topic, const std::string &payload, uint8_t qos = 0, bool retain = false);

  void publish(const std::string &topic, const char *payload, size_t payload_length,
               uint8_t qos = 0, bool retain = false);

  /** Construct and send a JSON MQTT message.
   *
   * @param topic The topic.
   * @param f The Json Message builder.
   * @param retain Whether to retain the message.
   */
  void publish_json(const std::string &topic, const json_build_t &f, uint8_t qos = 0, bool retain = false);

  /// Setup the MQTT client, registering a bunch of callbacks and attempting to connect.
  void setup() override;
  void dump_config() override;
  /// Reconnect if required
  void loop() override;
  /// MQTT client setup priority
  float get_setup_priority() const override;

  void on_message(const std::string &topic, const std::string &payload);

  MQTTMessageTrigger *make_message_trigger(const std::string &topic);

  MQTTJsonMessageTrigger *make_json_message_trigger(const std::string &topic, uint8_t qos = 0);

  template<typename T>
  MQTTPublishAction<T> *make_publish_action();

  template<typename T>
  MQTTPublishJsonAction<T> *make_publish_json_action();

  bool can_proceed() override;

  void check_connected();

  void set_reboot_timeout(uint32_t reboot_timeout);

  void register_mqtt_component(MQTTComponent *component);

  bool is_connected();

 protected:
  /// Reconnect to the MQTT broker if not already connected.
  void start_connect();

  /// Re-calculate the availability property.
  void recalculate_availability();

  MQTTCredentials credentials_;
  /// The last will message. Disabled optional denotes it being default and
  /// an empty topic denotes the the feature being disabled.
  MQTTMessage last_will_;
  /// The birth message (e.g. the message that's send on an established connection.
  /// See last_will_ for what different values denote.
  MQTTMessage birth_message_;
  MQTTMessage shutdown_message_;
  /// Caches availability.
  Availability availability_{};
  /// The discovery info options for Home Assistant. Undefined optional means
  /// default and empty prefix means disabled.
  MQTTDiscoveryInfo discovery_info_{
      .prefix = "homeassistant",
      .retain = true
  };
  std::string topic_prefix_{};
  MQTTMessage log_message_;
  int log_level_{ESPHOMELIB_LOG_LEVEL};

  std::vector<MQTTSubscription> subscriptions_;
  AsyncMqttClient mqtt_client_;
  MQTTClientState state_{MQTT_CLIENT_DISCONNECTED};
  std::vector<MQTTComponent *> children_;
  uint32_t reboot_timeout_{300000};
  uint32_t connect_begin_;
  uint32_t last_connected_{0};
  optional<AsyncMqttClientDisconnectReason> disconnect_reason_{};
};

extern MQTTClientComponent *global_mqtt_client;

class MQTTMessageTrigger : public Trigger<std::string>, public Component {
 public:
  explicit MQTTMessageTrigger(const std::string &topic);

  void set_qos(uint8_t qos);
  void set_payload(const std::string &payload);
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

 protected:
  std::string topic_;
  uint8_t qos_{0};
  optional<std::string> payload_;
};

class MQTTJsonMessageTrigger : public Trigger<const JsonObject &> {
 public:
  explicit MQTTJsonMessageTrigger(const std::string &topic, uint8_t qos = 0);
};

template<typename T>
class MQTTPublishAction : public Action<T> {
 public:
  MQTTPublishAction();

  void set_topic(std::function<std::string(T)> topic);
  void set_topic(std::string topic);
  void set_payload(std::function<std::string(T)> payload);
  void set_payload(std::string payload);
  void set_qos(std::function<uint8_t (T)> qos);
  void set_qos(uint8_t  qos);
  void set_retain(std::function<bool(T)> retain);
  void set_retain(bool retain);

  void play(T x) override;

 protected:
  TemplatableValue<std::string, T> topic_;
  TemplatableValue<std::string, T> payload_;
  TemplatableValue<uint8_t, T> qos_{0};
  TemplatableValue<bool, T> retain_{false};
};

template<typename T>
class MQTTPublishJsonAction : public Action<T> {
 public:
  MQTTPublishJsonAction();

  void set_topic(std::function<std::string(T)> topic);
  void set_topic(std::string topic);
  void set_payload(std::function<void(T, JsonObject &)> payload);
  void set_qos(uint8_t qos);
  void set_retain(bool retain);
  void play(T x) override;

 protected:
  TemplatableValue<std::string, T> topic_;
  std::function<void(T, JsonObject &)> payload_;
  uint8_t qos_{0};
  bool retain_{false};
};

// =============== TEMPLATE DEFINITIONS ===============

template<typename T>
void MQTTPublishJsonAction<T>::set_topic(std::function<std::string(T)> topic) {
  this->topic_ = std::move(topic);
}
template<typename T>
void MQTTPublishJsonAction<T>::set_topic(std::string topic) {
  this->topic_ = topic;
}
template<typename T>
void MQTTPublishJsonAction<T>::set_payload(std::function<void(T, JsonObject &)> payload) {
  this->payload_ = std::move(payload);
}
template<typename T>
void MQTTPublishJsonAction<T>::set_qos(uint8_t qos) {
  this->qos_ = qos;
}
template<typename T>
void MQTTPublishJsonAction<T>::set_retain(bool retain) {
  this->retain_ = retain;
}
template<typename T>
void MQTTPublishJsonAction<T>::play(T x) {
  auto f = [this, x](JsonObject &root) {
    this->payload_(x, root);
  };
  global_mqtt_client->publish_json(this->topic_.value(x), f, this->qos_, this->retain_);
  this->play_next(x);
}
template<typename T>
MQTTPublishJsonAction<T>::MQTTPublishJsonAction() = default;

template<typename T>
MQTTPublishJsonAction<T> *MQTTClientComponent::make_publish_json_action() {
  return new MQTTPublishJsonAction<T>();
}

template<typename T>
void MQTTPublishAction<T>::play(T x) {
  global_mqtt_client->publish(this->topic_.value(x), this->payload_.value(x),
                              this->qos_.value(x), this->retain_.value(x));
  this->play_next(x);
}
template<typename T>
MQTTPublishAction<T>::MQTTPublishAction() = default;

template<typename T>
void MQTTPublishAction<T>::set_topic(std::function<std::string(T)> topic) {
  this->topic_ = std::move(topic);
}
template<typename T>
void MQTTPublishAction<T>::set_topic(std::string topic) {
  this->topic_ = topic;
}
template<typename T>
void MQTTPublishAction<T>::set_payload(std::function<std::string(T)> payload) {
  this->payload_ = std::move(payload);
}
template<typename T>
void MQTTPublishAction<T>::set_payload(std::string payload) {
  this->payload_ = payload;
}
template<typename T>
void MQTTPublishAction<T>::set_qos(std::function<uint8_t(T)> qos) {
  this->qos_ = std::move(qos);
}
template<typename T>
void MQTTPublishAction<T>::set_qos(uint8_t qos) {
  this->qos_ = qos;
}
template<typename T>
void MQTTPublishAction<T>::set_retain(std::function<bool(T)> retain) {
  this->retain_ = std::move(retain);
}
template<typename T>
void MQTTPublishAction<T>::set_retain(bool retain) {
  this->retain_ = retain;
}

template<typename T>
MQTTPublishAction<T> *MQTTClientComponent::make_publish_action() {
  return new MQTTPublishAction<T>();
}

} // namespace mqtt

ESPHOMELIB_NAMESPACE_END

#endif //ESPHOMELIB_MQTT_MQTT_CLIENT_COMPONENT_H
