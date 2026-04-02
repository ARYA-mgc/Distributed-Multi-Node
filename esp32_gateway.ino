/**
 * @file    esp32_gateway.ino
 * @brief   ESP32 WiFi Gateway — Network Bridge & Cloud Uplink
 *
 * @author  ARYA mgc
 * @version 1.0.0
 * @date    2025
 *
 * @details The ESP32 gateway sits between the STM32 central node and the
 *          cloud/dashboard. It:
 *            - Receives aggregated sensor data from the central node via UART
 *            - Publishes data to an MQTT broker over WiFi
 *            - Subscribes to MQTT command topics and relays commands back
 *              to the central node via UART
 *            - Exposes a local REST API (HTTP) for LAN dashboard access
 *            - Maintains a circular buffer of the last 100 readings
 *            - Auto-reconnects WiFi and MQTT on drops
 *
 * @hardware ESP32-DevKitC or equivalent
 *           - UART2 (GPIO16 RX, GPIO17 TX): Link to STM32 central node
 *           - LED: GPIO2 (onboard)
 *
 * @dependencies
 *   - PubSubClient  (MQTT): Install via Arduino Library Manager
 *   - ArduinoJson   (JSON): Install via Arduino Library Manager
 *   - WebServer     (HTTP): Built into ESP32 Arduino core
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <HardwareSerial.h>

/* ============================================================
 *  USER CONFIG — Edit before flashing
 * ============================================================ */

#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASSWORD       "YOUR_WIFI_PASSWORD"

#define MQTT_BROKER         "192.168.1.100"   /* Your MQTT broker IP     */
#define MQTT_PORT           1883
#define MQTT_CLIENT_ID      "sensor-net-gw-arya"
#define MQTT_USER           ""                /* Leave blank if none     */
#define MQTT_PASS           ""

/* MQTT Topics */
#define TOPIC_SENSOR_DATA   "sensor_network/data"
#define TOPIC_FAULT         "sensor_network/fault"
#define TOPIC_RECOVERY      "sensor_network/recovery"
#define TOPIC_STATUS        "sensor_network/status"
#define TOPIC_CMD_IN        "sensor_network/cmd"    /* Subscribe: cloud → GW  */

#define HTTP_PORT           80
#define UART_BAUD           115200
#define UART_RX_PIN         16
#define UART_TX_PIN         17
#define STATUS_LED_PIN      2

/* ============================================================
 *  PROTOCOL INCLUDES  (shared with STM32)
 * ============================================================ */

/* Inline the protocol constants here since we can't use the STM32 HAL headers */
#define FRAME_START_BYTE    0xAA
#define FRAME_END_BYTE      0x55
#define MAX_PAYLOAD_SIZE    64
#define MSG_CLOUD_FORWARD   0x70
#define MSG_CLOUD_CMD       0x71
#define MSG_FAULT_REPORT    0x30
#define MSG_RECOVERY_CMD    0x40
#define CENTRAL_NODE_ID     0x00
#define NODE_ESP32_GW       0x05
#define BROADCAST_ID        0xFF

/* ============================================================
 *  RING BUFFER FOR READINGS
 * ============================================================ */

#define RING_BUF_SIZE  100

struct SensorReading {
    uint32_t timestamp;
    uint8_t  node_id;
    float    temperature;
    float    humidity;
    uint32_t pressure;
    int16_t  vibration;
    uint16_t voltage;
    uint16_t current;
    uint16_t gas_ppm;
    uint8_t  state;
    uint8_t  faults;
};

static SensorReading ring_buf[RING_BUF_SIZE];
static int ring_head = 0;
static int ring_count = 0;

/* ============================================================
 *  GLOBALS
 * ============================================================ */

HardwareSerial  SerialSTM32(2);   /* UART2 */
WiFiClient      wifi_client;
PubSubClient    mqtt(wifi_client);
WebServer       http_server(HTTP_PORT);

static uint8_t  rx_buf[256];
static int      rx_pos = 0;
static bool     led_state = false;
static uint32_t last_mqtt_reconnect = 0;
static uint32_t last_status_pub     = 0;
static uint32_t total_frames_rx     = 0;
static uint32_t total_faults_seen   = 0;

/* ============================================================
 *  SETUP
 * ============================================================ */

void setup()
{
    Serial.begin(115200);
    Serial.println("\n[GW] ESP32 Gateway — ARYA mgc v1.0");

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);

    /* UART to STM32 central node */
    SerialSTM32.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    /* WiFi */
    wifi_connect();

    /* MQTT */
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    mqtt.setCallback(mqtt_callback);
    mqtt.setBufferSize(1024);
    mqtt_connect();

    /* HTTP REST API */
    http_server.on("/",             HTTP_GET, handle_root);
    http_server.on("/api/status",   HTTP_GET, handle_api_status);
    http_server.on("/api/readings", HTTP_GET, handle_api_readings);
    http_server.on("/api/cmd",      HTTP_POST, handle_api_cmd);
    http_server.begin();
    Serial.printf("[GW] HTTP API on http://%s/\n", WiFi.localIP().toString().c_str());
}

/* ============================================================
 *  MAIN LOOP
 * ============================================================ */

void loop()
{
    /* Maintain WiFi */
    if (WiFi.status() != WL_CONNECTED) wifi_connect();

    /* Maintain MQTT */
    if (!mqtt.connected()) {
        uint32_t now = millis();
        if (now - last_mqtt_reconnect > 5000) {
            last_mqtt_reconnect = now;
            mqtt_connect();
        }
    }
    mqtt.loop();

    /* Process HTTP requests */
    http_server.handleClient();

    /* Read UART from STM32 */
    read_stm32_uart();

    /* Periodic gateway status publish */
    uint32_t now = millis();
    if (now - last_status_pub > 30000) {
        last_status_pub = now;
        publish_gateway_status();
    }

    /* Blink LED every 1s when running normally */
    static uint32_t last_blink = 0;
    if (now - last_blink > 1000) {
        last_blink = now;
        led_state = !led_state;
        digitalWrite(STATUS_LED_PIN, led_state);
    }
}

/* ============================================================
 *  UART FRAME READER
 * ============================================================ */

static void read_stm32_uart()
{
    while (SerialSTM32.available()) {
        uint8_t byte = SerialSTM32.read();
        rx_buf[rx_pos++] = byte;

        /* Simple framing: look for start + end markers */
        if (rx_pos >= 10 && rx_buf[0] == FRAME_START_BYTE) {
            /* Find end byte */
            if (byte == FRAME_END_BYTE && rx_pos >= 10) {
                process_stm32_frame(rx_buf, rx_pos);
                rx_pos = 0;
                memset(rx_buf, 0, sizeof(rx_buf));
            }
        } else if (byte != FRAME_START_BYTE) {
            rx_pos = 0; /* Resync */
        }

        if (rx_pos >= (int)sizeof(rx_buf)) rx_pos = 0; /* Overflow guard */
    }
}

/* ============================================================
 *  FRAME PROCESSOR
 * ============================================================ */

static void process_stm32_frame(const uint8_t *buf, int len)
{
    if (len < 10) return;

    uint8_t src_id   = buf[1];
    uint8_t dst_id   = buf[2];
    uint8_t msg_type = buf[3];
    uint8_t pay_len  = buf[6];
    const uint8_t *payload = &buf[7];

    if (dst_id != NODE_ESP32_GW && dst_id != BROADCAST_ID) return;

    total_frames_rx++;

    if (msg_type == MSG_CLOUD_FORWARD && pay_len >= 18) {
        /* Parse SensorDataPayload_t */
        SensorReading r;
        uint32_t ts;
        memcpy(&ts, payload, 4);
        r.timestamp   = millis();
        r.node_id     = src_id;
        int16_t  t16; memcpy(&t16, payload + 4, 2); r.temperature = t16 / 100.0f;
        uint16_t h16; memcpy(&h16, payload + 6, 2); r.humidity    = h16 / 100.0f;
        memcpy(&r.pressure,  payload + 8,  4);
        memcpy(&r.vibration, payload + 12, 2);
        memcpy(&r.voltage,   payload + 14, 2);
        memcpy(&r.current,   payload + 16, 2);
        memcpy(&r.gas_ppm,   payload + 18, 2);
        r.state  = payload[20];
        r.faults = payload[21];

        /* Store in ring buffer */
        ring_buf[ring_head] = r;
        ring_head = (ring_head + 1) % RING_BUF_SIZE;
        if (ring_count < RING_BUF_SIZE) ring_count++;

        /* Publish to MQTT */
        publish_sensor_data(&r);

        /* Check for fault flags */
        if (r.faults != 0) {
            total_faults_seen++;
            publish_fault_alert(src_id, r.faults);
        }

        Serial.printf("[GW] Frame from node 0x%02X: T=%.2f°C H=%.1f%% P=%u V=%d gas=%dppm\n",
                      src_id, r.temperature, r.humidity, r.pressure,
                      r.vibration, r.gas_ppm);
    }

    else if (msg_type == MSG_FAULT_REPORT) {
        total_faults_seen++;
        Serial.printf("[GW] Fault report from node 0x%02X code=0x%02X\n",
                      src_id, payload[0]);
        publish_fault_alert(src_id, payload[0]);
    }
}

/* ============================================================
 *  MQTT PUBLISH
 * ============================================================ */

static void publish_sensor_data(const SensorReading *r)
{
    if (!mqtt.connected()) return;

    StaticJsonDocument<256> doc;
    doc["node"]        = r->node_id;
    doc["ts"]          = r->timestamp;
    doc["temp_c"]      = serialized(String(r->temperature, 2));
    doc["humidity_pct"]= serialized(String(r->humidity, 1));
    doc["pressure_pa"] = r->pressure;
    doc["vibration_mg"]= r->vibration;
    doc["voltage_mv"]  = r->voltage;
    doc["current_ma"]  = r->current;
    doc["gas_ppm"]     = r->gas_ppm;
    doc["state"]       = r->state;
    doc["faults"]      = r->faults;

    char json_buf[256];
    serializeJson(doc, json_buf, sizeof(json_buf));

    char topic[64];
    snprintf(topic, sizeof(topic), "%s/node%02x", TOPIC_SENSOR_DATA, r->node_id);
    mqtt.publish(topic, json_buf);
}

static void publish_fault_alert(uint8_t node_id, uint8_t fault_code)
{
    if (!mqtt.connected()) return;

    StaticJsonDocument<128> doc;
    doc["node"]       = node_id;
    doc["fault_code"] = fault_code;
    doc["ts"]         = millis();

    char json_buf[128];
    serializeJson(doc, json_buf, sizeof(json_buf));
    mqtt.publish(TOPIC_FAULT, json_buf);
}

static void publish_gateway_status()
{
    if (!mqtt.connected()) return;

    StaticJsonDocument<128> doc;
    doc["ip"]          = WiFi.localIP().toString();
    doc["rssi"]        = WiFi.RSSI();
    doc["frames_rx"]   = total_frames_rx;
    doc["faults_seen"] = total_faults_seen;
    doc["uptime_s"]    = millis() / 1000;

    char json_buf[128];
    serializeJson(doc, json_buf, sizeof(json_buf));
    mqtt.publish(TOPIC_STATUS, json_buf);
}

/* ============================================================
 *  MQTT SUBSCRIBE CALLBACK (cloud → STM32)
 * ============================================================ */

static void mqtt_callback(char *topic, byte *payload_bytes, unsigned int length)
{
    char json[256];
    if (length >= sizeof(json)) return;
    memcpy(json, payload_bytes, length);
    json[length] = '\0';

    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, json) != DeserializationError::Ok) return;

    uint8_t node_id      = doc["node"]     | 0xFF;
    uint8_t recovery_cmd = doc["recovery"] | 0x00;

    if (node_id == 0xFF || recovery_cmd == 0x00) return;

    /* Build and forward MSG_CLOUD_CMD to STM32 central */
    uint8_t cmd_payload[2] = { node_id, recovery_cmd };
    send_to_central(MSG_CLOUD_CMD, cmd_payload, 2);

    Serial.printf("[GW] MQTT cmd: recovery=0x%02X for node 0x%02X\n",
                  recovery_cmd, node_id);
}

/* ============================================================
 *  SEND FRAME TO STM32 CENTRAL
 * ============================================================ */

static void send_to_central(uint8_t msg_type, const uint8_t *payload, uint8_t len)
{
    uint8_t wire[80];
    int i = 0;
    static uint16_t seq = 0;

    wire[i++] = FRAME_START_BYTE;
    wire[i++] = NODE_ESP32_GW;
    wire[i++] = CENTRAL_NODE_ID;
    wire[i++] = msg_type;
    wire[i++] = (uint8_t)(seq >> 8);
    wire[i++] = (uint8_t)(seq & 0xFF);
    seq++;
    wire[i++] = len;
    memcpy(&wire[i], payload, len); i += len;
    /* Simple CRC placeholder — implement crc16_compute if desired */
    wire[i++] = 0x00;
    wire[i++] = 0x00;
    wire[i++] = FRAME_END_BYTE;

    SerialSTM32.write(wire, i);
}

/* ============================================================
 *  HTTP REST API
 * ============================================================ */

static void handle_root()
{
    http_server.send(200, "text/plain",
        "Sensor Network Gateway — ARYA mgc\n"
        "GET  /api/status    -> gateway status\n"
        "GET  /api/readings  -> last 100 readings\n"
        "POST /api/cmd       -> send recovery command\n");
}

static void handle_api_status()
{
    StaticJsonDocument<256> doc;
    doc["gateway"]       = "ESP32";
    doc["author"]        = "ARYA mgc";
    doc["ip"]            = WiFi.localIP().toString();
    doc["rssi_dbm"]      = WiFi.RSSI();
    doc["mqtt_connected"]= mqtt.connected();
    doc["frames_rx"]     = total_frames_rx;
    doc["faults_seen"]   = total_faults_seen;
    doc["uptime_s"]      = millis() / 1000;
    doc["readings_buf"]  = ring_count;

    String out;
    serializeJsonPretty(doc, out);
    http_server.send(200, "application/json", out);
}

static void handle_api_readings()
{
    DynamicJsonDocument doc(8192);
    JsonArray arr = doc.to<JsonArray>();

    for (int i = 0; i < ring_count; i++) {
        int idx = (ring_head - ring_count + i + RING_BUF_SIZE) % RING_BUF_SIZE;
        const SensorReading &r = ring_buf[idx];
        JsonObject obj = arr.createNestedObject();
        obj["node"]        = r.node_id;
        obj["ts"]          = r.timestamp;
        obj["temp_c"]      = serialized(String(r.temperature, 2));
        obj["humidity_pct"]= serialized(String(r.humidity, 1));
        obj["pressure_pa"] = r.pressure;
        obj["vibration_mg"]= r.vibration;
        obj["voltage_mv"]  = r.voltage;
        obj["current_ma"]  = r.current;
        obj["gas_ppm"]     = r.gas_ppm;
        obj["state"]       = r.state;
        obj["faults"]      = r.faults;
    }

    String out;
    serializeJson(doc, out);
    http_server.send(200, "application/json", out);
}

static void handle_api_cmd()
{
    if (!http_server.hasArg("plain")) {
        http_server.send(400, "text/plain", "No body");
        return;
    }
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, http_server.arg("plain"))) {
        http_server.send(400, "text/plain", "Bad JSON");
        return;
    }
    uint8_t node_id      = doc["node"]     | 0;
    uint8_t recovery_cmd = doc["recovery"] | 0;
    uint8_t cmd[2]       = { node_id, recovery_cmd };
    send_to_central(MSG_CLOUD_CMD, cmd, 2);
    http_server.send(200, "application/json", "{\"ok\":true}");
}

/* ============================================================
 *  WIFI / MQTT HELPERS
 * ============================================================ */

static void wifi_connect()
{
    Serial.printf("[GW] Connecting to WiFi %s ...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries++ < 20) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[GW] WiFi OK — IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[GW] WiFi FAILED — running in UART-only mode");
    }
}

static void mqtt_connect()
{
    Serial.printf("[GW] Connecting to MQTT %s:%d ...\n", MQTT_BROKER, MQTT_PORT);
    bool ok = (strlen(MQTT_USER) > 0)
        ? mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)
        : mqtt.connect(MQTT_CLIENT_ID);

    if (ok) {
        Serial.println("[GW] MQTT connected");
        mqtt.subscribe(TOPIC_CMD_IN);
    } else {
        Serial.printf("[GW] MQTT failed, rc=%d\n", mqtt.state());
    }
}
