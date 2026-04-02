/**
 * @file    network_protocol.h
 * @brief   Distributed Multi-Node Sensor Network Protocol Definitions
 * @author  ARYA mgc
 * @version 1.0.0
 * @date    2025
 *
 * @details Shared protocol definitions for all nodes in the network.
 *          Compatible with STM32 (sensor nodes) and ESP32 (gateway/WiFi bridge).
 */

#ifndef NETWORK_PROTOCOL_H
#define NETWORK_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 *  NETWORK CONFIGURATION
 * ============================================================ */

#define NETWORK_VERSION         0x01
#define MAX_NODES               8
#define CENTRAL_NODE_ID         0x00
#define BROADCAST_ID            0xFF

#define UART_BAUD_RATE          115200
#define FRAME_START_BYTE        0xAA
#define FRAME_END_BYTE          0x55
#define MAX_PAYLOAD_SIZE        64

/* ============================================================
 *  NODE IDs
 * ============================================================ */

typedef enum {
    NODE_CENTRAL   = 0x00,   /* Central coordinator (STM32 master) */
    NODE_SENSOR_1  = 0x01,   /* Sensor node 1 - Temperature/Humidity */
    NODE_SENSOR_2  = 0x02,   /* Sensor node 2 - Pressure/Vibration   */
    NODE_SENSOR_3  = 0x03,   /* Sensor node 3 - Current/Voltage       */
    NODE_SENSOR_4  = 0x04,   /* Sensor node 4 - Gas/Air Quality       */
    NODE_ESP32_GW  = 0x05,   /* ESP32 WiFi gateway node               */
    NODE_UNKNOWN   = 0xFF
} NodeID_t;

/* ============================================================
 *  MESSAGE TYPES
 * ============================================================ */

typedef enum {
    /* Data messages */
    MSG_SENSOR_DATA       = 0x10,  /* Sensor node → Central: periodic data */
    MSG_SENSOR_ACK        = 0x11,  /* Central → Sensor: data acknowledged  */

    /* Health / Heartbeat */
    MSG_HEARTBEAT         = 0x20,  /* Periodic alive signal from each node */
    MSG_HEARTBEAT_RSP     = 0x21,  /* Central heartbeat response           */

    /* Fault reporting */
    MSG_FAULT_REPORT      = 0x30,  /* Sensor → Central: fault detected     */
    MSG_FAULT_ACK         = 0x31,  /* Central → Sensor: fault acknowledged */

    /* Recovery commands */
    MSG_RECOVERY_CMD      = 0x40,  /* Central → Sensor: recovery action    */
    MSG_RECOVERY_STATUS   = 0x41,  /* Sensor → Central: recovery result    */

    /* Configuration */
    MSG_CONFIG_SET        = 0x50,  /* Central → Sensor: set parameter      */
    MSG_CONFIG_GET        = 0x51,  /* Central → Sensor: request config     */
    MSG_CONFIG_RSP        = 0x52,  /* Sensor → Central: config response    */

    /* Network management */
    MSG_NODE_REGISTER     = 0x60,  /* Sensor → Central: node joining       */
    MSG_NODE_DEREGISTER   = 0x61,  /* Sensor → Central: node leaving       */
    MSG_NETWORK_STATUS    = 0x62,  /* Central → All: broadcast status      */

    /* ESP32 / Cloud bridge */
    MSG_CLOUD_FORWARD     = 0x70,  /* Central → ESP32: forward to cloud    */
    MSG_CLOUD_CMD         = 0x71,  /* ESP32 → Central: command from cloud  */
} MessageType_t;

/* ============================================================
 *  FAULT CODES
 * ============================================================ */

typedef enum {
    FAULT_NONE              = 0x00,
    FAULT_SENSOR_TIMEOUT    = 0x01,  /* Sensor not responding              */
    FAULT_SENSOR_OUT_OF_RANGE = 0x02,/* Reading outside valid range        */
    FAULT_CHECKSUM_ERROR    = 0x03,  /* Data integrity failure             */
    FAULT_COMM_ERROR        = 0x04,  /* UART communication error           */
    FAULT_POWER_ANOMALY     = 0x05,  /* Voltage/Current out of spec        */
    FAULT_TEMP_CRITICAL     = 0x06,  /* Temperature threshold exceeded     */
    FAULT_VIBRATION_HIGH    = 0x07,  /* Vibration above safe limit         */
    FAULT_GAS_LEAK          = 0x08,  /* Gas concentration critical         */
    FAULT_NODE_OFFLINE      = 0x09,  /* Node missed N heartbeats           */
    FAULT_MEMORY_OVERFLOW   = 0x0A,  /* Node buffer overflow               */
    FAULT_WATCHDOG_RESET    = 0x0B,  /* Node recovered from WDT reset      */
    FAULT_UNKNOWN           = 0xFF
} FaultCode_t;

/* ============================================================
 *  RECOVERY COMMANDS
 * ============================================================ */

typedef enum {
    RECOVERY_NONE           = 0x00,
    RECOVERY_RESET_SENSOR   = 0x01,  /* Re-init the sensor peripheral      */
    RECOVERY_SOFT_RESET     = 0x02,  /* Software reset the node            */
    RECOVERY_RECALIBRATE    = 0x03,  /* Trigger sensor recalibration       */
    RECOVERY_ENTER_SAFE     = 0x04,  /* Enter safe/low-power mode          */
    RECOVERY_EXIT_SAFE      = 0x05,  /* Exit safe mode, resume normal      */
    RECOVERY_CLEAR_FAULT    = 0x06,  /* Clear fault flag, resume           */
    RECOVERY_UPDATE_RATE    = 0x07,  /* Change reporting interval          */
    RECOVERY_FACTORY_RESET  = 0x0F,  /* Full factory reset (last resort)   */
} RecoveryCmd_t;

/* ============================================================
 *  NODE STATES
 * ============================================================ */

typedef enum {
    NODE_STATE_INIT         = 0x00,
    NODE_STATE_REGISTERING  = 0x01,
    NODE_STATE_ACTIVE       = 0x02,
    NODE_STATE_FAULT        = 0x03,
    NODE_STATE_RECOVERING   = 0x04,
    NODE_STATE_SAFE_MODE    = 0x05,
    NODE_STATE_OFFLINE      = 0x06,
} NodeState_t;

/* ============================================================
 *  FRAME STRUCTURE
 * ============================================================ */

/**
 * @brief UART frame format
 *
 * | START | SRC_ID | DST_ID | MSG_TYPE | SEQ | LENGTH | PAYLOAD[n] | CRC16 | END |
 * |  0xAA |  1B    |  1B    |   1B     | 2B  |   1B   |   0..64B   |  2B   | 0x55|
 */
typedef struct __attribute__((packed)) {
    uint8_t  start;                      /* 0xAA                        */
    uint8_t  src_id;                     /* Sender node ID              */
    uint8_t  dst_id;                     /* Destination (0xFF=broadcast)*/
    uint8_t  msg_type;                   /* MessageType_t               */
    uint16_t seq_num;                    /* Sequence number             */
    uint8_t  length;                     /* Payload length in bytes     */
    uint8_t  payload[MAX_PAYLOAD_SIZE];  /* Variable payload            */
    uint16_t crc16;                      /* CRC-16/CCITT checksum       */
    uint8_t  end;                        /* 0x55                        */
} NetworkFrame_t;

/* ============================================================
 *  PAYLOAD STRUCTURES
 * ============================================================ */

/** Sensor data payload (MSG_SENSOR_DATA) */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;    /* Node uptime in ms           */
    int16_t  temperature_c;   /* °C × 100 (fixed point)      */
    uint16_t humidity_pct;    /* % × 100                     */
    uint32_t pressure_pa;     /* Pascals                     */
    int16_t  vibration_mg;    /* milli-g                     */
    uint16_t voltage_mv;      /* Millivolts                  */
    uint16_t current_ma;      /* Milliamps                   */
    uint16_t gas_ppm;         /* Parts per million           */
    uint8_t  node_state;      /* NodeState_t                 */
    uint8_t  fault_flags;     /* Bitmask of active faults    */
} SensorDataPayload_t;

/** Heartbeat payload (MSG_HEARTBEAT) */
typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint8_t  node_state;
    uint8_t  fault_count;
    uint16_t free_heap;       /* Bytes free (ESP32) / stack watermark (STM32) */
} HeartbeatPayload_t;

/** Fault report payload (MSG_FAULT_REPORT) */
typedef struct __attribute__((packed)) {
    uint8_t  fault_code;      /* FaultCode_t                  */
    uint8_t  severity;        /* 1=low, 2=medium, 3=critical  */
    uint32_t fault_timestamp;
    int32_t  fault_value;     /* The out-of-range value        */
    int32_t  threshold_value; /* The threshold it crossed      */
} FaultPayload_t;

/** Recovery command payload (MSG_RECOVERY_CMD) */
typedef struct __attribute__((packed)) {
    uint8_t  recovery_cmd;    /* RecoveryCmd_t                 */
    uint32_t param;           /* Optional parameter            */
} RecoveryCmdPayload_t;

/** Node registration payload (MSG_NODE_REGISTER) */
typedef struct __attribute__((packed)) {
    uint8_t  node_id;
    uint8_t  hw_version;
    uint8_t  fw_version_major;
    uint8_t  fw_version_minor;
    char     node_name[16];
    uint8_t  capabilities;    /* Bitmask: sensors present      */
} NodeRegisterPayload_t;

/* ============================================================
 *  CAPABILITY FLAGS
 * ============================================================ */

#define CAP_TEMPERATURE     (1 << 0)
#define CAP_HUMIDITY        (1 << 1)
#define CAP_PRESSURE        (1 << 2)
#define CAP_VIBRATION       (1 << 3)
#define CAP_VOLTAGE         (1 << 4)
#define CAP_CURRENT         (1 << 5)
#define CAP_GAS             (1 << 6)
#define CAP_WIFI            (1 << 7)

/* ============================================================
 *  UTILITY MACROS
 * ============================================================ */

#define FRAME_OVERHEAD_SIZE   (sizeof(NetworkFrame_t) - MAX_PAYLOAD_SIZE)
#define FIXED_TO_FLOAT(x)     ((float)(x) / 100.0f)

#endif /* NETWORK_PROTOCOL_H */
