/**
 * @file    central_node_main.c
 * @brief   Central Coordinator Node — STM32
 *          Fault Detection, Self-Recovery, Network Management
 *
 * @author  ARYA mgc
 * @version 1.0.0
 * @date    2025
 *
 * @details The central node is the master of the distributed sensor network.
 *          It:
 *            - Polls heartbeats from all registered sensor nodes
 *            - Collects and logs sensor data over UART
 *            - Detects faults via timeout, out-of-range, and explicit reports
 *            - Issues recovery commands automatically (tiered escalation)
 *            - Forwards aggregated data to ESP32 gateway for cloud upload
 *
 * @hardware STM32F4xx (e.g. STM32F407VGT6)
 *           - USART1: Sensor bus (RS-485 or direct UART ring)
 *           - USART2: ESP32 gateway link
 *           - USART3: Debug / PC terminal
 *           - TIM2:   1ms system tick
 *           - LED_STATUS: PA5
 *           - LED_FAULT:  PA6
 */

#include "stm32f4xx_hal.h"
#include "network_protocol.h"
#include "frame_codec.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ============================================================
 *  COMPILE-TIME CONFIG
 * ============================================================ */

#define HEARTBEAT_TIMEOUT_MS        3000   /* Miss 3s → node suspect          */
#define HEARTBEAT_DEAD_MS           9000   /* Miss 9s → node offline           */
#define SENSOR_REPORT_TIMEOUT_MS    5000   /* No data in 5s → fault            */
#define RECOVERY_RETRY_MAX          3      /* Attempts before escalation       */
#define RECOVERY_BACKOFF_MS         2000   /* Delay between retry attempts     */
#define CLOUD_FORWARD_INTERVAL_MS   10000  /* Push to ESP32 every 10s          */

#define UART_RX_BUF_SIZE            256
#define LOG_BUF_SIZE                128

/* ============================================================
 *  NODE REGISTRY
 * ============================================================ */

typedef struct {
    uint8_t     id;
    char        name[16];
    NodeState_t state;
    uint8_t     capabilities;
    uint8_t     hw_version;
    uint8_t     fw_major;
    uint8_t     fw_minor;

    uint32_t    last_heartbeat_ms;
    uint32_t    last_data_ms;
    uint16_t    missed_heartbeats;

    FaultCode_t active_fault;
    uint8_t     fault_severity;
    uint8_t     recovery_attempts;
    uint32_t    recovery_start_ms;

    SensorDataPayload_t latest_data;
    bool                data_valid;
    bool                registered;
} NodeEntry_t;

static NodeEntry_t node_registry[MAX_NODES];
static uint8_t     node_count = 0;

/* ============================================================
 *  SYSTEM GLOBALS
 * ============================================================ */

static UART_HandleTypeDef huart1;   /* Sensor bus        */
static UART_HandleTypeDef huart2;   /* ESP32 gateway     */
static UART_HandleTypeDef huart3;   /* Debug terminal    */

static uint8_t rx_buf1[UART_RX_BUF_SIZE];
static uint8_t rx_buf2[UART_RX_BUF_SIZE];
static volatile uint16_t rx1_len = 0;
static volatile uint16_t rx2_len = 0;
static volatile bool rx1_ready = false;
static volatile bool rx2_ready = false;

static uint32_t last_cloud_forward_ms = 0;
static char     log_buf[LOG_BUF_SIZE];

/* ============================================================
 *  FORWARD DECLARATIONS
 * ============================================================ */

static void system_clock_config(void);
static void uart_init(void);
static void gpio_init(void);

static NodeEntry_t *registry_find(uint8_t id);
static NodeEntry_t *registry_register(const NodeRegisterPayload_t *reg);

static void handle_frame(const NetworkFrame_t *frame, UART_HandleTypeDef *reply_uart);
static void handle_heartbeat(NodeEntry_t *node, const HeartbeatPayload_t *hb);
static void handle_sensor_data(NodeEntry_t *node, const SensorDataPayload_t *data);
static void handle_fault_report(NodeEntry_t *node, const FaultPayload_t *fault);
static void handle_recovery_status(NodeEntry_t *node, const uint8_t *payload);

static void fault_detection_tick(void);
static void trigger_recovery(NodeEntry_t *node, RecoveryCmd_t cmd);
static RecoveryCmd_t select_recovery_action(NodeEntry_t *node);
static void escalate_recovery(NodeEntry_t *node);

static void cloud_forward_tick(void);
static void send_frame(UART_HandleTypeDef *uart, uint8_t dst,
                       uint8_t msg_type, const void *payload, uint8_t len);
static void debug_log(const char *fmt, ...);
static void status_leds_update(void);

/* ============================================================
 *  SENSOR THRESHOLD LIMITS
 * ============================================================ */

typedef struct {
    int16_t  temp_min_c100;      /* -40.00°C  */
    int16_t  temp_max_c100;      /* +85.00°C  */
    int16_t  temp_crit_c100;     /* +75.00°C  */
    uint32_t pressure_min_pa;    /* 80000 Pa  */
    uint32_t pressure_max_pa;    /* 110000 Pa */
    int16_t  vibration_max_mg;   /* 2000 mg   */
    uint16_t voltage_min_mv;     /* 4750 mV   */
    uint16_t voltage_max_mv;     /* 5250 mV   */
    uint16_t current_max_ma;     /* 2000 mA   */
    uint16_t gas_warn_ppm;       /* 400 ppm   */
    uint16_t gas_crit_ppm;       /* 1000 ppm  */
} SensorThresholds_t;

static const SensorThresholds_t thresholds = {
    .temp_min_c100    = -4000,
    .temp_max_c100    =  8500,
    .temp_crit_c100   =  7500,
    .pressure_min_pa  = 80000,
    .pressure_max_pa  = 110000,
    .vibration_max_mg = 2000,
    .voltage_min_mv   = 4750,
    .voltage_max_mv   = 5250,
    .current_max_ma   = 2000,
    .gas_warn_ppm     = 400,
    .gas_crit_ppm     = 1000,
};

/* ============================================================
 *  MAIN
 * ============================================================ */

int main(void)
{
    HAL_Init();
    system_clock_config();
    gpio_init();
    uart_init();

    memset(node_registry, 0, sizeof(node_registry));

    debug_log("\r\n[CENTRAL] Distributed Sensor Network v1.0 — ARYA mgc\r\n");
    debug_log("[CENTRAL] Waiting for nodes to register...\r\n");

    /* Start UART idle-line DMA reception */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf1, UART_RX_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf2, UART_RX_BUF_SIZE);

    while (1) {
        /* Process incoming sensor bus frame */
        if (rx1_ready) {
            rx1_ready = false;
            NetworkFrame_t *frame = (NetworkFrame_t *)rx_buf1;
            if (frame_decode(frame) == CODEC_OK) {
                handle_frame(frame, &huart1);
            } else {
                debug_log("[CENTRAL] Bad frame from sensor bus (CRC/format error)\r\n");
            }
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf1, UART_RX_BUF_SIZE);
        }

        /* Process incoming ESP32 gateway frame */
        if (rx2_ready) {
            rx2_ready = false;
            NetworkFrame_t *frame = (NetworkFrame_t *)rx_buf2;
            if (frame_decode(frame) == CODEC_OK) {
                handle_frame(frame, &huart2);
            }
            HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf2, UART_RX_BUF_SIZE);
        }

        /* Periodic fault detection (runs every loop iteration) */
        fault_detection_tick();

        /* Periodic cloud forwarding to ESP32 */
        cloud_forward_tick();

        /* Status LED update */
        status_leds_update();
    }
}

/* ============================================================
 *  FRAME DISPATCHER
 * ============================================================ */

static void handle_frame(const NetworkFrame_t *frame, UART_HandleTypeDef *reply_uart)
{
    NodeEntry_t *node = registry_find(frame->src_id);

    /* Allow registration even if not yet in registry */
    if (frame->msg_type == MSG_NODE_REGISTER) {
        const NodeRegisterPayload_t *reg = (const NodeRegisterPayload_t *)frame->payload;
        node = registry_register(reg);
        if (node) {
            debug_log("[CENTRAL] Node 0x%02X '%s' registered (HW:%d FW:%d.%d caps:0x%02X)\r\n",
                reg->node_id, reg->node_name,
                reg->hw_version, reg->fw_version_major, reg->fw_version_minor,
                reg->capabilities);
            /* ACK registration */
            send_frame(reply_uart, frame->src_id, MSG_NETWORK_STATUS, NULL, 0);
        }
        return;
    }

    if (!node) {
        debug_log("[CENTRAL] Frame from unknown node 0x%02X — ignoring\r\n", frame->src_id);
        return;
    }

    switch (frame->msg_type) {
        case MSG_HEARTBEAT:
            handle_heartbeat(node, (const HeartbeatPayload_t *)frame->payload);
            send_frame(reply_uart, node->id, MSG_HEARTBEAT_RSP, NULL, 0);
            break;

        case MSG_SENSOR_DATA:
            handle_sensor_data(node, (const SensorDataPayload_t *)frame->payload);
            send_frame(reply_uart, node->id, MSG_SENSOR_ACK, NULL, 0);
            break;

        case MSG_FAULT_REPORT:
            handle_fault_report(node, (const FaultPayload_t *)frame->payload);
            send_frame(reply_uart, node->id, MSG_FAULT_ACK, NULL, 0);
            break;

        case MSG_RECOVERY_STATUS:
            handle_recovery_status(node, frame->payload);
            break;

        case MSG_NODE_DEREGISTER:
            debug_log("[CENTRAL] Node 0x%02X deregistered\r\n", node->id);
            node->state      = NODE_STATE_OFFLINE;
            node->registered = false;
            break;

        default:
            debug_log("[CENTRAL] Unknown msg type 0x%02X from node 0x%02X\r\n",
                      frame->msg_type, frame->src_id);
            break;
    }
}

/* ============================================================
 *  HEARTBEAT HANDLER
 * ============================================================ */

static void handle_heartbeat(NodeEntry_t *node, const HeartbeatPayload_t *hb)
{
    node->last_heartbeat_ms  = HAL_GetTick();
    node->missed_heartbeats  = 0;

    /* If node was in fault/offline due to missed HBs, recover it */
    if (node->state == NODE_STATE_OFFLINE || node->state == NODE_STATE_FAULT) {
        debug_log("[CENTRAL] Node 0x%02X back online (uptime=%lums)\r\n",
                  node->id, hb->uptime_ms);
        node->state          = NODE_STATE_ACTIVE;
        node->active_fault   = FAULT_NONE;
        node->recovery_attempts = 0;
    }

    debug_log("[HB] Node 0x%02X | state=%d fault_cnt=%d heap=%d\r\n",
              node->id, hb->node_state, hb->fault_count, hb->free_heap);
}

/* ============================================================
 *  SENSOR DATA HANDLER
 * ============================================================ */

static void handle_sensor_data(NodeEntry_t *node, const SensorDataPayload_t *data)
{
    node->last_data_ms = HAL_GetTick();
    node->data_valid   = true;
    memcpy(&node->latest_data, data, sizeof(SensorDataPayload_t));

    /* ---- In-range checks ---- */

    if (data->temperature_c < thresholds.temp_min_c100 ||
        data->temperature_c > thresholds.temp_max_c100) {
        debug_log("[FAULT] Node 0x%02X: Temp out of range: %.2f°C\r\n",
                  node->id, FIXED_TO_FLOAT(data->temperature_c));
        FaultPayload_t fp = {
            .fault_code      = FAULT_SENSOR_OUT_OF_RANGE,
            .severity        = (data->temperature_c > thresholds.temp_crit_c100) ? 3 : 2,
            .fault_timestamp = HAL_GetTick(),
            .fault_value     = data->temperature_c,
            .threshold_value = thresholds.temp_max_c100,
        };
        handle_fault_report(node, &fp);
    }

    if (data->vibration_mg > thresholds.vibration_max_mg) {
        debug_log("[FAULT] Node 0x%02X: Vibration critical: %d mg\r\n",
                  node->id, data->vibration_mg);
        FaultPayload_t fp = {
            .fault_code  = FAULT_VIBRATION_HIGH,
            .severity    = 3,
            .fault_value = data->vibration_mg,
            .threshold_value = thresholds.vibration_max_mg,
        };
        handle_fault_report(node, &fp);
    }

    if (data->voltage_mv < thresholds.voltage_min_mv ||
        data->voltage_mv > thresholds.voltage_max_mv) {
        debug_log("[FAULT] Node 0x%02X: Power anomaly: %d mV\r\n",
                  node->id, data->voltage_mv);
        FaultPayload_t fp = {
            .fault_code  = FAULT_POWER_ANOMALY,
            .severity    = 2,
            .fault_value = data->voltage_mv,
        };
        handle_fault_report(node, &fp);
    }

    if (data->gas_ppm >= thresholds.gas_crit_ppm) {
        debug_log("[FAULT] Node 0x%02X: GAS LEAK CRITICAL: %d ppm\r\n",
                  node->id, data->gas_ppm);
        FaultPayload_t fp = {
            .fault_code  = FAULT_GAS_LEAK,
            .severity    = 3,
            .fault_value = data->gas_ppm,
            .threshold_value = thresholds.gas_crit_ppm,
        };
        handle_fault_report(node, &fp);
    }

    debug_log("[DATA] Node 0x%02X | T=%.2f°C H=%.1f%% P=%luPa V=%dmg Vcc=%dmV I=%dmA Gas=%dppm\r\n",
              node->id,
              FIXED_TO_FLOAT(data->temperature_c),
              FIXED_TO_FLOAT(data->humidity_pct),
              data->pressure_pa,
              data->vibration_mg,
              data->voltage_mv,
              data->current_ma,
              data->gas_ppm);
}

/* ============================================================
 *  FAULT REPORT HANDLER
 * ============================================================ */

static void handle_fault_report(NodeEntry_t *node, const FaultPayload_t *fault)
{
    node->active_fault   = (FaultCode_t)fault->fault_code;
    node->fault_severity = fault->severity;

    if (node->state != NODE_STATE_FAULT && node->state != NODE_STATE_RECOVERING) {
        node->state              = NODE_STATE_FAULT;
        node->recovery_attempts  = 0;
        debug_log("[RECOVERY] Starting recovery for node 0x%02X fault=0x%02X sev=%d\r\n",
                  node->id, fault->fault_code, fault->severity);
        trigger_recovery(node, select_recovery_action(node));
    }
}

/* ============================================================
 *  FAULT DETECTION TICK (timeout-based)
 * ============================================================ */

static void fault_detection_tick(void)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < MAX_NODES; i++) {
        NodeEntry_t *node = &node_registry[i];
        if (!node->registered) continue;
        if (node->state == NODE_STATE_OFFLINE) continue;

        /* Heartbeat timeout check */
        uint32_t hb_age = now - node->last_heartbeat_ms;

        if (hb_age >= HEARTBEAT_DEAD_MS) {
            if (node->state != NODE_STATE_OFFLINE) {
                debug_log("[FAULT] Node 0x%02X OFFLINE — no heartbeat for %lums\r\n",
                          node->id, hb_age);
                node->state        = NODE_STATE_OFFLINE;
                node->active_fault = FAULT_NODE_OFFLINE;
                /* Last-resort: try to ping with recovery reset cmd */
                trigger_recovery(node, RECOVERY_SOFT_RESET);
            }
        } else if (hb_age >= HEARTBEAT_TIMEOUT_MS) {
            node->missed_heartbeats++;
            debug_log("[WARN] Node 0x%02X heartbeat late (%lums, miss=%d)\r\n",
                      node->id, hb_age, node->missed_heartbeats);
        }

        /* Sensor data timeout check */
        if (node->state == NODE_STATE_ACTIVE) {
            uint32_t data_age = now - node->last_data_ms;
            if (node->data_valid && data_age >= SENSOR_REPORT_TIMEOUT_MS) {
                debug_log("[FAULT] Node 0x%02X: No sensor data for %lums\r\n",
                          node->id, data_age);
                FaultPayload_t fp = {
                    .fault_code  = FAULT_SENSOR_TIMEOUT,
                    .severity    = 2,
                    .fault_timestamp = now,
                };
                handle_fault_report(node, &fp);
            }
        }

        /* Recovery escalation check */
        if (node->state == NODE_STATE_RECOVERING) {
            uint32_t recovery_age = now - node->recovery_start_ms;
            if (recovery_age >= (RECOVERY_BACKOFF_MS * (node->recovery_attempts + 1))) {
                escalate_recovery(node);
            }
        }
    }
}

/* ============================================================
 *  RECOVERY ENGINE
 * ============================================================ */

static RecoveryCmd_t select_recovery_action(NodeEntry_t *node)
{
    switch (node->active_fault) {
        case FAULT_SENSOR_TIMEOUT:
        case FAULT_SENSOR_OUT_OF_RANGE:
            return RECOVERY_RESET_SENSOR;

        case FAULT_TEMP_CRITICAL:
            return RECOVERY_ENTER_SAFE;

        case FAULT_VIBRATION_HIGH:
            return RECOVERY_RECALIBRATE;

        case FAULT_POWER_ANOMALY:
            return RECOVERY_ENTER_SAFE;

        case FAULT_GAS_LEAK:
            return RECOVERY_ENTER_SAFE;

        case FAULT_COMM_ERROR:
        case FAULT_CHECKSUM_ERROR:
            return RECOVERY_SOFT_RESET;

        case FAULT_NODE_OFFLINE:
            return RECOVERY_SOFT_RESET;

        case FAULT_MEMORY_OVERFLOW:
            return RECOVERY_SOFT_RESET;

        default:
            return RECOVERY_CLEAR_FAULT;
    }
}

static void trigger_recovery(NodeEntry_t *node, RecoveryCmd_t cmd)
{
    RecoveryCmdPayload_t payload = {
        .recovery_cmd = (uint8_t)cmd,
        .param        = 0,
    };

    node->state              = NODE_STATE_RECOVERING;
    node->recovery_start_ms  = HAL_GetTick();
    node->recovery_attempts++;

    debug_log("[RECOVERY] → Node 0x%02X cmd=0x%02X attempt=%d\r\n",
              node->id, cmd, node->recovery_attempts);

    send_frame(&huart1, node->id, MSG_RECOVERY_CMD,
               &payload, sizeof(RecoveryCmdPayload_t));
}

static void escalate_recovery(NodeEntry_t *node)
{
    if (node->recovery_attempts >= RECOVERY_RETRY_MAX) {
        debug_log("[RECOVERY] Node 0x%02X ESCALATE to factory reset (attempts=%d)\r\n",
                  node->id, node->recovery_attempts);
        trigger_recovery(node, RECOVERY_FACTORY_RESET);
        node->state = NODE_STATE_OFFLINE;   /* Assume lost until it re-registers */
        return;
    }

    /* Progressive escalation */
    RecoveryCmd_t next;
    if (node->recovery_attempts == 1)      next = RECOVERY_SOFT_RESET;
    else if (node->recovery_attempts == 2) next = RECOVERY_RECALIBRATE;
    else                                    next = RECOVERY_FACTORY_RESET;

    debug_log("[RECOVERY] Node 0x%02X escalating to cmd=0x%02X\r\n", node->id, next);
    trigger_recovery(node, next);
}

static void handle_recovery_status(NodeEntry_t *node, const uint8_t *payload)
{
    uint8_t success = payload[0];
    if (success) {
        debug_log("[RECOVERY] Node 0x%02X recovery SUCCESS\r\n", node->id);
        node->state             = NODE_STATE_ACTIVE;
        node->active_fault      = FAULT_NONE;
        node->recovery_attempts = 0;
    } else {
        debug_log("[RECOVERY] Node 0x%02X recovery FAILED\r\n", node->id);
        escalate_recovery(node);
    }
}

/* ============================================================
 *  CLOUD FORWARD (→ ESP32)
 * ============================================================ */

static void cloud_forward_tick(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - last_cloud_forward_ms) < CLOUD_FORWARD_INTERVAL_MS) return;
    last_cloud_forward_ms = now;

    /* Pack all valid node data into one cloud payload and send to ESP32 */
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        NodeEntry_t *node = &node_registry[i];
        if (!node->registered || !node->data_valid) continue;

        send_frame(&huart2, NODE_ESP32_GW, MSG_CLOUD_FORWARD,
                   &node->latest_data, sizeof(SensorDataPayload_t));
    }
}

/* ============================================================
 *  NODE REGISTRY
 * ============================================================ */

static NodeEntry_t *registry_find(uint8_t id)
{
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (node_registry[i].registered && node_registry[i].id == id)
            return &node_registry[i];
    }
    return NULL;
}

static NodeEntry_t *registry_register(const NodeRegisterPayload_t *reg)
{
    /* Update if already registered */
    NodeEntry_t *existing = registry_find(reg->node_id);
    if (existing) {
        existing->state = NODE_STATE_ACTIVE;
        existing->last_heartbeat_ms = HAL_GetTick();
        return existing;
    }

    /* Find free slot */
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (!node_registry[i].registered) {
            NodeEntry_t *n = &node_registry[i];
            memset(n, 0, sizeof(NodeEntry_t));
            n->id           = reg->node_id;
            n->hw_version   = reg->hw_version;
            n->fw_major     = reg->fw_version_major;
            n->fw_minor     = reg->fw_version_minor;
            n->capabilities = reg->capabilities;
            n->state        = NODE_STATE_ACTIVE;
            n->registered   = true;
            n->last_heartbeat_ms = HAL_GetTick();
            memcpy(n->name, reg->node_name, 16);
            node_count++;
            return n;
        }
    }
    return NULL;   /* Registry full */
}

/* ============================================================
 *  UART SEND HELPER
 * ============================================================ */

static void send_frame(UART_HandleTypeDef *uart, uint8_t dst,
                       uint8_t msg_type, const void *payload, uint8_t len)
{
    NetworkFrame_t frame;
    uint8_t        wire_buf[sizeof(NetworkFrame_t)];

    frame_encode(&frame, CENTRAL_NODE_ID, dst, msg_type, payload, len);
    int wire_len = frame_serialize(&frame, wire_buf, sizeof(wire_buf));
    if (wire_len > 0)
        HAL_UART_Transmit(uart, wire_buf, (uint16_t)wire_len, 100);
}

/* ============================================================
 *  STATUS LEDs
 * ============================================================ */

static void status_leds_update(void)
{
    bool any_fault = false;
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (node_registry[i].registered &&
            (node_registry[i].state == NODE_STATE_FAULT ||
             node_registry[i].state == NODE_STATE_OFFLINE)) {
            any_fault = true;
            break;
        }
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);    /* STATUS always ON  */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,
        any_fault ? GPIO_PIN_SET : GPIO_PIN_RESET);         /* FAULT LED         */
}

/* ============================================================
 *  UART IDLE-LINE DMA CALLBACKS
 * ============================================================ */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        rx1_len   = Size;
        rx1_ready = true;
    } else if (huart->Instance == USART2) {
        rx2_len   = Size;
        rx2_ready = true;
    }
}

/* ============================================================
 *  DEBUG LOG (USART3 → PC)
 * ============================================================ */

#include <stdarg.h>
static void debug_log(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(log_buf, LOG_BUF_SIZE, fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart3, (uint8_t *)log_buf,
                      (uint16_t)strlen(log_buf), 50);
}

/* ============================================================
 *  PERIPHERAL INIT STUBS (implement per your BSP)
 * ============================================================ */

static void system_clock_config(void)
{
    /* Configure 168 MHz HSE PLL on STM32F4
     * Implement with STM32CubeMX-generated code */
}

static void uart_init(void)
{
    /* USART1: Sensor bus  115200 8N1
     * USART2: ESP32 link  115200 8N1
     * USART3: Debug       115200 8N1
     * Enable DMA streams for USART1 and USART2 RX */
}

static void gpio_init(void)
{
    /* PA5 → STATUS LED (output push-pull)
     * PA6 → FAULT LED  (output push-pull)  */
}
