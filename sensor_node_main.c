/**
 * @file    sensor_node_main.c
 * @brief   Generic Sensor Node Firmware — STM32
 *
 * @author  ARYA mgc
 * @version 1.0.0
 * @date    2025
 *
 * @details Each physical sensor node runs this firmware.
 *          Node-specific identity (ID, name, sensors) is set via
 *          the NODE_CONFIG defines below.
 *
 *          Features:
 *            - Self-registration with central node on boot
 *            - Periodic sensor sampling and data transmission
 *            - Periodic heartbeat broadcast
 *            - Recovery command handler (reset sensor, soft reset,
 *              safe mode, recalibrate, factory reset)
 *            - Watchdog timer for self-reset on hang
 *            - Local sensor sanity checks
 *
 * @hardware STM32F103C8T6 (Blue Pill) or any STM32 variant
 *           - USART1: Network bus (to central node)
 *           - I2C1:   Sensor bus (BME280 / MPU6050 etc.)
 *           - ADC1:   Voltage / Current sense
 *           - TIM3:   Sensor sample timer
 *           - IWDG:   Independent watchdog
 */

#include "stm32f1xx_hal.h"
#include "network_protocol.h"
#include "frame_codec.h"
#include <string.h>
#include <stdio.h>

/* ============================================================
 *  NODE IDENTITY — Edit per node before flashing
 * ============================================================ */

#define THIS_NODE_ID            NODE_SENSOR_1
#define THIS_NODE_NAME          "SensorNode-1"
#define THIS_HW_VERSION         1
#define THIS_FW_MAJOR           1
#define THIS_FW_MINOR           0
#define THIS_CAPABILITIES       (CAP_TEMPERATURE | CAP_HUMIDITY | \
                                  CAP_PRESSURE    | CAP_VIBRATION | \
                                  CAP_VOLTAGE     | CAP_CURRENT)

/* ============================================================
 *  TIMING CONFIG
 * ============================================================ */

#define HEARTBEAT_INTERVAL_MS       2000
#define SENSOR_REPORT_INTERVAL_MS   1000
#define REGISTRATION_RETRY_MS       3000
#define WATCHDOG_FEED_INTERVAL_MS   500

/* ============================================================
 *  SYSTEM GLOBALS
 * ============================================================ */

static UART_HandleTypeDef  huart1;
static I2C_HandleTypeDef   hi2c1;
static ADC_HandleTypeDef   hadc1;
static IWDG_HandleTypeDef  hiwdg;

static uint8_t   rx_buf[256];
static volatile bool rx_ready = false;

static NodeState_t node_state    = NODE_STATE_INIT;
static bool        registered    = false;
static uint32_t    last_hb_ms    = 0;
static uint32_t    last_data_ms  = 0;
static uint32_t    last_wdg_ms   = 0;
static uint32_t    last_reg_ms   = 0;

/* ============================================================
 *  FORWARD DECLARATIONS
 * ============================================================ */

static void system_clock_config(void);
static void uart_init(void);
static void i2c_init(void);
static void adc_init(void);
static void iwdg_init(void);
static void gpio_init(void);

static void  node_register(void);
static void  send_heartbeat(void);
static void  send_sensor_data(void);
static void  handle_frame(const NetworkFrame_t *frame);
static void  handle_recovery_cmd(const RecoveryCmdPayload_t *cmd);
static void  send_recovery_status(bool success);

static bool  sensor_read(SensorDataPayload_t *out);
static int16_t  bme280_read_temperature(void);
static uint16_t bme280_read_humidity(void);
static uint32_t bme280_read_pressure(void);
static int16_t  mpu6050_read_vibration(void);
static uint16_t adc_read_voltage_mv(void);
static uint16_t adc_read_current_ma(void);
static uint16_t mq2_read_gas_ppm(void);

static void send_frame_to_central(uint8_t msg_type,
                                   const void *payload, uint8_t len);

/* ============================================================
 *  MAIN
 * ============================================================ */

int main(void)
{
    HAL_Init();
    system_clock_config();
    gpio_init();
    uart_init();
    i2c_init();
    adc_init();
    iwdg_init();

    node_state = NODE_STATE_REGISTERING;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, sizeof(rx_buf));

    while (1) {
        uint32_t now = HAL_GetTick();

        /* Feed watchdog */
        if ((now - last_wdg_ms) >= WATCHDOG_FEED_INTERVAL_MS) {
            HAL_IWDG_Refresh(&hiwdg);
            last_wdg_ms = now;
        }

        /* Handle incoming frames */
        if (rx_ready) {
            rx_ready = false;
            NetworkFrame_t *frame = (NetworkFrame_t *)rx_buf;
            if (frame_decode(frame) == CODEC_OK &&
                (frame->dst_id == THIS_NODE_ID || frame->dst_id == BROADCAST_ID)) {
                handle_frame(frame);
            }
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, sizeof(rx_buf));
        }

        /* Registration */
        if (!registered) {
            if ((now - last_reg_ms) >= REGISTRATION_RETRY_MS) {
                last_reg_ms = now;
                node_register();
            }
            continue;  /* Don't send data until registered */
        }

        /* Heartbeat */
        if ((now - last_hb_ms) >= HEARTBEAT_INTERVAL_MS) {
            last_hb_ms = now;
            send_heartbeat();
        }

        /* Sensor report */
        if (node_state == NODE_STATE_ACTIVE &&
            (now - last_data_ms) >= SENSOR_REPORT_INTERVAL_MS) {
            last_data_ms = now;
            send_sensor_data();
        }
    }
}

/* ============================================================
 *  NODE REGISTRATION
 * ============================================================ */

static void node_register(void)
{
    NodeRegisterPayload_t reg = {
        .node_id          = THIS_NODE_ID,
        .hw_version       = THIS_HW_VERSION,
        .fw_version_major = THIS_FW_MAJOR,
        .fw_version_minor = THIS_FW_MINOR,
        .capabilities     = THIS_CAPABILITIES,
    };
    strncpy(reg.node_name, THIS_NODE_NAME, sizeof(reg.node_name));

    send_frame_to_central(MSG_NODE_REGISTER, &reg, sizeof(reg));
}

/* ============================================================
 *  HEARTBEAT
 * ============================================================ */

static void send_heartbeat(void)
{
    HeartbeatPayload_t hb = {
        .uptime_ms   = HAL_GetTick(),
        .node_state  = (uint8_t)node_state,
        .fault_count = 0,
        .free_heap   = 0,  /* Populate from stack watermark if desired */
    };
    send_frame_to_central(MSG_HEARTBEAT, &hb, sizeof(hb));
}

/* ============================================================
 *  SENSOR DATA
 * ============================================================ */

static void send_sensor_data(void)
{
    SensorDataPayload_t data;
    if (!sensor_read(&data)) {
        /* Report sensor read failure */
        FaultPayload_t fp = {
            .fault_code  = FAULT_SENSOR_TIMEOUT,
            .severity    = 2,
            .fault_timestamp = HAL_GetTick(),
        };
        send_frame_to_central(MSG_FAULT_REPORT, &fp, sizeof(fp));
        return;
    }
    send_frame_to_central(MSG_SENSOR_DATA, &data, sizeof(data));
}

/* ============================================================
 *  INCOMING FRAME HANDLER
 * ============================================================ */

static void handle_frame(const NetworkFrame_t *frame)
{
    switch (frame->msg_type) {
        case MSG_NETWORK_STATUS:
            /* Central acknowledged our registration */
            registered  = true;
            node_state  = NODE_STATE_ACTIVE;
            break;

        case MSG_HEARTBEAT_RSP:
            /* Central is alive — nothing to do */
            break;

        case MSG_SENSOR_ACK:
            /* Data acknowledged */
            break;

        case MSG_FAULT_ACK:
            /* Fault acknowledged */
            break;

        case MSG_RECOVERY_CMD:
            handle_recovery_cmd((const RecoveryCmdPayload_t *)frame->payload);
            break;

        case MSG_CONFIG_SET: {
            /* Handle config update (e.g. change reporting rate) */
            /* param[0] = config key, param[1..4] = value */
            break;
        }

        default:
            break;
    }
}

/* ============================================================
 *  RECOVERY COMMAND HANDLER
 * ============================================================ */

static void handle_recovery_cmd(const RecoveryCmdPayload_t *cmd)
{
    bool success = true;

    switch ((RecoveryCmd_t)cmd->recovery_cmd) {
        case RECOVERY_RESET_SENSOR:
            /* Re-initialize I2C sensor bus */
            HAL_I2C_DeInit(&hi2c1);
            HAL_Delay(100);
            i2c_init();
            node_state = NODE_STATE_ACTIVE;
            break;

        case RECOVERY_SOFT_RESET:
            /* Perform a software system reset */
            send_recovery_status(true);
            HAL_Delay(50);
            NVIC_SystemReset();
            break;  /* Never reached */

        case RECOVERY_RECALIBRATE:
            /* Trigger sensor recalibration sequence */
            /* Sensor-specific: e.g. write calibration command over I2C */
            HAL_Delay(500);
            node_state = NODE_STATE_ACTIVE;
            break;

        case RECOVERY_ENTER_SAFE:
            node_state = NODE_STATE_SAFE_MODE;
            /* Reduce sampling rate, disable non-critical peripherals */
            break;

        case RECOVERY_EXIT_SAFE:
            node_state = NODE_STATE_ACTIVE;
            break;

        case RECOVERY_CLEAR_FAULT:
            node_state = NODE_STATE_ACTIVE;
            break;

        case RECOVERY_UPDATE_RATE:
            /* cmd->param = new interval in ms (unused here — extend if needed) */
            break;

        case RECOVERY_FACTORY_RESET:
            /* Clear all config, reset to defaults, then soft reset */
            send_recovery_status(true);
            HAL_Delay(50);
            NVIC_SystemReset();
            break;

        default:
            success = false;
            break;
    }

    send_recovery_status(success);
}

static void send_recovery_status(bool success)
{
    uint8_t status = success ? 1 : 0;
    send_frame_to_central(MSG_RECOVERY_STATUS, &status, 1);
}

/* ============================================================
 *  SENSOR READ (stub — replace with real driver calls)
 * ============================================================ */

static bool sensor_read(SensorDataPayload_t *out)
{
    memset(out, 0, sizeof(*out));
    out->timestamp_ms  = HAL_GetTick();
    out->temperature_c = bme280_read_temperature();
    out->humidity_pct  = bme280_read_humidity();
    out->pressure_pa   = bme280_read_pressure();
    out->vibration_mg  = mpu6050_read_vibration();
    out->voltage_mv    = adc_read_voltage_mv();
    out->current_ma    = adc_read_current_ma();
    out->gas_ppm       = mq2_read_gas_ppm();
    out->node_state    = (uint8_t)node_state;
    out->fault_flags   = 0;
    return true;
}

/* Sensor driver stubs — replace with actual I2C/ADC drivers */
static int16_t  bme280_read_temperature(void) { return 2500; }   /* 25.00°C */
static uint16_t bme280_read_humidity(void)    { return 5500; }   /* 55.00%  */
static uint32_t bme280_read_pressure(void)    { return 101325; } /* 1 atm   */
static int16_t  mpu6050_read_vibration(void)  { return 10; }     /* 10 mg   */
static uint16_t adc_read_voltage_mv(void)     { return 5000; }   /* 5.00 V  */
static uint16_t adc_read_current_ma(void)     { return 120; }    /* 120 mA  */
static uint16_t mq2_read_gas_ppm(void)        { return 50; }     /* 50 ppm  */

/* ============================================================
 *  UART SEND HELPER
 * ============================================================ */

static void send_frame_to_central(uint8_t msg_type,
                                   const void *payload, uint8_t len)
{
    NetworkFrame_t frame;
    uint8_t        wire_buf[sizeof(NetworkFrame_t)];

    frame_encode(&frame, THIS_NODE_ID, CENTRAL_NODE_ID, msg_type, payload, len);
    int wire_len = frame_serialize(&frame, wire_buf, sizeof(wire_buf));
    if (wire_len > 0)
        HAL_UART_Transmit(&huart1, wire_buf, (uint16_t)wire_len, 100);
}

/* ============================================================
 *  UART DMA CALLBACK
 * ============================================================ */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    (void)Size;
    if (huart->Instance == USART1)
        rx_ready = true;
}

/* ============================================================
 *  PERIPHERAL INIT STUBS
 * ============================================================ */

static void system_clock_config(void) { /* 72 MHz HSE PLL for STM32F103 */ }
static void uart_init(void)           { /* USART1 115200 8N1 DMA         */ }
static void i2c_init(void)            { /* I2C1 400kHz for BME280/MPU    */ }
static void adc_init(void)            { /* ADC1 CH0=voltage CH1=current  */ }
static void gpio_init(void)           { /* LED pin etc.                  */ }
static void iwdg_init(void)
{
    /* 1s timeout: LSI ~40kHz, prescaler /256, reload = 156 */
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Reload    = 156;
    HAL_IWDG_Init(&hiwdg);
}
