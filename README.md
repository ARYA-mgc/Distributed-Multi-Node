# Distributed Multi-Node Sensor Network with Fault Detection & Self-Recovery

> **Industrial IoT — STM32 + ESP32**
> Author: **ARYA mgc**

---

## Overview

A production-grade distributed sensor network built on **STM32** (sensor and central nodes) and **ESP32** (WiFi cloud gateway). The network uses a custom **UART frame protocol** with CRC-16 integrity, automatic **fault detection**, and a tiered **self-recovery engine** — all without an RTOS.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     DISTRIBUTED SENSOR NETWORK                          │
│                                                                         │
│  ┌──────────────┐  UART   ┌─────────────────────┐  UART  ┌──────────┐  │
│  │ Sensor Node 1│◄───────►│                     │◄──────►│  ESP32   │  │
│  │  (STM32)     │         │   Central Node      │        │ Gateway  │◄─┼──► WiFi → MQTT
│  ├──────────────┤         │   (STM32 Master)    │        │          │  │              → HTTP API
│  │ Sensor Node 2│◄───────►│                     │        └──────────┘  │
│  │  (STM32)     │         │  • Fault Detection  │                      │
│  ├──────────────┤         │  • Recovery Engine  │                      │
│  │ Sensor Node 3│◄───────►│  • Node Registry    │                      │
│  │  (STM32)     │         │  • Cloud Bridge     │                      │
│  └──────────────┘         └─────────────────────┘                      │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```
sensor-network/
├── src/
│   ├── common/
│   │   ├── network_protocol.h   # Frame format, message types, payload structs
│   │   ├── frame_codec.h        # Encoder / decoder / serializer
│   │   └── crc16.h              # CRC-16/CCITT implementation
│   ├── central_node/
│   │   └── central_node_main.c  # STM32 master: fault detection + recovery
│   ├── sensor_node/
│   │   └── sensor_node_main.c   # STM32 sensor node (generic template)
│   └── esp32/
│       └── esp32_gateway.ino    # ESP32: WiFi bridge, MQTT, HTTP REST API
├── tools/
│   └── sensor_net_monitor.py    # Python live dashboard (Rich TUI)
└── README.md
```

---

## Protocol

### UART Frame Format

```
| START | SRC_ID | DST_ID | MSG_TYPE | SEQ(2) | LEN | PAYLOAD[0..64] | CRC16(2) | END |
|  0xAA |   1B   |   1B   |    1B    |   2B   |  1B |     0..64B     |    2B    | 0x55|
```

- **CRC-16/CCITT-FALSE** (poly `0x1021`, init `0xFFFF`) over header + payload
- Sequence numbers for duplicate detection
- Broadcast address `0xFF` supported
- Max payload: 64 bytes

### Message Types

| Type | Hex | Direction | Description |
|------|-----|-----------|-------------|
| `MSG_SENSOR_DATA` | `0x10` | Sensor → Central | Periodic sensor readings |
| `MSG_HEARTBEAT` | `0x20` | Sensor → Central | Node alive signal |
| `MSG_FAULT_REPORT` | `0x30` | Sensor → Central | Detected fault |
| `MSG_RECOVERY_CMD` | `0x40` | Central → Sensor | Recovery action |
| `MSG_NODE_REGISTER` | `0x60` | Sensor → Central | Node joining network |
| `MSG_CLOUD_FORWARD` | `0x70` | Central → ESP32 | Data to cloud |

---

## Fault Detection

The central node runs `fault_detection_tick()` every loop iteration and detects:

| Fault | Detection Method | Severity |
|-------|-----------------|----------|
| `FAULT_SENSOR_TIMEOUT` | No data in 5000ms | Medium |
| `FAULT_SENSOR_OUT_OF_RANGE` | Reading outside threshold | Medium/Critical |
| `FAULT_TEMP_CRITICAL` | Temperature > 75°C | Critical |
| `FAULT_VIBRATION_HIGH` | Vibration > 2000 mg | Critical |
| `FAULT_POWER_ANOMALY` | Voltage outside 4.75–5.25V | Medium |
| `FAULT_GAS_LEAK` | Gas > 1000 ppm | Critical |
| `FAULT_NODE_OFFLINE` | No heartbeat for 9000ms | Critical |

### Sensor Thresholds

| Parameter | Min | Max | Critical |
|-----------|-----|-----|----------|
| Temperature | -40°C | +85°C | +75°C |
| Pressure | 80000 Pa | 110000 Pa | — |
| Vibration | — | 2000 mg | — |
| Voltage | 4750 mV | 5250 mV | — |
| Current | — | 2000 mA | — |
| Gas | — | 400 ppm (warn) | 1000 ppm |

---

## Self-Recovery Engine

Tiered escalation with backoff — no human intervention required:

```
Fault Detected
     │
     ▼
Attempt 1: Select action (RESET_SENSOR / ENTER_SAFE / SOFT_RESET)
     │  (wait BACKOFF * 1)
     ▼
Attempt 2: SOFT_RESET
     │  (wait BACKOFF * 2)
     ▼
Attempt 3: RECALIBRATE
     │  (wait BACKOFF * 3)
     ▼
Attempt 4: FACTORY_RESET → mark OFFLINE
```

### Recovery Commands

| Command | Action |
|---------|--------|
| `RECOVERY_RESET_SENSOR` | Re-initialize I2C sensor peripheral |
| `RECOVERY_SOFT_RESET` | `NVIC_SystemReset()` on the sensor node |
| `RECOVERY_RECALIBRATE` | Trigger sensor self-calibration sequence |
| `RECOVERY_ENTER_SAFE` | Reduce sampling rate, idle non-critical paths |
| `RECOVERY_EXIT_SAFE` | Resume normal operation |
| `RECOVERY_FACTORY_RESET` | Clear config, soft reset — last resort |

---

## Hardware Setup

### STM32 Central Node (e.g. STM32F407VGT6)

| Pin | Function |
|-----|----------|
| USART1 TX/RX | Sensor network bus |
| USART2 TX/RX | ESP32 gateway link |
| USART3 TX/RX | Debug / PC terminal |
| PA5 | STATUS LED (always on when running) |
| PA6 | FAULT LED (on when any node is in fault/offline) |

### STM32 Sensor Nodes (e.g. STM32F103C8T6 Blue Pill)

| Pin | Function |
|-----|----------|
| USART1 TX/RX | Network bus to central |
| I2C1 SDA/SCL | BME280 (temp/humidity/pressure) + MPU6050 (vibration) |
| ADC1 CH0 | Voltage divider sense |
| ADC1 CH1 | Current shunt sense |
| ADC2 CH0 | MQ-2 gas sensor |

### ESP32 Gateway

| Pin | Function |
|-----|----------|
| GPIO16 (RX2) | UART from STM32 central node TX |
| GPIO17 (TX2) | UART to STM32 central node RX |
| WiFi | MQTT broker + HTTP REST API |

### UART Wiring (RS-485 or direct)

```
STM32 Sensor Node 1 TX ──────────────────────────┐
STM32 Sensor Node 2 TX ──────────────────────────┤
STM32 Sensor Node 3 TX ──────────────────────────┼──► USART1 RX (Central)
                                                   │
STM32 Central USART1 TX ──────────────────────────┼──► All Sensor Node RX (broadcast)
```

> ⚠️ For more than 2 nodes on a shared bus, use RS-485 transceivers (e.g. MAX485) with direction control on a DE/RE pin.

---

## Getting Started

### 1. Central Node

1. Open `src/central_node/central_node_main.c` in STM32CubeIDE
2. Include `src/common/` in your include path
3. Implement the peripheral init stubs (`uart_init`, `gpio_init`, etc.) using STM32CubeMX-generated code for your board
4. Flash to your STM32F4xx board

### 2. Sensor Nodes

1. Open `src/sensor_node/sensor_node_main.c`
2. Set `THIS_NODE_ID`, `THIS_NODE_NAME`, and `THIS_CAPABILITIES` at the top
3. Replace sensor driver stubs (`bme280_read_temperature()` etc.) with your actual I2C/ADC drivers
4. Flash a separate binary to each sensor STM32 board

### 3. ESP32 Gateway

1. Open `src/esp32/esp32_gateway.ino` in Arduino IDE
2. Set `WIFI_SSID`, `WIFI_PASSWORD`, and `MQTT_BROKER`
3. Install required libraries: `PubSubClient`, `ArduinoJson`
4. Flash to ESP32-DevKitC

### 4. Monitor

```bash
pip install pyserial rich
python tools/sensor_net_monitor.py --port /dev/ttyUSB0 --baud 115200
```

---

## MQTT Topics

| Topic | Direction | Content |
|-------|-----------|---------|
| `sensor_network/data/node{XX}` | GW → Broker | JSON sensor reading |
| `sensor_network/fault` | GW → Broker | JSON fault alert |
| `sensor_network/status` | GW → Broker | Gateway health (every 30s) |
| `sensor_network/cmd` | Broker → GW | Recovery command `{"node":1,"recovery":2}` |

---

## Dependencies

### STM32 (HAL)
- STM32Cube HAL (any STM32 family — adapt accordingly)
- No external libraries required

### ESP32 (Arduino)
- [PubSubClient](https://github.com/knolleary/pubsubclient) — MQTT client
- [ArduinoJson](https://arduinojson.org/) — JSON serialization

### Python Monitor
- `pyserial` — Serial communication
- `rich` — Terminal UI (optional but recommended)

---

## License

MIT License — © 2025 ARYA mgc

---

*Built as part of an Industrial IoT embedded systems project.*
*Framework inspired by production-grade sensor fusion and telemetry architectures.*
