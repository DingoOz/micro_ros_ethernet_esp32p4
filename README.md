# micro-ROS Ethernet on ESP32-P4

micro-ROS node running on the **Waveshare ESP32-P4-WiFi6-PoE-ETH** board with native EMAC Ethernet (IP101GRI PHY), PCA9685 I2C servo control, and a web status dashboard.

Ported from [micro_ros_pico_ethernet](https://github.com/DingoOz/micro_ros_pico_ethernet) (Raspberry Pi Pico W + W5500).

## Features

- **Publisher**: `/pico_publisher` (std_msgs/Int32, 1 Hz counter)
- **Subscriptions**: `/servo_0` through `/servo_3` (std_msgs/Float32, 0-180 degrees)
- **PCA9685 servo driver**: 4-channel I2C servo control at 50 Hz
- **Web dashboard**: Dark-themed auto-refreshing HTML on port 80
  - System info (chip, CPU, temperature, flash, reset reason)
  - Memory (internal SRAM + 32MB PSRAM with progress bars)
  - Ethernet (link speed, duplex, PHY, MAC/IP)
  - micro-ROS status (publish count, servo angles)
  - FreeRTOS task table (name, core affinity, priority, stack watermark)
- **JSON API**: `GET /api/stats` returns all telemetry as JSON
- **Dual-core**: micro-ROS on core 1, networking on core 0

## Hardware

- **Board**: Waveshare ESP32-P4-WiFi6-PoE-ETH
  - ESP32-P4 dual-core RISC-V @ 400 MHz
  - 32 MB PSRAM, 32 MB Flash
  - Native EMAC + IP101GRI PHY (RMII)
  - PoE support
- **Servo driver**: PCA9685 on I2C (default SDA=GPIO7, SCL=GPIO8, addr=0x44)

## Prerequisites

- **ESP-IDF v5.4+** ([installation guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/get-started/))
- **micro-ROS agent** on the host PC
- Ethernet connection between the board and the agent machine

## Build & Flash

```bash
# Clone the project
git clone https://github.com/DingoOz/micro_ros_ethernet_esp32p4.git
cd micro_ros_ethernet_esp32p4

# Clone the micro-ROS component (jazzy branch)
git clone -b jazzy https://github.com/micro-ROS/micro_ros_espidf_component.git components/micro_ros_espidf_component

# Set target and build
idf.py set-target esp32p4
idf.py build

# Flash and monitor
idf.py flash monitor
```

## Configuration

Use `idf.py menuconfig` → "micro-ROS ESP32-P4 Configuration" to adjust:

| Setting | Default | Description |
|---------|---------|-------------|
| Agent IP | 192.168.1.241 | micro-ROS agent IP address |
| Agent Port | 8888 | micro-ROS agent UDP port |
| Domain ID | 30 | ROS 2 domain ID |
| I2C SDA | GPIO 7 | PCA9685 SDA pin |
| I2C SCL | GPIO 8 | PCA9685 SCL pin |
| PCA9685 Addr | 0x44 | I2C address |
| ETH MDC | GPIO 31 | RMII MDC pin |
| ETH MDIO | GPIO 52 | RMII MDIO pin |
| PHY Reset | GPIO 51 | PHY reset pin |
| PHY Address | 1 | MDIO bus address |
| Webserver Port | 80 | Dashboard HTTP port |

## Running the Agent

On your host PC:

```bash
# Install micro-ROS agent
sudo apt install ros-jazzy-micro-ros-agent

# Run the agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## Testing

```bash
# Check the publisher
ros2 topic echo /pico_publisher

# Move servo 0 to 45 degrees
ros2 topic pub --once /servo_0 std_msgs/msg/Float32 "data: 45.0"

# Open dashboard
xdg-open http://<device-ip>/
```

## Wiring

### PCA9685 Servo Driver

| PCA9685 Pin | ESP32-P4 GPIO |
|-------------|---------------|
| SDA | 7 |
| SCL | 8 |
| VCC | 3.3V |
| GND | GND |
| V+ | 5-6V (servo power) |

### Ethernet

Built-in EMAC with IP101GRI PHY on the Waveshare board — no external wiring needed. Connect via the RJ45 jack (PoE supported).

## License

Apache 2.0
