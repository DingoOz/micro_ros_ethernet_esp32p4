#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

#include <uros_network_interfaces.h>

#include "pca9685.h"
#include "eth_transport.h"
// #include "wifi_transport.h"  // TODO: re-enable after C6 slave firmware
#include "webserver.h"
#include "system_stats.h"
#include "radio.h"
#include "rgb_led.h"
#include "ultrasonic.h"
#include "ssd1306.h"
#include "slide_pot.h"

static const char *TAG = "main";

#define NUM_SERVOS 4

// micro-ROS objects
static rcl_publisher_t publisher;
static std_msgs__msg__Int32 pub_msg;
static rcl_subscription_t servo_subs[NUM_SERVOS];
static std_msgs__msg__Float32 servo_msgs[NUM_SERVOS];
static int servo_channels[NUM_SERVOS] = {0, 1, 2, 3};

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    rcl_publish(&publisher, &pub_msg, NULL);
    pub_msg.data++;
    webserver_get_stats()->publish_count = pub_msg.data;
}

static void servo_callback(const void *msg_in, void *context)
{
    const std_msgs__msg__Float32 *servo_msg = (const std_msgs__msg__Float32 *)msg_in;
    int channel = *(int *)context;
    float angle = servo_msg->data;

    ESP_LOGI(TAG, "Servo %d -> %.1f deg", channel, angle);
    pca9685_set_servo_angle((uint8_t)channel, angle);
    if (channel >= 0 && channel < NUM_SERVOS) {
        webserver_get_stats()->servo_angles[channel] = angle;
    }
}

static void micro_ros_task(void *arg)
{
    (void)arg;

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_timer_t timer;
    rclc_executor_t executor;

    // Wait for agent — ping for up to 2 minutes
    ESP_LOGI(TAG, "Waiting for micro-ROS agent...");
    const int timeout_ms = 1000;
    const uint8_t attempts = 30;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "micro-ROS agent unreachable, task exiting");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "micro-ROS agent connected");

    // Init support with domain ID
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, CONFIG_MICRO_ROS_DOMAIN_ID);
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // Node
    rclc_node_init_default(&node, "esp32p4_ethernet_node", "", &support);

    // Publisher: /pico_publisher (Int32, 1 Hz)
    rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

    // Timer: 1 Hz
    rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(1000), timer_callback, true);

    // Subscriptions: /servo_0 .. /servo_3
    char topic_name[16];
    for (int i = 0; i < NUM_SERVOS; i++) {
        snprintf(topic_name, sizeof(topic_name), "servo_%d", i);
        rclc_subscription_init_default(
            &servo_subs[i], &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            topic_name);
    }

    // Executor: 1 timer + 4 subscriptions = 5 handles
    rclc_executor_init(&executor, &support.context, 1 + NUM_SERVOS, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    for (int i = 0; i < NUM_SERVOS; i++) {
        rclc_executor_add_subscription_with_context(
            &executor, &servo_subs[i], &servo_msgs[i],
            servo_callback, &servo_channels[i], ON_NEW_DATA);
    }

    pub_msg.data = 0;
    ESP_LOGI(TAG, "micro-ROS spinning on core %d", xPortGetCoreID());

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    // NVS init (required by some ESP-IDF components)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "=== ESP32-P4 micro-ROS Ethernet ===");

    // Initialize system stats (temperature sensor)
    system_stats_init();

    // Initialize Ethernet and wait for IP
    if (!eth_transport_init()) {
        ESP_LOGE(TAG, "Ethernet init failed or no IP, continuing anyway...");
    }

    // TODO: Initialize WiFi STA after C6 slave firmware is flashed
    // if (!wifi_transport_init()) {
    //     ESP_LOGW(TAG, "WiFi init failed or no IP, continuing anyway...");
    // }

    // Create shared I2C master bus (used by PCA9685 and ES8311)
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = CONFIG_SERVO_I2C_SCL_PIN,
        .sda_io_num = CONFIG_SERVO_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus = NULL;
    esp_err_t i2c_err = i2c_new_master_bus(&bus_config, &i2c_bus);
    if (i2c_err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(i2c_err));
    }

    // Initialize PCA9685 servo driver (shares I2C bus)
    if (i2c_bus && !pca9685_init(i2c_bus, CONFIG_SERVO_PCA9685_ADDR)) {
        ESP_LOGW(TAG, "PCA9685 init failed - servos unavailable");
    } else if (i2c_bus) {
        // Centre all servos to 90 deg on startup
        for (int i = 0; i < NUM_SERVOS; i++) {
            pca9685_set_servo_angle(i, 90.0f);
        }
        ESP_LOGI(TAG, "Servos centred to 90 deg");
    }

    // Initialize RGB LED module (PiicoDev on same I2C bus)
    if (i2c_bus && !rgb_led_init(i2c_bus)) {
        ESP_LOGW(TAG, "RGB LED init failed - LEDs unavailable");
    }

    // Initialize ultrasonic rangefinder (PiicoDev on same I2C bus)
    if (i2c_bus && !ultrasonic_init(i2c_bus, ULTRASONIC_DEFAULT_ADDR)) {
        ESP_LOGW(TAG, "Ultrasonic init failed - rangefinder unavailable");
    }

    // Initialize SSD1306 OLED display (PiicoDev on same I2C bus)
    if (i2c_bus && !ssd1306_init(i2c_bus, SSD1306_DEFAULT_ADDR)) {
        ESP_LOGW(TAG, "SSD1306 init failed - OLED unavailable");
    } else if (i2c_bus) {
        ssd1306_clear();
        ssd1306_text_large(10, 4, "ESP32-P4");
        ssd1306_text(16, 28, "micro-ROS Dashboard");
        ssd1306_text(34, 44, "Booting...");
        ssd1306_show();
    }

    // Initialize slide potentiometer (PiicoDev on same I2C bus, ASW1 on = 0x36)
    if (i2c_bus && !slide_pot_init(i2c_bus, SLIDE_POT_DEFAULT_ADDR)) {
        ESP_LOGW(TAG, "Slide pot init failed - volume slider unavailable");
    }

    // Initialize radio (ES8311 codec on same I2C bus)
    if (i2c_bus && !radio_init(i2c_bus)) {
        ESP_LOGW(TAG, "Radio init failed - audio unavailable");
    }

    // Start web dashboard
    webserver_start(i2c_bus);

    // Spawn micro-ROS task on core 1
    xTaskCreatePinnedToCore(
        micro_ros_task, "micro_ros",
        16384, NULL, 5, NULL, 1);
}
