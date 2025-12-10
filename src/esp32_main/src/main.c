#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/touch_pad.h"
#include "driver/rmt.h"
#include "cJSON.h"
#include "my_enc28j60_phy.h"

// MicroROS Headers
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h>

static const char *TAG = "MACHANI_MAIN";

// --- Configuration ---
#define AGENT_IP "192.168.1.100" // Default Agent IP, should match Jetson
#define AGENT_PORT 8888

// Ethernet Pins (ENC28J60)
#if CONFIG_IDF_TARGET_ESP32
    #define SPI_MISO_GPIO 19
    #define SPI_MOSI_GPIO 23
    #define SPI_CLK_GPIO  18
    #define SPI_CS_GPIO   5
#else
    #error "Target not supported or pins not defined"
#endif

// Servo UART
#define SERVO_UART_NUM UART_NUM_1
#define SERVO_TX_PIN 4
#define SERVO_RX_PIN 2
#define SERVO_BAUD 1000000

// LED
#define LED_PIN GPIO_NUM_27
#define LED_RMT_CHANNEL RMT_CHANNEL_0

// I2C Battery
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define BATTERY_I2C_ADDR 0x40 // Example address (INA219 or similar)

// Touch
#define TOUCH_PAD_NUM TOUCH_PAD_NUM0 // GPIO 4 is touch0 but used for UART TX? Check pins.
// ESP32 Touch: T0=GPIO4, T1=GPIO0, T2=GPIO2, T3=MTDO/15, T4=MTCK/13, T5=MTDI/12, T6=MTMS/14, T7=27, T8=33, T9=32
// Used Pins: 
// SPI: 19, 23, 18, 5
// UART: 4 (TX), 2 (RX) -> 4 is T0, 2 is T2.
// LED: 27 -> T7.
// I2C: 22, 21.
// Available Touch: T3(15), T4(13), T5(12), T6(14), T8(33), T9(32).
#define TOUCH_PIN_NUM TOUCH_PAD_NUM8 // GPIO 33
#define TOUCH_THRESH 600 // Adjust based on calibration

// --- Globals ---
static EventGroupHandle_t eth_event_group;
const int GOT_IP_BIT = BIT0;

rcl_publisher_t pub_touch;
rcl_publisher_t pub_batt;
std_msgs__msg__String msg_touch;
std_msgs__msg__String msg_batt;

// --- Helper Functions ---

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// Servo Protocol (Simple SCServo/STS WritePos)
void send_servo_cmd(uint8_t id, int angle, int speed, int accel) {
    // Map angle (-180 to 180 or 0 to 360) to 0-4095
    // Assuming input angle is 0-360 for simplicity or mapped
    int pos = (int)((angle / 360.0f) * 4095);
    if (pos < 0) pos = 0;
    if (pos > 4095) pos = 4095;

    // Packet: FF FF ID LEN INSTR(0x03) P1(AddrL) P2(AddrH) P3(DatL) P4(DatH) ...
    // SMS/STS WritePosEx (Reg 0x2A):
    // ID LEN 0x03 0x2A 0x00 PosL PosH SpdL SpdH Acc
    uint8_t buffer[13];
    buffer[0] = 0xFF;
    buffer[1] = 0xFF;
    buffer[2] = id;
    buffer[3] = 9; // Length (Instr + Params + Checksum)
    buffer[4] = 0x03; // WRITE
    buffer[5] = 0x2A; // Reg Address
    buffer[6] = 0x00;
    buffer[7] = (uint8_t)(pos & 0xFF);
    buffer[8] = (uint8_t)((pos >> 8) & 0xFF);
    buffer[9] = (uint8_t)(speed & 0xFF);
    buffer[10] = (uint8_t)((speed >> 8) & 0xFF);
    buffer[11] = (uint8_t)(accel & 0xFF);
    
    uint8_t checksum = 0;
    for(int i = 2; i < 12; i++) checksum += buffer[i];
    buffer[12] = ~checksum;

    uart_write_bytes(SERVO_UART_NUM, (const char*)buffer, 13);
}

void parse_and_move(const char* json, uint8_t id) {
    cJSON *root = cJSON_Parse(json);
    if (root) {
        int angle = 0, speed = 1000, accel = 50;
        cJSON *j_angle = cJSON_GetObjectItem(root, "angle");
        cJSON *j_speed = cJSON_GetObjectItem(root, "speed");
        cJSON *j_accel = cJSON_GetObjectItem(root, "accel");
        
        if (j_angle) angle = j_angle->valueint;
        if (j_speed) speed = j_speed->valueint;
        if (j_accel) accel = j_accel->valueint;
        
        send_servo_cmd(id, angle, speed, accel);
        cJSON_Delete(root);
    }
}

// WS2812B Minimal (Using RMT for simplicity, just one color for state)
void set_led_state(const char* state) {
    // Determine color based on state
    uint8_t r=0, g=0, b=0;
    if (strcmp(state, "listen") == 0) { r=0; g=255; b=0; }
    else if (strcmp(state, "mute") == 0) { r=255; g=0; b=0; }
    else if (strcmp(state, "loading") == 0) { r=0; g=0; b=255; }
    else { r=50; g=50; b=50; } // Default white-ish

    // TODO: Implement full WS2812B RMT protocol
    // For now, logging state
    ESP_LOGI(TAG, "LED State: %s (R%d G%d B%d)", state, r, g, b);
}

// --- Callbacks ---

void torso_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    ESP_LOGI(TAG, "Torso Cmd: %s", msg->data.data);
    parse_and_move(msg->data.data, 1); // ID 1
}

void neck_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    ESP_LOGI(TAG, "Neck Cmd: %s", msg->data.data);
    parse_and_move(msg->data.data, 2); // ID 2
}

void head_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    ESP_LOGI(TAG, "Head Cmd: %s", msg->data.data);
    parse_and_move(msg->data.data, 3); // ID 3
}

void led_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    cJSON *root = cJSON_Parse(msg->data.data);
    if (root) {
        cJSON *state = cJSON_GetObjectItem(root, "state");
        if (state) set_led_state(state->valuestring);
        cJSON_Delete(root);
    }
}

void reb_jet_callback(const void * msgin) {
    ESP_LOGW(TAG, "Reboot Jetson Signal Received");
    // Logic to reboot jetson? Maybe toggle a GPIO connected to RST?
}

// --- Tasks ---

void micro_ros_task(void * arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;
    rcl_node_t node;
    
    // Subs
    rcl_subscription_t sub_torso, sub_neck, sub_head, sub_led, sub_reb;
    std_msgs__msg__String msg_torso, msg_neck, msg_head, msg_led, msg_reb;

    // Wait for IP
    xEventGroupWaitBits(eth_event_group, GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // Initialize MicroROS
    // IMPORTANT: Setup custom transport for Ethernet if not default
    // Assuming ESP-IDF component handles it via Kconfig or we use UDP
    
    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
    // If UDP: rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init MicroROS support. check Agent connection.");
        vTaskDelete(NULL);
    }

    rclc_node_init_default(&node, "esp32_machani_node", "", &support);

    // Create Publishers
    rclc_publisher_init_default(&pub_touch, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "touch");
    rclc_publisher_init_default(&pub_batt, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "batt");

    // Create Subscribers
    rclc_subscription_init_default(&sub_torso, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "torso_cmd");
    rclc_subscription_init_default(&sub_neck, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "neck_cmd");
    rclc_subscription_init_default(&sub_head, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "head_cmd");
    rclc_subscription_init_default(&sub_led, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "led");
    rclc_subscription_init_default(&sub_reb, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "reb_jet");

    // Msg Init
    std_msgs__msg__String__init(&msg_torso);
    std_msgs__msg__String__init(&msg_neck);
    std_msgs__msg__String__init(&msg_head);
    std_msgs__msg__String__init(&msg_led);
    std_msgs__msg__String__init(&msg_reb);
    std_msgs__msg__String__init(&msg_touch);
    std_msgs__msg__String__init(&msg_batt);

    // Executor
    rclc_executor_init(&executor, &support.context, 5, &allocator);
    rclc_executor_add_subscription(&executor, &sub_torso, &msg_torso, &torso_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_neck, &msg_neck, &neck_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_head, &msg_head, &head_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_led, &msg_led, &led_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_reb, &msg_reb, &reb_jet_callback, ON_NEW_DATA);

    ESP_LOGI(TAG, "MicroROS Node Started");

    while(1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void sensing_task(void * arg) {
    uint16_t touch_val;
    char buf[64];
    
    // Allocate memory for messages
    msg_touch.data.capacity = 64;
    msg_touch.data.data = (char*)malloc(msg_touch.data.capacity);
    msg_touch.data.size = 0;

    msg_batt.data.capacity = 64;
    msg_batt.data.data = (char*)malloc(msg_batt.data.capacity);
    msg_batt.data.size = 0;

    while(1) {
        // Touch
        touch_pad_read(TOUCH_PIN_NUM, &touch_val);
        if (touch_val < TOUCH_THRESH) {
            sprintf(msg_touch.data.data, "{\"state\":\"tap\"}"); // Simple tap detection
            msg_touch.data.size = strlen(msg_touch.data.data);
            rcl_publish(&pub_touch, &msg_touch, NULL);
        }

        // Battery (Dummy I2C Read)
        // uint8_t data[2];
        // i2c_master_read_from_device(I2C_MASTER_NUM, BATTERY_I2C_ADDR, data, 2, 100);
        int batt_pct = 80; // Mock
        sprintf(msg_batt.data.data, "{\"batt\":%d, \"ps\":true, \"hb_out\":1}", batt_pct);
        msg_batt.data.size = strlen(msg_batt.data.data);
        rcl_publish(&pub_batt, &msg_batt, NULL);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// --- Ethernet Setup (Same as Example) ---
static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;
    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    default: break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;
    ESP_LOGI(TAG, "Ethernet Got IP: " IPSTR, IP2STR(&ip_info->ip));
    xEventGroupSetBits(eth_event_group, GOT_IP_BIT);
}

void app_main(void)
{
    // Init Drivers
    eth_event_group = xEventGroupCreate();
    
    // UART for Servo
    uart_config_t uart_config = {
        .baud_rate = SERVO_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(SERVO_UART_NUM, &uart_config);
    uart_set_pin(SERVO_UART_NUM, SERVO_TX_PIN, SERVO_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(SERVO_UART_NUM, 1024, 0, 0, NULL, 0);

    // I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    // Touch
    touch_pad_init();
    touch_pad_config(TOUCH_PIN_NUM, 0);

    // Ethernet
    gpio_install_isr_service(0);
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO_GPIO,
        .mosi_io_num = SPI_MOSI_GPIO,
        .sclk_io_num = SPI_CLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 4 * 1000 * 1000,
        .spics_io_num = SPI_CS_GPIO,
        .queue_size = 20
    };
    eth_enc28j60_config_t enc28j60_config = ETH_ENC28J60_DEFAULT_CONFIG(SPI3_HOST, &devcfg);
    enc28j60_config.int_gpio_num = -1; // Poll or Interrupt? Example had -1 for INT_GPIO in defines but passed it?
    // Checking example main.c: #define INT_GPIO -1. So interrupt disabled, polling mode?
    // "enc28j60_config.int_gpio_num = INT_GPIO;"
    
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_enc28j60(&enc28j60_config, &mac_config);
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = -1;
    phy_config.reset_gpio_num = -1;
    esp_eth_phy_t *phy = my_esp_eth_phy_new_enc28j60(&phy_config);
    
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    esp_eth_driver_install(&config, &eth_handle);
    esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle));
    
    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL);
    
    esp_eth_start(eth_handle);

    // Create Tasks
    xTaskCreate(micro_ros_task, "uros_task", 10000, NULL, 5, NULL);
    xTaskCreate(sensing_task, "sense_task", 4096, NULL, 5, NULL);
}
