#include "iot_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_eth.h"
#include "esp_eth_enc28j60.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "driver/spi_master.h"
#include "lwip/sockets.h"
#include "lwip/ip_addr.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "iot_protocol";

#define MAX_SUBSCRIPTIONS 10
#define MAX_PAYLOAD 512
#define UDP_PORT 5555
#define HANDSHAKE_TIMEOUT_MS 5000

// Event group bits
#define ETH_CONNECTED_BIT BIT0

typedef struct {
    char topic[64];
    iot_msg_callback_t callback;
} subscription_t;

typedef struct {
    esp_eth_handle_t eth_handle;
    esp_netif_t *eth_netif;
    EventGroupHandle_t eth_event_group;
    subscription_t subscriptions[MAX_SUBSCRIPTIONS];
    uint8_t num_subscriptions;
    int socket;
    uint8_t mac_addr[6];
    char ip_addr[16];
    bool is_connected;
} iot_protocol_ctx_t;

static iot_protocol_ctx_t ctx = {0};

// Event handler for Ethernet
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_base == ETH_EVENT) {
        switch (event_id) {
            case ETHERNET_EVENT_CONNECTED:
                ESP_LOGI(TAG, "Ethernet Link Up");
                ctx.is_connected = true;
                xEventGroupSetBits(ctx.eth_event_group, ETH_CONNECTED_BIT);
                break;
            case ETHERNET_EVENT_DISCONNECTED:
                ESP_LOGI(TAG, "Ethernet Link Down");
                ctx.is_connected = false;
                xEventGroupClearBits(ctx.eth_event_group, ETH_CONNECTED_BIT);
                break;
            case ETHERNET_EVENT_START:
                ESP_LOGI(TAG, "Ethernet Started");
                break;
            case ETHERNET_EVENT_STOP:
                ESP_LOGI(TAG, "Ethernet Stopped");
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_ETH_GOT_IP: {
                ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
                ESP_LOGI(TAG, "Ethernet Got IP Address");
                ESP_LOGI(TAG, "IPADDR: " IPSTR, IP2STR(&event->ip_info.ip));
                ESP_LOGI(TAG, "NETMASK: " IPSTR, IP2STR(&event->ip_info.netmask));
                ESP_LOGI(TAG, "GWTEWAY: " IPSTR, IP2STR(&event->ip_info.gw));
                
                snprintf(ctx.ip_addr, sizeof(ctx.ip_addr), IPSTR, IP2STR(&event->ip_info.ip));
                ctx.is_connected = true;
                xEventGroupSetBits(ctx.eth_event_group, ETH_CONNECTED_BIT);
                break;
            }
            case IP_EVENT_ETH_LOST_IP:
                ESP_LOGI(TAG, "Ethernet Lost IP");
                ctx.is_connected = false;
                xEventGroupClearBits(ctx.eth_event_group, ETH_CONNECTED_BIT);
                break;
            default:
                break;
        }
    }
}

esp_err_t iot_protocol_init(int uart_port)
{
    ESP_LOGI(TAG, "Initializing IoT Protocol...");

    // Initialize event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create event group
    ctx.eth_event_group = xEventGroupCreate();
    if (!ctx.eth_event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_FAIL;
    }

    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default Ethernet interface
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    ctx.eth_netif = esp_netif_new(&cfg);

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_LOST_IP, &eth_event_handler, NULL));

    // Initialize ENC28J60 SPI Ethernet using wiring provided by user
    // SPI bus configuration (VSPI: SCLK=18, MOSI=23, MISO=19)
    spi_bus_config_t spi_bus_cfg = {
        .miso_io_num = 19,      // GPIO19 - MISO
        .mosi_io_num = 23,      // GPIO23 - MOSI
        .sclk_io_num = 18,      // GPIO18 - SCLK
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4094,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));

    // SPI device configuration for ENC28J60 (CS=GPIO5, clock 4MHz as documented)
    spi_device_interface_config_t devcfg = {
        .command_bits = 8,
        .address_bits = 8,
        .mode = 0,
        .clock_speed_hz = 4 * 1000 * 1000,  // 4 MHz (stable for ENC28J60)
        .spics_io_num = 5,                  // GPIO5 - CS
        .queue_size = 20,
    };

    // ENC28J60 MAC configuration (INT not used / polled)
    eth_enc28j60_config_t enc28j60_config = ETH_ENC28J60_DEFAULT_CONFIG(VSPI_HOST, &devcfg);
    enc28j60_config.int_gpio_num = -1;  // No interrupt (N/C)
    
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    esp_eth_mac_t *mac = esp_eth_mac_new_enc28j60(&enc28j60_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_enc28j60(&phy_config);

    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &ctx.eth_handle));

    // Attach Ethernet to TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_attach(ctx.eth_netif, esp_eth_new_netif_glue(ctx.eth_handle)));

    // Use static IP configuration as provided
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 169, 254, 1, 200);
    IP4_ADDR(&ip_info.gw, 169, 254, 1, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 0, 0);
    // Stop DHCP client (we are using static IP)
    esp_err_t err = esp_netif_dhcpc_stop(ctx.eth_netif);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_netif_dhcpc_stop returned %d", err);
    }
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ctx.eth_netif, &ip_info));
    snprintf(ctx.ip_addr, sizeof(ctx.ip_addr), "%u.%u.%u.%u", 169, 254, 1, 200);

    // Start Ethernet
    ESP_ERROR_CHECK(esp_eth_start(ctx.eth_handle));

    // Get MAC address
    ESP_ERROR_CHECK(esp_eth_ioctl(ctx.eth_handle, ETH_CMD_G_MAC_ADDR, ctx.mac_addr));
    ESP_LOGI(TAG, "MAC Address: %02x:%02x:%02x:%02x:%02x:%02x",
             ctx.mac_addr[0], ctx.mac_addr[1], ctx.mac_addr[2],
             ctx.mac_addr[3], ctx.mac_addr[4], ctx.mac_addr[5]);

    // Wait for IP assignment (up to 10 seconds)
    EventBits_t bits = xEventGroupWaitBits(ctx.eth_event_group,
                                           ETH_CONNECTED_BIT,
                                           false,
                                           true,
                                           pdMS_TO_TICKS(10000));

    if (bits & ETH_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Ethernet initialized successfully. IP: %s", ctx.ip_addr);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Ethernet not connected yet, continuing anyway...");
        return ESP_OK;  // Still return OK, might connect later
    }
}

esp_err_t iot_protocol_handshake(void)
{
    ESP_LOGI(TAG, "Performing handshake...");

    // Wait for IP if not already got
    EventBits_t bits = xEventGroupWaitBits(ctx.eth_event_group,
                                           ETH_CONNECTED_BIT,
                                           false,
                                           true,
                                           pdMS_TO_TICKS(HANDSHAKE_TIMEOUT_MS));

    if (!(bits & ETH_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "Handshake: Waiting for IP assignment...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Create UDP socket for pub/sub communication
    ctx.socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (ctx.socket < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        return ESP_FAIL;
    }

    // Bind socket to local port
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(UDP_PORT);

    if (bind(ctx.socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed");
        close(ctx.socket);
        return ESP_FAIL;
    }

    // Send handshake message to broadcast
    cJSON *hs_json = cJSON_CreateObject();
    cJSON_AddStringToObject(hs_json, "type", "handshake");
    cJSON_AddStringToObject(hs_json, "device", "esp32_robot");
    cJSON_AddStringToObject(hs_json, "ip", ctx.ip_addr);
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
             ctx.mac_addr[0], ctx.mac_addr[1], ctx.mac_addr[2],
             ctx.mac_addr[3], ctx.mac_addr[4], ctx.mac_addr[5]);
    cJSON_AddStringToObject(hs_json, "mac", mac_str);

    char *hs_payload = cJSON_Print(hs_json);
    if (!hs_payload) {
        cJSON_Delete(hs_json);
        return ESP_FAIL;
    }

    struct sockaddr_in broadcast_addr = {0};
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    broadcast_addr.sin_port = htons(5556);  // Windows listening port

    sendto(ctx.socket, hs_payload, strlen(hs_payload), 0,
           (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));

    free(hs_payload);
    cJSON_Delete(hs_json);

    ESP_LOGI(TAG, "Handshake sent");
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

int iot_protocol_process_incoming(void)
{
    if (ctx.socket < 0) {
        return 0;
    }

    struct sockaddr_in src_addr = {0};
    socklen_t src_addr_len = sizeof(src_addr);
    uint8_t rx_buffer[MAX_PAYLOAD];

    int len = recvfrom(ctx.socket, rx_buffer, sizeof(rx_buffer) - 1, MSG_DONTWAIT,
                       (struct sockaddr *)&src_addr, &src_addr_len);

    if (len < 0) {
        // No data available or error
        return 0;
    }

    rx_buffer[len] = '\0';

    // Parse incoming message
    cJSON *msg = cJSON_Parse((const char *)rx_buffer);
    if (!msg) {
        ESP_LOGW(TAG, "Failed to parse incoming message");
        return 0;
    }

    cJSON *topic_item = cJSON_GetObjectItem(msg, "topic");
    cJSON *payload_item = cJSON_GetObjectItem(msg, "payload");

    if (!cJSON_IsString(topic_item) || !cJSON_IsObject(payload_item)) {
        cJSON_Delete(msg);
        return 0;
    }

    const char *topic = topic_item->valuestring;
    char *payload = cJSON_Print(payload_item);

    // Find and call matching subscription callbacks
    for (uint8_t i = 0; i < ctx.num_subscriptions; i++) {
        if (strcmp(ctx.subscriptions[i].topic, topic) == 0) {
            if (ctx.subscriptions[i].callback) {
                ctx.subscriptions[i].callback(topic, (const uint8_t *)payload, strlen(payload));
            }
        }
    }

    free(payload);
    cJSON_Delete(msg);

    return 1;
}

esp_err_t iot_protocol_publish(const char *topic, const char *payload)
{
    if (!ctx.is_connected || ctx.socket < 0) {
        ESP_LOGW(TAG, "Not connected, cannot publish");
        return ESP_FAIL;
    }

    // Create message object
    cJSON *msg = cJSON_CreateObject();
    cJSON_AddStringToObject(msg, "topic", topic);
    cJSON *payload_obj = cJSON_Parse(payload);
    if (payload_obj) {
        cJSON_AddItemToObject(msg, "payload", payload_obj);
    } else {
        cJSON_AddStringToObject(msg, "payload", payload);
    }

    char *msg_str = cJSON_Print(msg);
    if (!msg_str) {
        cJSON_Delete(msg);
        return ESP_FAIL;
    }

    // Send to Windows host (broadcast for now)
    struct sockaddr_in dest_addr = {0};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    dest_addr.sin_port = htons(5556);

    int result = sendto(ctx.socket, msg_str, strlen(msg_str), 0,
                        (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    free(msg_str);
    cJSON_Delete(msg);

    if (result < 0) {
        ESP_LOGE(TAG, "Failed to send message");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Published to %s", topic);
    return ESP_OK;
}

esp_err_t iot_protocol_subscribe(const char *topic, iot_msg_callback_t callback)
{
    if (ctx.num_subscriptions >= MAX_SUBSCRIPTIONS) {
        ESP_LOGE(TAG, "Maximum subscriptions reached");
        return ESP_FAIL;
    }

    for (uint8_t i = 0; i < ctx.num_subscriptions; i++) {
        if (strcmp(ctx.subscriptions[i].topic, topic) == 0) {
            // Update existing subscription
            ctx.subscriptions[i].callback = callback;
            return ESP_OK;
        }
    }

    // Add new subscription
    strncpy(ctx.subscriptions[ctx.num_subscriptions].topic, topic, sizeof(ctx.subscriptions[0].topic) - 1);
    ctx.subscriptions[ctx.num_subscriptions].callback = callback;
    ctx.num_subscriptions++;

    ESP_LOGI(TAG, "Subscribed to: %s", topic);
    return ESP_OK;
}

bool iot_protocol_is_connected(void)
{
    return ctx.is_connected;
}

bool iot_protocol_get_ip_address(char *ip_str)
{
    if (!ip_str || !ctx.is_connected) {
        return false;
    }
    strcpy(ip_str, ctx.ip_addr);
    return true;
}

bool iot_protocol_get_mac_address(char *mac_str)
{
    if (!mac_str) {
        return false;
    }
    snprintf(mac_str, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
             ctx.mac_addr[0], ctx.mac_addr[1], ctx.mac_addr[2],
             ctx.mac_addr[3], ctx.mac_addr[4], ctx.mac_addr[5]);
    return true;
}

esp_err_t iot_protocol_deinit(void)
{
    if (ctx.socket >= 0) {
        close(ctx.socket);
        ctx.socket = -1;
    }

    if (ctx.eth_handle) {
        esp_eth_stop(ctx.eth_handle);
        esp_eth_driver_uninstall(ctx.eth_handle);
        ctx.eth_handle = NULL;
    }

    if (ctx.eth_event_group) {
        vEventGroupDelete(ctx.eth_event_group);
        ctx.eth_event_group = NULL;
    }

    memset(&ctx, 0, sizeof(ctx));
    ctx.socket = -1;

    return ESP_OK;
}
