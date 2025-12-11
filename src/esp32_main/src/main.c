/* ENC28J60 standalone Ethernet test application
   This replaces app_main to isolate and verify ENC28J60 wiring and init.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include <esp_eth_enc28j60.h>
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include "esp_mac.h"

static const char *TAG = "eth_test";
#define PORT 3333
#include "freertos/event_groups.h"

static EventGroupHandle_t eth_event_group;
const int GOT_IP_BIT = BIT0;

// Pins configuration matching your wiring
#if CONFIG_IDF_TARGET_ESP32
    #define SPI_MISO_GPIO 19
    #define SPI_MOSI_GPIO 23
    #define SPI_CLK_GPIO  18
    #define SPI_CS_GPIO   5
#elif CONFIG_IDF_TARGET_ESP32C3
    #define SPI_MISO_GPIO 2
    #define SPI_MOSI_GPIO 7
    #define SPI_CLK_GPIO  6
    #define SPI_CS_GPIO   10
#else
    #define SPI_MISO_GPIO 13
    #define SPI_MOSI_GPIO 11
    #define SPI_CLK_GPIO  12
    #define SPI_CS_GPIO   10
#endif

#define INT_GPIO -1
#define SPI_HOST_ID SPI3_HOST
#define SPI_CLOCK_MHZ 4 // try 4MHz, lower to 1MHz if unstable

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
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
}

static void udp_server_task(void *pvParameters)
{
    ESP_LOGI(TAG, "UDP Server Task Started");
    char rx_buffer[1514];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {
        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr;
        socklen_t socklen = sizeof(source_addr);

        while (1) {
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            } else {
                rx_buffer[len] = 0;
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }
                if (len < 100) {
                    ESP_LOGI(TAG, "Received %d bytes from %s: %s", len, addr_str, rx_buffer);
                } else {
                    ESP_LOGI(TAG, "Received %d bytes from %s (payload hidden)", len, addr_str);
                }
                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));

    xEventGroupSetBits(eth_event_group, GOT_IP_BIT);

    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
}

void app_main(void)
{
    eth_event_group = xEventGroupCreate();

    gpio_install_isr_service(0);

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    esp_netif_set_hostname(eth_netif, "esp32-ethernet");

    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO_GPIO,
        .mosi_io_num = SPI_MOSI_GPIO,
        .sclk_io_num = SPI_CLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_ID, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = SPI_CS_GPIO,
        .queue_size = 20
    };

    eth_enc28j60_config_t enc28j60_config = ETH_ENC28J60_DEFAULT_CONFIG(SPI_HOST_ID, &devcfg);
    enc28j60_config.int_gpio_num = INT_GPIO;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_enc28j60(&enc28j60_config, &mac_config);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = -1;
    phy_config.reset_gpio_num = -1;
    esp_eth_phy_t *phy = esp_eth_phy_new_enc28j60(&phy_config);

    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));

    uint8_t base_mac_addr[ETH_ADDR_LEN];
    ESP_ERROR_CHECK(esp_read_mac(base_mac_addr, ESP_MAC_ETH));
    ESP_LOGI(TAG, "Setting MAC address to %02x:%02x:%02x:%02x:%02x:%02x",
             base_mac_addr[0], base_mac_addr[1], base_mac_addr[2],
             base_mac_addr[3], base_mac_addr[4], base_mac_addr[5]);
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, base_mac_addr));

    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    esp_netif_dhcpc_stop(eth_netif);
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 169, 254, 1, 200);
    IP4_ADDR(&ip_info.gw, 169, 254, 1, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 0, 0);
    esp_netif_set_ip_info(eth_netif, &ip_info);

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    ESP_LOGI(TAG, "Waiting for IP address...");
    EventBits_t bits = xEventGroupWaitBits(eth_event_group, GOT_IP_BIT, pdTRUE, pdTRUE, pdMS_TO_TICKS(10000));
    if (bits & GOT_IP_BIT) {
        ESP_LOGI(TAG, "IP Address obtained! Ready for communication.");
    } else {
        ESP_LOGE(TAG, "DHCP Timeout. Please check your network connection.");
    }
}
