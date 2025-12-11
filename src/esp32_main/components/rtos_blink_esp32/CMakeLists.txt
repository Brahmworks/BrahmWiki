idf_component_register(SRCS "rtos_blink_esp32.c" "iot_protocol.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver uart_standard esp_eth esp_netif lwip json esp_eth_enc28j60)
