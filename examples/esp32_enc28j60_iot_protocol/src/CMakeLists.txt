idf_component_register(SRCS "main.c" "my_enc28j60_phy.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_eth lwip esp_http_client esp-tls esp_eth_enc28j60)
