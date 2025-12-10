idf_component_register(SRCS "main.c" "esp_eth_mac_enc28j60.c" "esp_eth_phy_enc28j60.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_eth lwip)
