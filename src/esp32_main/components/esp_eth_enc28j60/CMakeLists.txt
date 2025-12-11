idf_component_register(SRCS "src/esp_eth_mac_enc28j60.c" "src/esp_eth_phy_enc28j60.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_eth driver spi_flash)
