#pragma once
#include "esp_eth.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_eth_phy_t *my_esp_eth_phy_new_enc28j60(const eth_phy_config_t *config);

#ifdef __cplusplus
}
#endif
