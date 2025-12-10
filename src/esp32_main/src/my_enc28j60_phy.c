/*
 * Custom ENC28J60 PHY driver to bypass strict ID check
 */
#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "esp_eth.h"
// #include "eth_phy_802_3_regs.h" // Define manually to avoid dependency issues
#include "esp_eth_enc28j60.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char *TAG = "my_enc28j60_phy";

#define ETH_PHY_BMCR_REG_ADDR 0x00
#define ETH_PHY_IDR1_REG_ADDR 0x02
#define ETH_PHY_IDR2_REG_ADDR 0x03

#define PHY_CHECK(a, str, goto_tag, ...)                                          \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

/***************Vendor Specific Register***************/

typedef union {
    struct {
        uint32_t reserved_7_0 : 8;
        uint32_t pdpxmd : 1;
        uint32_t reserved_10_9: 2;
        uint32_t ppwrsv: 1;
        uint32_t reserved_13_12: 2;
        uint32_t ploopbk: 1;
        uint32_t prst: 1;
    };
    uint32_t val;
} phcon1_reg_t;
#define ETH_PHY_PHCON1_REG_ADDR (0x00)

typedef union {
    struct {
        uint32_t reserved_7_0 : 8;
        uint32_t hdldis : 1;
        uint32_t reserved_9: 1;
        uint32_t jabber: 1;
        uint32_t reserved_12_11: 2;
        uint32_t txdis: 1;
        uint32_t frclnk: 1;
        uint32_t reserved_15: 1;
    };
    uint32_t val;
} phcon2_reg_t;
#define ETH_PHY_PHCON2_REG_ADDR (0x10)

typedef union {
    struct {
        uint32_t reserved_4_0 : 5;
        uint32_t plrity : 1;
        uint32_t reserved_8_6 : 3;
        uint32_t dpxstat : 1;
        uint32_t lstat : 1;
        uint32_t colstat : 1;
        uint32_t rxstat : 1;
        uint32_t txstat : 1;
        uint32_t reserved_15_14 : 2;
    };
    uint32_t val;
} phstat2_reg_t;
#define ETH_PHY_PHSTAT2_REG_ADDR (0x11)

typedef union {
    struct {
        uint32_t reserved_0 : 1;
        uint32_t duplex_mode : 1;
        uint32_t restart_auto_negotiation : 1;
        uint32_t isolate : 1;
        uint32_t power_down : 1;
        uint32_t auto_negotiation_enable : 1;
        uint32_t speed_selection_msb : 1;
        uint32_t collision_test : 1;
        uint32_t speed_selection_lsb : 1;
        uint32_t uni_directional_enable : 1;
        uint32_t low_power : 1;
        uint32_t reserved_11 : 1;
        uint32_t loopback : 1;
        uint32_t speed_selection : 1;
        uint32_t link_status : 1;
        uint32_t reset : 1;
    };
    uint32_t val;
} bmcr_reg_t;

typedef union {
    struct {
        uint32_t oui_lsb : 6;
        uint32_t vendor_model : 6;
        uint32_t model_revision : 4;
    };
    uint32_t val;
} phyidr2_reg_t;

typedef union {
    struct {
        uint32_t oui_msb : 16;
    };
    uint32_t val;
} phyidr1_reg_t;


typedef struct {
    esp_eth_phy_t parent;
    esp_eth_mediator_t *eth;
    uint32_t addr;
    uint32_t reset_timeout_ms;
    eth_link_t link_status;
    int reset_gpio_num;
} phy_enc28j60_t;

static esp_err_t enc28j60_update_link_duplex_speed(phy_enc28j60_t *enc28j60)
{
    esp_eth_mediator_t *eth = enc28j60->eth;
    eth_speed_t speed = ETH_SPEED_10M;
    eth_duplex_t duplex = ETH_DUPLEX_HALF;
    phstat2_reg_t phstat;
    PHY_CHECK(eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_PHSTAT2_REG_ADDR, &(phstat.val)) == ESP_OK,
              "read PHSTAT2 failed", err);
    eth_link_t link = phstat.lstat ? ETH_LINK_UP : ETH_LINK_DOWN;
    if (enc28j60->link_status != link) {
        if (link == ETH_LINK_UP) {
            if (phstat.dpxstat) {
                duplex = ETH_DUPLEX_FULL;
            } else {
                duplex = ETH_DUPLEX_HALF;
            }
            PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_SPEED, (void *)speed) == ESP_OK,
                      "change speed failed", err);
            PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_DUPLEX, (void *)duplex) == ESP_OK,
                      "change duplex failed", err);
        }
        PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_LINK, (void *)link) == ESP_OK,
                  "change link failed", err);
        enc28j60->link_status = link;
    }
    return ESP_OK;
err:
    return ESP_FAIL;
}

static esp_err_t enc28j60_set_mediator(esp_eth_phy_t *phy, esp_eth_mediator_t *eth)
{
    PHY_CHECK(eth, "can't set mediator for enc28j60 to null", err);
    phy_enc28j60_t *enc28j60 = __containerof(phy, phy_enc28j60_t, parent);
    enc28j60->eth = eth;
    return ESP_OK;
err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t enc28j60_get_link(esp_eth_phy_t *phy)
{
    phy_enc28j60_t *enc28j60 = __containerof(phy, phy_enc28j60_t, parent);
    PHY_CHECK(enc28j60_update_link_duplex_speed(enc28j60) == ESP_OK, "update link duplex speed failed", err);
    return ESP_OK;
err:
    return ESP_FAIL;
}

static esp_err_t enc28j60_reset(esp_eth_phy_t *phy)
{
    phy_enc28j60_t *enc28j60 = __containerof(phy, phy_enc28j60_t, parent);
    enc28j60->link_status = ETH_LINK_DOWN;
    esp_eth_mediator_t *eth = enc28j60->eth;
    bmcr_reg_t bmcr = {.reset = 1};
    PHY_CHECK(eth->phy_reg_write(eth, enc28j60->addr, ETH_PHY_BMCR_REG_ADDR, bmcr.val) == ESP_OK,
              "write BMCR failed", err);
    uint32_t to = 0;
    for (to = 0; to < enc28j60->reset_timeout_ms / 10; to++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        PHY_CHECK(eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)) == ESP_OK,
                  "read BMCR failed", err);
        if (!bmcr.reset) {
            break;
        }
    }
    PHY_CHECK(to < enc28j60->reset_timeout_ms / 10, "PHY reset timeout", err);
    return ESP_OK;
err:
    return ESP_FAIL;
}

static esp_err_t enc28j60_reset_hw(esp_eth_phy_t *phy)
{
    phy_enc28j60_t *enc28j60 = __containerof(phy, phy_enc28j60_t, parent);
    if (enc28j60->reset_gpio_num >= 0) {
        gpio_reset_pin(enc28j60->reset_gpio_num);
        gpio_set_direction(enc28j60->reset_gpio_num, GPIO_MODE_OUTPUT);
        gpio_set_level(enc28j60->reset_gpio_num, 0);
        gpio_set_level(enc28j60->reset_gpio_num, 1);
    }
    return ESP_OK;
}

static esp_err_t enc28j60_autonego_ctrl(esp_eth_phy_t *phy, eth_phy_autoneg_cmd_t cmd, bool *autoneg_en_stat)
{
    switch (cmd) {
    case ESP_ETH_PHY_AUTONEGO_RESTART:
    case ESP_ETH_PHY_AUTONEGO_EN:
        return ESP_ERR_NOT_SUPPORTED;
    case ESP_ETH_PHY_AUTONEGO_DIS:
    case ESP_ETH_PHY_AUTONEGO_G_STAT:
        *autoneg_en_stat = false;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t enc28j60_set_speed(esp_eth_phy_t *phy, eth_speed_t speed)
{
    if (speed == ETH_SPEED_10M) {
        return ESP_OK;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t enc28j60_set_duplex(esp_eth_phy_t *phy, eth_duplex_t duplex)
{
    phy_enc28j60_t *enc28j60 = __containerof(phy, phy_enc28j60_t, parent);
    esp_eth_mediator_t *eth = enc28j60->eth;
    phcon1_reg_t phcon1;

    enc28j60->link_status = ETH_LINK_DOWN;

    PHY_CHECK(eth->phy_reg_read(eth, enc28j60->addr, 0, &phcon1.val) == ESP_OK,
              "read PHCON1 failed", err);
    switch (duplex) {
    case ETH_DUPLEX_HALF:
        phcon1.pdpxmd = 0;
        break;
    case ETH_DUPLEX_FULL:
        phcon1.pdpxmd = 1;
        break;
    default:
        PHY_CHECK(false, "unknown duplex", err);
        break;
    }

    PHY_CHECK(eth->phy_reg_write(eth, enc28j60->addr, 0, phcon1.val) == ESP_OK,
              "write PHCON1 failed", err);

    return ESP_OK;
err:
    return ESP_FAIL;
}

static esp_err_t enc28j60_pwrctl(esp_eth_phy_t *phy, bool enable)
{
    phy_enc28j60_t *enc28j60 = __containerof(phy, phy_enc28j60_t, parent);
    esp_eth_mediator_t *eth = enc28j60->eth;
    bmcr_reg_t bmcr;
    PHY_CHECK(eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)) == ESP_OK,
              "read BMCR failed", err);
    if (!enable) {
        bmcr.power_down = 1;
    } else {
        bmcr.power_down = 0;
    }
    PHY_CHECK(eth->phy_reg_write(eth, enc28j60->addr, ETH_PHY_BMCR_REG_ADDR, bmcr.val) == ESP_OK,
              "write BMCR failed", err);
    PHY_CHECK(eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)) == ESP_OK,
              "read BMCR failed", err);
    if (!enable) {
        PHY_CHECK(bmcr.power_down == 1, "power down failed", err);
    } else {
        PHY_CHECK(bmcr.power_down == 0, "power up failed", err);
    }
    return ESP_OK;
err:
    return ESP_FAIL;
}

static esp_err_t enc28j60_set_addr(esp_eth_phy_t *phy, uint32_t addr)
{
    phy_enc28j60_t *enc28j60 = __containerof(phy, phy_enc28j60_t, parent);
    enc28j60->addr = addr;
    return ESP_OK;
}

static esp_err_t enc28j60_get_addr(esp_eth_phy_t *phy, uint32_t *addr)
{
    PHY_CHECK(addr, "addr can't be null", err);
    phy_enc28j60_t *enc28j60 = __containerof(phy, phy_enc28j60_t, parent);
    *addr = enc28j60->addr;
    return ESP_OK;
err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t enc28j60_del(esp_eth_phy_t *phy)
{
    phy_enc28j60_t *enc28j60 = __containerof(phy, phy_enc28j60_t, parent);
    free(enc28j60);
    return ESP_OK;
}

static esp_err_t enc28j60_init(esp_eth_phy_t *phy)
{
    phy_enc28j60_t *enc28j60 = __containerof(phy, phy_enc28j60_t, parent);
    esp_eth_mediator_t *eth = enc28j60->eth;
    PHY_CHECK(enc28j60_pwrctl(phy, true) == ESP_OK, "power control failed", err);
    PHY_CHECK(enc28j60_reset(phy) == ESP_OK, "reset failed", err);
    
    // Custom Check: Just log, don't fail
    phyidr1_reg_t id1;
    phyidr2_reg_t id2;
    eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_IDR1_REG_ADDR, &(id1.val));
    eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_IDR2_REG_ADDR, &(id2.val));
    ESP_LOGI(TAG, "PHY ID Read: ID1=0x%04lx, ID2=0x%04lx", id1.val, id2.val);
    
    phcon2_reg_t phcon2;
    PHY_CHECK(eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_PHCON2_REG_ADDR, &(phcon2.val)) == ESP_OK,
              "read PHCON2 failed", err);
    phcon2.hdldis = 1;
    PHY_CHECK(eth->phy_reg_write(eth, enc28j60->addr, ETH_PHY_PHCON2_REG_ADDR, phcon2.val) == ESP_OK,
              "write PHCON2 failed", err);
    return ESP_OK;
err:
    return ESP_FAIL;
}

static esp_err_t enc28j60_deinit(esp_eth_phy_t *phy)
{
    PHY_CHECK(enc28j60_pwrctl(phy, false) == ESP_OK, "power off Ethernet PHY failed", err);
    return ESP_OK;
err:
    return ESP_FAIL;
}

esp_eth_phy_t *my_esp_eth_phy_new_enc28j60(const eth_phy_config_t *config)
{
    PHY_CHECK(config, "can't set phy config to null", err);
    phy_enc28j60_t *enc28j60 = calloc(1, sizeof(phy_enc28j60_t));
    PHY_CHECK(enc28j60, "calloc enc28j60 failed", err);
    enc28j60->addr = config->phy_addr;
    enc28j60->reset_timeout_ms = config->reset_timeout_ms;
    enc28j60->reset_gpio_num = config->reset_gpio_num;
    enc28j60->link_status = ETH_LINK_DOWN;
    enc28j60->parent.reset = enc28j60_reset;
    enc28j60->parent.reset_hw = enc28j60_reset_hw;
    enc28j60->parent.init = enc28j60_init;
    enc28j60->parent.deinit = enc28j60_deinit;
    enc28j60->parent.set_mediator = enc28j60_set_mediator;
    enc28j60->parent.autonego_ctrl = enc28j60_autonego_ctrl;
    enc28j60->parent.get_link = enc28j60_get_link;
    enc28j60->parent.pwrctl = enc28j60_pwrctl;
    enc28j60->parent.get_addr = enc28j60_get_addr;
    enc28j60->parent.set_addr = enc28j60_set_addr;
    enc28j60->parent.set_speed = enc28j60_set_speed;
    enc28j60->parent.set_duplex = enc28j60_set_duplex;
    enc28j60->parent.del = enc28j60_del;
    return &(enc28j60->parent);
err:
    return NULL;
}
