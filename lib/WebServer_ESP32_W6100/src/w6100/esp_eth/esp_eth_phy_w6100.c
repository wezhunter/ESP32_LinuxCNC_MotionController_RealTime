/****************************************************************************************************************************
  esp_eth_phy_w6100.c

  For Ethernet shields using ESP32_W6100 (ESP32 + W6100)

  WebServer_ESP32_W6100 is a library for the ESP32 with Ethernet W6100 to run WebServer

  Based on and modified from ESP32-IDF https://github.com/espressif/esp-idf
  Built by Khoi Hoang https://github.com/khoih-prog/WebServer_ESP32_W6100
  Licensed under GPLv3 license

  Version: 1.5.3

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.5.2   K Hoang      06/01/2022 Initial coding for ESP32_W6100 (ESP32 + W6100). Sync with WebServer_ESP32_W5500 v1.5.2
  1.5.3   K Hoang      11/01/2023 Using `SPI_DMA_CH_AUTO`
 *****************************************************************************************************************************/
// Copyright 2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

////////////////////////////////////////

#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_eth.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "w6100.h"

////////////////////////////////////////

static const char *TAG = "w6100.phy";

////////////////////////////////////////

/***************Vendor Specific Register***************/
/**
   @brief PHYCFGR(PHY Configuration Register)

*/
typedef union
{
  struct
  {
    uint8_t link: 1;   /*!< Link status */
    uint8_t speed: 1;  /*!< Speed status */
    uint8_t duplex: 1; /*!< Duplex status */
    uint8_t opmode: 3; /*!< Operation mode */
    uint8_t opsel: 1;  /*!< Operation select */
    uint8_t reset: 1;  /*!< Reset, when this bit is '0', PHY will get reset */
  };

  uint8_t val;
} phycfg_reg_t;

////////////////////////////////////////

typedef struct
{
  esp_eth_phy_t parent;
  esp_eth_mediator_t *eth;
  int addr;
  uint32_t reset_timeout_ms;
  uint32_t autonego_timeout_ms;
  eth_link_t link_status;
  int reset_gpio_num;
} phy_w6100_t;

////////////////////////////////////////

static esp_err_t w6100_update_link_duplex_speed(phy_w6100_t *w6100)
{
  esp_err_t ret = ESP_OK;

  esp_eth_mediator_t *eth = w6100->eth;
  eth_speed_t speed = ETH_SPEED_10M;
  eth_duplex_t duplex = ETH_DUPLEX_HALF;
  phycfg_reg_t phycfg;

  ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, w6100->addr, W6100_REG_PHYCFGR, (uint32_t *) & (phycfg.val)), err, TAG,
                    "Read PHYCFG failed");
  eth_link_t link = phycfg.link ? ETH_LINK_UP : ETH_LINK_DOWN;

  /* check if link status changed */
  if (w6100->link_status != link)
  {
    /* when link up, read negotiation result */
    if (link == ETH_LINK_UP)
    {
      if (phycfg.speed)
      {
        //Inverted compared to W5500
        speed = ETH_SPEED_10M;
      }
      else
      {
        //Inverted compared to W5500
        speed = ETH_SPEED_100M;
      }

      if (phycfg.duplex)
      {
        duplex = ETH_DUPLEX_FULL;
      }
      else
      {
        duplex = ETH_DUPLEX_HALF;
      }

      ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_SPEED, (void *)speed), err, TAG, "Change speed failed");
      ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_DUPLEX, (void *)duplex), err, TAG, "Change duplex failed");
    }

    ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_LINK, (void *)link), err, TAG, "Change link failed");
    w6100->link_status = link;
  }

  return ESP_OK;

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_set_mediator(esp_eth_phy_t *phy, esp_eth_mediator_t *eth)
{
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_FALSE(eth, ESP_ERR_INVALID_ARG, err, TAG, "Can't set mediator to null");
  phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
  w6100->eth = eth;

  return ESP_OK;

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_get_link(esp_eth_phy_t *phy)
{
  esp_err_t ret = ESP_OK;

  phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);

  /* Updata information about link, speed, duplex */
  ESP_GOTO_ON_ERROR(w6100_update_link_duplex_speed(w6100), err, TAG, "Update link duplex speed failed");

  return ESP_OK;

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_reset(esp_eth_phy_t *phy)
{
  esp_err_t ret = ESP_OK;

  phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);

  w6100->link_status = ETH_LINK_DOWN;
  esp_eth_mediator_t *eth = w6100->eth;

  phycfg_reg_t phycfg;
  ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, w6100->addr, W6100_REG_PHYCFGR, (uint32_t *) & (phycfg.val)), err, TAG,
                    "Read PHYCFG failed");

  phycfg.reset = 0; // set to '0' will reset internal PHY
  ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCFGR, phycfg.val), err, TAG, "Write PHYCFG failed");

  vTaskDelay(pdMS_TO_TICKS(10));

  phycfg.reset = 1; // set to '1' after reset
  ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCFGR, phycfg.val), err, TAG, "Write PHYCFG failed");

  return ESP_OK;

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_reset_hw(esp_eth_phy_t *phy)
{
  phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);

  // set reset_gpio_num to a negative value can skip hardware reset phy chip
  if (w6100->reset_gpio_num >= 0)
  {
    esp_rom_gpio_pad_select_gpio(w6100->reset_gpio_num);
    gpio_set_direction(w6100->reset_gpio_num, GPIO_MODE_OUTPUT);
    gpio_set_level(w6100->reset_gpio_num, 0);
    esp_rom_delay_us(100); // insert min input assert time
    gpio_set_level(w6100->reset_gpio_num, 1);
  }

  return ESP_OK;
}

////////////////////////////////////////

static esp_err_t w6100_negotiate(esp_eth_phy_t *phy)
{
  esp_err_t ret = ESP_OK;

  phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);

  esp_eth_mediator_t *eth = w6100->eth;

  /* in case any link status has changed, let's assume we're in link down status */
  w6100->link_status = ETH_LINK_DOWN;
  phycfg_reg_t phycfg;
  ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, w6100->addr, W6100_REG_PHYCFGR, (uint32_t *) & (phycfg.val)), err, TAG,
                    "Read PHYCFG failed");

  phycfg.opsel = 1;  // PHY working mode configured by register
  phycfg.opmode = 7; // all capable, auto-negotiation enabled
  ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCFGR, phycfg.val), err, TAG, "Write PHYCFG failed");

  return ESP_OK;

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_pwrctl(esp_eth_phy_t *phy, bool enable)
{
  // power control is not supported for W6100 internal PHY
  return ESP_OK;
}

////////////////////////////////////////

static esp_err_t w6100_set_addr(esp_eth_phy_t *phy, uint32_t addr)
{
  phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
  w6100->addr = addr;

  return ESP_OK;
}

////////////////////////////////////////

static esp_err_t w6100_get_addr(esp_eth_phy_t *phy, uint32_t *addr)
{
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "Addr can't be null");
  phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
  *addr = w6100->addr;

  return ESP_OK;

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_del(esp_eth_phy_t *phy)
{
  phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
  free(w6100);

  return ESP_OK;
}

////////////////////////////////////////

static esp_err_t w6100_advertise_pause_ability(esp_eth_phy_t *phy, uint32_t ability)
{
  // pause ability advertisement is not supported for W6100 internal PHY
  return ESP_OK;
}

////////////////////////////////////////

static esp_err_t w6100_loopback(esp_eth_phy_t *phy, bool enable)
{
  // Loopback is not supported for W6100 internal PHY
  return ESP_ERR_NOT_SUPPORTED;
}

////////////////////////////////////////

static esp_err_t w6100_init(esp_eth_phy_t *phy)
{
  esp_err_t ret = ESP_OK;

  /* Power on Ethernet PHY */
  ESP_GOTO_ON_ERROR(w6100_pwrctl(phy, true), err, TAG, "Power control failed");

  /* Reset Ethernet PHY */
  ESP_GOTO_ON_ERROR(w6100_reset(phy), err, TAG, "Reset failed");

  return ESP_OK;

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_deinit(esp_eth_phy_t *phy)
{
  esp_err_t ret = ESP_OK;

  /* Power off Ethernet PHY */
  ESP_GOTO_ON_ERROR(w6100_pwrctl(phy, false), err, TAG, "Power control failed");

  return ESP_OK;

err:
  return ret;
}

////////////////////////////////////////

esp_eth_phy_t *esp_eth_phy_new_w6100(const eth_phy_config_t *config)
{
  esp_eth_phy_t *ret = NULL;

  ESP_GOTO_ON_FALSE(config, NULL, err, TAG, "Invalid arguments");

  phy_w6100_t *w6100 = calloc(1, sizeof(phy_w6100_t));
  ESP_GOTO_ON_FALSE(w6100, NULL, err, TAG, "No mem for PHY instance");

  /* bind methods and attributes */
  w6100->addr = config->phy_addr;
  w6100->reset_timeout_ms = config->reset_timeout_ms;
  w6100->reset_gpio_num = config->reset_gpio_num;
  w6100->link_status = ETH_LINK_DOWN;
  w6100->autonego_timeout_ms = config->autonego_timeout_ms;
  w6100->parent.reset = w6100_reset;
  w6100->parent.reset_hw = w6100_reset_hw;
  w6100->parent.init = w6100_init;
  w6100->parent.deinit = w6100_deinit;
  w6100->parent.set_mediator = w6100_set_mediator;
  w6100->parent.negotiate = w6100_negotiate;
  w6100->parent.get_link = w6100_get_link;
  w6100->parent.pwrctl = w6100_pwrctl;
  w6100->parent.get_addr = w6100_get_addr;
  w6100->parent.set_addr = w6100_set_addr;
  w6100->parent.advertise_pause_ability = w6100_advertise_pause_ability;
  w6100->parent.loopback = w6100_loopback;
  w6100->parent.del = w6100_del;

  return &(w6100->parent);

err:
  return ret;
}

////////////////////////////////////////

