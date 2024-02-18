/****************************************************************************************************************************
  esp_eth_w5500.h

  For Ethernet shields using ESP32_W5500 (ESP32 + W5500)

  WebServer_ESP32_W5500 is a library for the ESP32 with Ethernet W5500 to run WebServer

  Based on and modified from ESP32-IDF https://github.com/espressif/esp-idf
  Built by Khoi Hoang https://github.com/khoih-prog/WebServer_ESP32_W5500
  Licensed under GPLv3 license

  Version: 1.5.3

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.5.1   K Hoang      29/11/2022 Initial coding for ESP32_W5500 (ESP32 + W5500). Sync with WebServer_WT32_ETH01 v1.5.1
  1.5.2   K Hoang      06/01/2023 Suppress compile error when using aggressive compile settings
  1.5.3   K Hoang      11/01/2023 Using `SPI_DMA_CH_AUTO` and built-in ESP32 MAC
 *****************************************************************************************************************************/

// Copyright 2021 Espressif Systems (Shanghai) PTE LTD
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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////

#include "esp_eth_phy.h"
#include "esp_eth_mac.h"
#include "driver/spi_master.h"

////////////////////////////////////////

#define CS_HOLD_TIME_MIN_NS     210

////////////////////////////////////////

/*
  // From tools/sdk/esp32/include/esp_eth/include/esp_eth_mac.h

  typedef struct
  {
  void *spi_hdl;     //!< Handle of SPI device driver
  int int_gpio_num;  //!< Interrupt GPIO number
  } eth_w5500_config_t;


  #define ETH_W5500_DEFAULT_CONFIG(spi_device) \
  {                                            \
    .spi_hdl = spi_device,                   \
    .int_gpio_num = 4,                       \
  }

*/


////////////////////////////////////////

/**
   @brief Compute amount of SPI bit-cycles the CS should stay active after the transmission
          to meet w5500 CS Hold Time specification.

   @param clock_speed_mhz SPI Clock frequency in MHz (valid range is <1, 20>)
   @return uint8_t
*/
static inline uint8_t w5500_cal_spi_cs_hold_time(int clock_speed_mhz)
{
  if (clock_speed_mhz <= 0 || clock_speed_mhz > 20)
  {
    return 0;
  }

  int temp = clock_speed_mhz * CS_HOLD_TIME_MIN_NS;
  uint8_t cs_posttrans = temp / 1000;

  if (temp % 1000)
  {
    cs_posttrans += 1;
  }

  return cs_posttrans;
}

////////////////////////////////////////

/**
  @brief Create w5500 Ethernet MAC instance

  @param[in] w5500_config: w5500 specific configuration
  @param[in] mac_config: Ethernet MAC configuration

  @return
       - instance: create MAC instance successfully
       - NULL: create MAC instance failed because some error occurred
*/
esp_eth_mac_t *esp_eth_mac_new_w5500(const eth_w5500_config_t *w5500_config,
                                     const eth_mac_config_t *mac_config);

////////////////////////////////////////

/**
  @brief Create a PHY instance of w5500

  @param[in] config: configuration of PHY

  @return
       - instance: create PHY instance successfully
       - NULL: create PHY instance failed because some error occurred
*/
esp_eth_phy_t *esp_eth_phy_new_w5500(const eth_phy_config_t *config);

////////////////////////////////////////

// todo: the below functions should be accessed through ioctl in the future
/**
   @brief Set w5500 Duplex mode. It sets Duplex mode first to the PHY and then
          MAC is set based on what PHY indicates.

   @param phy w5500 PHY Handle
   @param duplex Duplex mode

   @return esp_err_t
            - ESP_OK when PHY registers were correctly written.
*/
esp_err_t w5500_set_phy_duplex(esp_eth_phy_t *phy, eth_duplex_t duplex);

////////////////////////////////////////

#ifdef __cplusplus
}
#endif
