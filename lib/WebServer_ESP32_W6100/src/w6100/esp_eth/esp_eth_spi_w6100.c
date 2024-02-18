/****************************************************************************************************************************
  esp_eth_spi_w6100.c

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

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "esp_eth_w6100.h"
#include "driver/spi_master.h"

#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "w6100.spi";

/*
  // From tools/sdk/esp32/include/esp_eth/include/esp_eth_mac.h
  #define ETH_MAC_DEFAULT_CONFIG()                        \
    {                                                     \
        .sw_reset_timeout_ms = 100,                       \
        .rx_task_stack_size = 2048,                       \
        .rx_task_prio = 15,                               \
        .smi_mdc_gpio_num = 23,                           \
        .smi_mdio_gpio_num = 18,                          \
        .flags = 0,                                       \
        .interface = EMAC_DATA_INTERFACE_RMII,            \
        .clock_config =                                   \
        {                                                 \
            .rmii =                                       \
            {                                             \
                .clock_mode = EMAC_CLK_DEFAULT,           \
                .clock_gpio = EMAC_CLK_IN_GPIO            \
            }                                             \
        }                                                 \
    }
*/

////////////////////////////////////////

// KH for W6100
#define ETH_W6100_DEFAULT_CONFIG      ETH_W5500_DEFAULT_CONFIG

////////////////////////////////////////

esp_eth_mac_t* w6100_new_mac( spi_device_handle_t *spi_handle, int INT_GPIO )
{
  eth_w6100_config_t w6100_config = ETH_W6100_DEFAULT_CONFIG( *spi_handle );
  w6100_config.int_gpio_num = INT_GPIO;

  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();

  //eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  //phy_config.reset_gpio_num = -1;

  mac_config.smi_mdc_gpio_num  = -1; // w6100 doesn't have SMI interface
  mac_config.smi_mdio_gpio_num = -1;
  mac_config.rx_task_prio      = 1;

  return esp_eth_mac_new_w6100( &w6100_config, &mac_config );
}

////////////////////////////////////////

esp_eth_mac_t* w6100_begin(int MISO_GPIO, int MOSI_GPIO, int SCLK_GPIO, int CS_GPIO, int INT_GPIO, int SPICLOCK_MHZ,
                           int SPIHOST)
{
  if (ESP_OK != gpio_install_isr_service(0))
  {
    ESP_LOGE(TAG, "%s(%d): Error gpio_install_isr_service", __FUNCTION__, __LINE__);

    return NULL;
  }

  /* w6100 ethernet driver is based on spi driver */
  spi_bus_config_t buscfg =
  {
    .miso_io_num   = MISO_GPIO,
    .mosi_io_num   = MOSI_GPIO,
    .sclk_io_num   = SCLK_GPIO,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
  };

  if ( ESP_OK != spi_bus_initialize( SPIHOST, &buscfg, SPI_DMA_CH_AUTO ))
  {
    ESP_LOGE(TAG, "%s(%d): Error spi_bus_initialize", __FUNCTION__, __LINE__);

    return NULL;
  }

  spi_device_interface_config_t devcfg =
  {
    .command_bits = 16,
    .address_bits = 8,
    .mode = 0,
    .clock_speed_hz = SPICLOCK_MHZ * 1000 * 1000,
    .spics_io_num = CS_GPIO,
    .queue_size = 20,
    .cs_ena_posttrans = w6100_cal_spi_cs_hold_time(SPICLOCK_MHZ),
  };

  spi_device_handle_t spi_handle = NULL;

  if (ESP_OK != spi_bus_add_device( SPIHOST, &devcfg, &spi_handle ))
  {
    ESP_LOGE(TAG, "%s(%d): Error spi_bus_add_device", __FUNCTION__, __LINE__);

    return NULL;
  }

  return w6100_new_mac( &spi_handle, INT_GPIO );
}

////////////////////////////////////////

