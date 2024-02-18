/****************************************************************************************************************************
  esp_eth_mac_w6100.c

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
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_eth.h"
#include "esp_system.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "hal/cpu_hal.h"
#include "w6100.h"
#include "sdkconfig.h"

// KH for W6100
#define eth_w6100_config_t      eth_w5500_config_t
//////

////////////////////////////////////////

static const char *TAG = "w6100.mac";

#define W6100_SPI_LOCK_TIMEOUT_MS (50)
#define W6100_TX_MEM_SIZE (0x4000)
#define W6100_RX_MEM_SIZE (0x4000)

////////////////////////////////////////

typedef struct
{
  esp_eth_mac_t parent;
  esp_eth_mediator_t *eth;
  spi_device_handle_t spi_hdl;
  SemaphoreHandle_t spi_lock;
  TaskHandle_t rx_task_hdl;
  uint32_t sw_reset_timeout_ms;
  int int_gpio_num;
  uint8_t addr[6];
  bool packets_remain;
} emac_w6100_t;

////////////////////////////////////////

static inline bool w6100_lock(emac_w6100_t *emac)
{
  return xSemaphoreTake(emac->spi_lock, pdMS_TO_TICKS(W6100_SPI_LOCK_TIMEOUT_MS)) == pdTRUE;
}

////////////////////////////////////////

static inline bool w6100_unlock(emac_w6100_t *emac)
{
  return xSemaphoreGive(emac->spi_lock) == pdTRUE;
}

////////////////////////////////////////

static esp_err_t w6100_write(emac_w6100_t *emac, uint32_t address, const void *value, uint32_t len)
{
  esp_err_t ret = ESP_OK;

  spi_transaction_t trans =
  {
    .cmd = (address >> W6100_ADDR_OFFSET),
    .addr = ((address & 0xFFFF) | (W6100_ACCESS_MODE_WRITE << W6100_RWB_OFFSET) | W6100_SPI_OP_MODE_VDM),
    .length = 8 * len,
    .tx_buffer = value
  };

  if (w6100_lock(emac))
  {
    if (spi_device_polling_transmit(emac->spi_hdl, &trans) != ESP_OK)
    {
      ESP_LOGE(TAG, "%s(%d): SPI transmit failed", __FUNCTION__, __LINE__);
      ret = ESP_FAIL;
    }

    w6100_unlock(emac);
  }
  else
  {
    ret = ESP_ERR_TIMEOUT;
  }

  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_read(emac_w6100_t *emac, uint32_t address, void *value, uint32_t len)
{
  esp_err_t ret = ESP_OK;

  spi_transaction_t trans =
  {
    // use direct reads for registers to prevent overwrites by 4-byte boundary writes
    .flags = len <= 4 ? SPI_TRANS_USE_RXDATA : 0,
    .cmd = (address >> W6100_ADDR_OFFSET),
    .addr = ((address & 0xFFFF) | (W6100_ACCESS_MODE_READ << W6100_RWB_OFFSET) | W6100_SPI_OP_MODE_VDM),
    .length = 8 * len,
    .rx_buffer = value
  };

  if (w6100_lock(emac))
  {
    if (spi_device_polling_transmit(emac->spi_hdl, &trans) != ESP_OK)
    {
      ESP_LOGE(TAG, "%s(%d): SPI transmit failed", __FUNCTION__, __LINE__);
      ret = ESP_FAIL;
    }

    w6100_unlock(emac);
  }
  else
  {
    ret = ESP_ERR_TIMEOUT;
  }

  if ((trans.flags & SPI_TRANS_USE_RXDATA) && len <= 4)
  {
    memcpy(value, trans.rx_data, len);  // copy register values to output
  }

  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_send_command(emac_w6100_t *emac, uint8_t command, uint32_t timeout_ms)
{
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_CR(0), &command, sizeof(command)), err, TAG, "Write SCR failed");

  // after W6100 accepts the command, the command register will be cleared automatically
  uint32_t to = 0;

  for (to = 0; to < timeout_ms / 10; to++)
  {
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_CR(0), &command, sizeof(command)), err, TAG, "Read SCR failed");

    if (!command)
    {
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  ESP_GOTO_ON_FALSE(to < timeout_ms / 10, ESP_ERR_TIMEOUT, err, TAG, "Send command timeout");

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_get_tx_free_size(emac_w6100_t *emac, uint16_t *size)
{
  esp_err_t ret = ESP_OK;
  uint16_t free0, free1 = 0;

  // read TX_FSR register more than once, until we get the same value
  // this is a trick because we might be interrupted between reading the high/low part of the TX_FSR register (16 bits in length)

  do
  {
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_TX_FSR(0), &free0, sizeof(free0)), err, TAG, "Read TX FSR failed");
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_TX_FSR(0), &free1, sizeof(free1)), err, TAG, "Read TX FSR failed");
  } while (free0 != free1);

  *size = __builtin_bswap16(free0);

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_get_rx_received_size(emac_w6100_t *emac, uint16_t *size)
{
  esp_err_t ret = ESP_OK;
  uint16_t received0, received1 = 0;

  do
  {
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_RX_RSR(0), &received0, sizeof(received0)), err, TAG,
                      "Read RX RSR failed");
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_RX_RSR(0), &received1, sizeof(received1)), err, TAG,
                      "Read RX RSR failed");
  } while (received0 != received1);

  *size = __builtin_bswap16(received0);

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_write_buffer(emac_w6100_t *emac, const void *buffer, uint32_t len, uint16_t offset)
{
  esp_err_t ret = ESP_OK;
  uint32_t remain = len;
  const uint8_t *buf = buffer;
  offset %= W6100_TX_MEM_SIZE;

  if (offset + len > W6100_TX_MEM_SIZE)
  {
    remain = (offset + len) % W6100_TX_MEM_SIZE;
    len = W6100_TX_MEM_SIZE - offset;
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_MEM_SOCK_TX(0, offset), buf, len), err, TAG, "Write TX buffer failed");
    offset += len;
    buf += len;
  }

  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_MEM_SOCK_TX(0, offset), buf, remain), err, TAG, "Write TX buffer failed");

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_read_buffer(emac_w6100_t *emac, void *buffer, uint32_t len, uint16_t offset)
{
  esp_err_t ret = ESP_OK;
  uint32_t remain = len;
  uint8_t *buf = buffer;
  offset %= W6100_RX_MEM_SIZE;

  if (offset + len > W6100_RX_MEM_SIZE)
  {
    remain = (offset + len) % W6100_RX_MEM_SIZE;
    len = W6100_RX_MEM_SIZE - offset;
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_MEM_SOCK_RX(0, offset), buf, len), err, TAG, "Read RX buffer failed");
    offset += len;
    buf += len;
  }

  ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_MEM_SOCK_RX(0, offset), buf, remain), err, TAG, "Read RX buffer failed");

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_set_mac_addr(emac_w6100_t *emac)
{
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_MAC, emac->addr, 6), err, TAG, "Write MAC address register failed");

err:
  return ret;
}

////////////////////////////////////////

// KH
static esp_err_t w6100_reset(emac_w6100_t *emac)
{
  esp_err_t ret = ESP_OK;

  // Unlock SYSR[CHPL]
  uint8_t mr = W6100_CHPLCKR_UNLOCK;

  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_CHPLCKR_W6100, &mr, sizeof(mr)), err, TAG, "Write CHPLCKR_W6100 failed");

  uint32_t to = 0;

  for (to = 0; to < emac->sw_reset_timeout_ms / 10; to++)
  {
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SYSR_W6100, &mr, sizeof(mr)), err, TAG, "Read SYSR_W6100 failed");

    // Check CHPL = 0
    if (! ( (mr & W6100_SYSR_CHPL_LOCK) ^ W6100_SYSR_CHPL_ULOCK) )
    {
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  ESP_GOTO_ON_FALSE(to < emac->sw_reset_timeout_ms / 10, ESP_ERR_TIMEOUT, err, TAG, "Unlock timeout");

  /* software reset */
  mr = 0;

  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SYCR0, &mr, sizeof(mr)), err, TAG, "Write SYCR0 failed");

  for (to = 0; to < emac->sw_reset_timeout_ms / 10; to++)
  {
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SYSR_W6100, &mr, sizeof(mr)), err, TAG, "Read SYSR_W6100 failed");

    // Check CHPL = 1
    // Wait Lock Complete
    if (! ( (mr & W6100_SYSR_CHPL_LOCK) ^ W6100_SYSR_CHPL_LOCK) )
    {
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  ESP_GOTO_ON_FALSE(to < emac->sw_reset_timeout_ms / 10, ESP_ERR_TIMEOUT, err, TAG, "Reset timeout");

  // Unlock
  mr = W6100_CHPLCKR_UNLOCK;
  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_CHPLCKR_W6100, &mr, sizeof(mr)), err, TAG, "Write CHPLCKR_W6100 failed");

  mr = W6100_NETLCKR_UNLOCK;
  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_NETLCKR_W6100, &mr, sizeof(mr)), err, TAG, "Write NETLCKR_W6100 failed");

  mr = W6100_PHYLCKR_UNLOCK;
  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_PHYLCKR_W6100, &mr, sizeof(mr)), err, TAG, "Write PHYLCKR_W6100 failed");

  ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SYSR_W6100, &mr, sizeof(mr)), err, TAG, "Read SYSR_W6100 failed");

  ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_VERSIONR_W6100, &mr, sizeof(mr)), err, TAG, "Read VERSIONR_W6100 failed");
  ESP_LOGI(TAG, "version=0x%x", mr);

  ESP_GOTO_ON_FALSE(mr == 0x61, ESP_ERR_TIMEOUT, err, TAG, "Wrong Version");

  ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_CVERSIONR_W6100 + 1, &mr, sizeof(mr)), err, TAG,
                    "Read CVERSIONR_W6100 failed");
  ESP_LOGI(TAG, "cversion=0x%x", mr);

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_verify_id(emac_w6100_t *emac)
{
  esp_err_t ret = ESP_OK;
  uint8_t version = 0;
  ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_VERSIONR_W6100, &version, sizeof(version)), err, TAG,
                    "Read W6100_REG_VERSIONR_W6100 failed");

  // W6100 doesn't have chip ID, we just print the version number instead
  ESP_LOGI(TAG, "version=%x", version);

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t w6100_setup_default(emac_w6100_t *emac)
{
  esp_err_t ret = ESP_OK;
  uint8_t reg_value = 16;

  // Only SOCK0 can be used as MAC RAW mode, so we give the whole buffer (16KB TX and 16KB RX) to SOCK0
  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_RXBUF_SIZE(0), &reg_value, sizeof(reg_value)), err, TAG,
                    "Set rx buffer size failed");

  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_TXBUF_SIZE(0), &reg_value, sizeof(reg_value)), err, TAG,
                    "Set tx buffer size failed");

  reg_value = 0;

  for (int i = 1; i < 8; i++)
  {
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_RXBUF_SIZE(i), &reg_value, sizeof(reg_value)), err, TAG,
                      "Set SOCK_RXBUF_SIZE failed");
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_TXBUF_SIZE(i), &reg_value, sizeof(reg_value)), err, TAG,
                      "Set SOCK_TXBUF_SIZE failed");
  }

  /* Enable ping block, disable PPPoE, WOL */
  reg_value = W6100_MR_PB;
  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_MR, &reg_value, sizeof(reg_value)), err, TAG, "Write MR failed");

  /* Disable interrupt for all sockets by default */
  reg_value = 0;
  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SIMR, &reg_value, sizeof(reg_value)), err, TAG, "Write SIMR failed");

  /* Enable MAC RAW mode for SOCK0, enable MAC filter, no blocking broadcast and multicast */
  reg_value = W6100_SMR_MAC_RAW | W6100_SMR_MAC_FILTER;
  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_MR(0), &reg_value, sizeof(reg_value)), err, TAG,
                    "Write SOCK0 MR failed");

  /* Enable receive event for SOCK0 */
  reg_value = W6100_SIR_RECV;
  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_IMR(0), &reg_value, sizeof(reg_value)), err, TAG,
                    "Write SOCK0 IMR failed");

  /* Set the interrupt re-assert level to maximum (~1.5ms) to lower the chances of missing it */
  //uint16_t int_level = __builtin_bswap16(0xFFFF);
  //ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_INTLEVEL, &int_level, sizeof(int_level)), err, TAG,
  //                  "Write INTLEVEL failed");

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t emac_w6100_start(esp_eth_mac_t *mac)
{
  esp_err_t ret = ESP_OK;
  emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);

  uint8_t reg_value = 0;
  /* open SOCK0 */
  ESP_GOTO_ON_ERROR(w6100_send_command(emac, W6100_SCR_OPEN, 100), err, TAG, "Issue OPEN command failed");

  /* enable interrupt for SOCK0 */
  reg_value = W6100_SIMR_SOCK0;
  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SIMR, &reg_value, sizeof(reg_value)), err, TAG, "Write SIMR failed");

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t emac_w6100_stop(esp_eth_mac_t *mac)
{
  esp_err_t ret = ESP_OK;

  emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
  uint8_t reg_value = 0;
  /* disable interrupt */
  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SIMR, &reg_value, sizeof(reg_value)), err, TAG, "Write SIMR failed");
  /* close SOCK0 */
  ESP_GOTO_ON_ERROR(w6100_send_command(emac, W6100_SCR_CLOSE, 100), err, TAG, "Issue SCR_CLOSE command failed");

err:
  return ret;
}

////////////////////////////////////////

IRAM_ATTR static void w6100_isr_handler(void *arg)
{
  emac_w6100_t *emac = (emac_w6100_t *)arg;
  BaseType_t high_task_wakeup = pdFALSE;

  /* notify w6100 task */
  vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);

  if (high_task_wakeup != pdFALSE)
  {
    portYIELD_FROM_ISR();
  }
}

////////////////////////////////////////

static void emac_w6100_task(void *arg)
{
  emac_w6100_t *emac = (emac_w6100_t *)arg;
  uint8_t status = 0;
  uint8_t *buffer = NULL;
  uint32_t length = 0;

  while (1)
  {
    // check if the task receives any notification
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0 &&    // if no notification ...
        gpio_get_level(emac->int_gpio_num) != 0)
    {
      // ...and no interrupt asserted
      continue;                                                // -> just continue to check again
    }

    /* read interrupt status */
    w6100_read(emac, W6100_REG_SOCK_IR(0), &status, sizeof(status));

    /* packet received */
    if (status & W6100_SIR_RECV)
    {
      status = W6100_SIR_RECV;
      // clear interrupt status
      w6100_write(emac, W6100_REG_SOCK_IR(0), &status, sizeof(status));

      do
      {
        length = ETH_MAX_PACKET_SIZE;
        buffer = heap_caps_malloc(length, MALLOC_CAP_DMA);

        if (!buffer)
        {
          ESP_LOGE(TAG, "No mem for receive buffer");
          break;
        }
        else if (emac->parent.receive(&emac->parent, buffer, &length) == ESP_OK)
        {
          /* pass the buffer to stack (e.g. TCP/IP layer) */
          if (length)
          {
            emac->eth->stack_input(emac->eth, buffer, length);
          }
          else
          {
            free(buffer);
          }
        }
        else
        {
          free(buffer);
        }
      } while (emac->packets_remain);
    }
  }

  vTaskDelete(NULL);
}

////////////////////////////////////////

static esp_err_t emac_w6100_set_mediator(esp_eth_mac_t *mac, esp_eth_mediator_t *eth)
{
  esp_err_t ret = ESP_OK;
  ESP_GOTO_ON_FALSE(eth, ESP_ERR_INVALID_ARG, err, TAG, "Can't set mac's mediator to null");
  emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
  emac->eth = eth;
  return ESP_OK;
err:
  return ret;
}

////////////////////////////////////////

static esp_err_t emac_w6100_write_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value)
{
  esp_err_t ret = ESP_OK;
  emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);

  // PHY register and MAC registers are mixed together in W6100
  // The only PHY register is PHYCFGR
  ESP_GOTO_ON_FALSE(phy_reg == W6100_REG_PHYCFGR, ESP_FAIL, err, TAG, "Wrong PHY register");
  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_PHYCFGR, &reg_value, sizeof(uint8_t)), err, TAG,
                    "write PHY register failed");

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t emac_w6100_read_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value)
{
  esp_err_t ret = ESP_OK;
  ESP_GOTO_ON_FALSE(reg_value, ESP_ERR_INVALID_ARG, err, TAG, "Can't set reg_value to null");
  emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);

  // PHY register and MAC registers are mixed together in W6100
  // The only PHY register is PHYCFGR
  ESP_GOTO_ON_FALSE(phy_reg == W6100_REG_PHYCFGR, ESP_FAIL, err, TAG, "Wrong PHY register");
  ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_PHYCFGR, reg_value, sizeof(uint8_t)), err, TAG,
                    "read PHY register failed");

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t emac_w6100_set_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "Invalid argument");
  emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
  memcpy(emac->addr, addr, 6);
  ESP_GOTO_ON_ERROR(w6100_set_mac_addr(emac), err, TAG, "Set mac address failed");

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t emac_w6100_get_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "Invalid argument");
  emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
  memcpy(addr, emac->addr, 6);

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t emac_w6100_set_link(esp_eth_mac_t *mac, eth_link_t link)
{
  esp_err_t ret = ESP_OK;

  switch (link)
  {
    case ETH_LINK_UP:
      ESP_LOGD(TAG, "Link is up");
      ESP_GOTO_ON_ERROR(mac->start(mac), err, TAG, "W6100 start failed");
      break;

    case ETH_LINK_DOWN:
      ESP_LOGD(TAG, "link is down");
      ESP_GOTO_ON_ERROR(mac->stop(mac), err, TAG, "W6100 stop failed");
      break;

    default:
      ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "Unknown link status");
      break;
  }

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t emac_w6100_set_speed(esp_eth_mac_t *mac, eth_speed_t speed)
{
  esp_err_t ret = ESP_OK;

  switch (speed)
  {
    case ETH_SPEED_10M:
      ESP_LOGD(TAG, "Setting to 10Mbps");
      break;

    case ETH_SPEED_100M:
      ESP_LOGD(TAG, "Setting to 100Mbps");
      break;

    default:
      ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "Unknown speed");
      break;
  }

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t emac_w6100_set_duplex(esp_eth_mac_t *mac, eth_duplex_t duplex)
{
  esp_err_t ret = ESP_OK;

  switch (duplex)
  {
    case ETH_DUPLEX_HALF:
      ESP_LOGD(TAG, "Setting to HALF_DUPLEX");
      break;

    case ETH_DUPLEX_FULL:
      ESP_LOGD(TAG, "Setting to FULL_DUPLEX");
      break;

    default:
      ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "Unknown duplex");
      break;
  }

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t emac_w6100_set_promiscuous(esp_eth_mac_t *mac, bool enable)
{
  esp_err_t ret = ESP_OK;
  emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);

  uint8_t smr = 0;
  ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_MR(0), &smr, sizeof(smr)), err, TAG, "Read SOCK0 MR failed");

  if (enable)
  {
    smr &= ~W6100_SMR_MAC_FILTER;
  }
  else
  {
    smr |= W6100_SMR_MAC_FILTER;
  }

  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_MR(0), &smr, sizeof(smr)), err, TAG, "Write SOCK0 MR failed");

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t emac_w6100_enable_flow_ctrl(esp_eth_mac_t *mac, bool enable)
{
  /* w6100 doesn't support flow control function, so accept any value */
  return ESP_ERR_NOT_SUPPORTED;
}

////////////////////////////////////////

static esp_err_t emac_w6100_set_peer_pause_ability(esp_eth_mac_t *mac, uint32_t ability)
{
  /* w6100 doesn't support PAUSE function, so accept any value */
  return ESP_ERR_NOT_SUPPORTED;
}

////////////////////////////////////////

static inline bool is_w6100_sane_for_rxtx(emac_w6100_t *emac)
{
  uint8_t phycfg;

  /* phy is ok for rx and tx operations if bits RST and LNK are set (no link down, no reset) */
  if (w6100_read(emac, W6100_REG_PHYCFGR, &phycfg, 1) == ESP_OK && (phycfg & 0x8001))
  {
    return true;
  }

  return false;
}

////////////////////////////////////////

static esp_err_t emac_w6100_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
  esp_err_t ret = ESP_OK;

  emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
  uint16_t offset = 0;

  // check if there're free memory to store this packet
  uint16_t free_size = 0;
  ESP_GOTO_ON_ERROR(w6100_get_tx_free_size(emac, &free_size), err, TAG, "Get free size failed");

  ESP_GOTO_ON_FALSE(length <= free_size, ESP_ERR_NO_MEM, err, TAG, "Free size (%d) < send length (%d)", free_size,
                    length);

  // get current write pointer
  ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_TX_WR(0), &offset, sizeof(offset)), err, TAG, "Read TX WR failed");
  offset = __builtin_bswap16(offset);

  // copy data to tx memory
  ESP_GOTO_ON_ERROR(w6100_write_buffer(emac, buf, length, offset), err, TAG, "Write frame failed");

  // update write pointer
  offset += length;
  offset = __builtin_bswap16(offset);
  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_TX_WR(0), &offset, sizeof(offset)), err, TAG, "Write TX WR failed");

  // issue SEND command
  ESP_GOTO_ON_ERROR(w6100_send_command(emac, W6100_SCR_SEND, 100), err, TAG, "Issue SEND command failed");

  // pooling the TX done event
  int retry = 0;
  uint8_t status = 0;

  while (!(status & W6100_SIR_SEND))
  {
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_IR(0), &status, sizeof(status)), err, TAG, "Read SOCK0 IR failed");

    if ((retry++ > 3 && !is_w6100_sane_for_rxtx(emac)) || retry > 10)
    {
      return ESP_FAIL;
    }
  }

  // clear the event bit
  status  = W6100_SIR_SEND;
  ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_IR(0), &status, sizeof(status)), err, TAG, "Write SOCK0 IR failed");

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t emac_w6100_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
{
  esp_err_t ret = ESP_OK;

  emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);

  uint16_t offset = 0;
  uint16_t rx_len = 0;
  uint16_t remain_bytes = 0;
  emac->packets_remain  = false;

  w6100_get_rx_received_size(emac, &remain_bytes);

  if (remain_bytes)
  {
    // get current read pointer
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_RX_RD(0), &offset, sizeof(offset)), err, TAG, "Read RX RD failed");

    offset = __builtin_bswap16(offset);

    // read head first
    ESP_GOTO_ON_ERROR(w6100_read_buffer(emac, &rx_len, sizeof(rx_len), offset), err, TAG, "Read frame header failed");

    rx_len = __builtin_bswap16(rx_len) - 2; // data size includes 2 bytes of header
    offset += 2;

    // read the payload
    ESP_GOTO_ON_ERROR(w6100_read_buffer(emac, buf, rx_len, offset), err, TAG, "Read payload failed, len=%d, offset=%d",
                      rx_len, offset);

    offset += rx_len;

    // update read pointer
    offset = __builtin_bswap16(offset);
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_RX_RD(0), &offset, sizeof(offset)), err, TAG, "Write RX RD failed");

    /* issue RECV command */
    ESP_GOTO_ON_ERROR(w6100_send_command(emac, W6100_SCR_RECV, 100), err, TAG, "Issue RECV command failed");

    // check if there're more data need to process
    remain_bytes -= rx_len + 2;
    emac->packets_remain = remain_bytes > 0;
  }

  *length = rx_len;

err:
  return ret;
}

////////////////////////////////////////

static esp_err_t emac_w6100_init(esp_eth_mac_t *mac)
{
  esp_err_t ret = ESP_OK;

  emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);

  esp_eth_mediator_t *eth = emac->eth;
  esp_rom_gpio_pad_select_gpio(emac->int_gpio_num);
  gpio_set_direction(emac->int_gpio_num, GPIO_MODE_INPUT);
  gpio_set_pull_mode(emac->int_gpio_num, GPIO_PULLUP_ONLY);
  gpio_set_intr_type(emac->int_gpio_num, GPIO_INTR_NEGEDGE); // active low
  gpio_intr_enable(emac->int_gpio_num);
  gpio_isr_handler_add(emac->int_gpio_num, w6100_isr_handler, emac);

  ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_LLINIT, NULL), err, TAG, "Lowlevel init failed");

  /* reset w6100 */
  ESP_GOTO_ON_ERROR(w6100_reset(emac), err, TAG, "Reset w6100 failed");

  /* verify chip id */
  ESP_GOTO_ON_ERROR(w6100_verify_id(emac), err, TAG, "Verify chip ID failed");

  /* default setup of internal registers */
  ESP_GOTO_ON_ERROR(w6100_setup_default(emac), err, TAG, "W6100 default setup failed");

  return ESP_OK;

err:
  gpio_isr_handler_remove(emac->int_gpio_num);
  gpio_reset_pin(emac->int_gpio_num);
  eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);

  return ret;
}

////////////////////////////////////////

static esp_err_t emac_w6100_deinit(esp_eth_mac_t *mac)
{
  emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);

  esp_eth_mediator_t *eth = emac->eth;
  mac->stop(mac);
  gpio_isr_handler_remove(emac->int_gpio_num);
  gpio_reset_pin(emac->int_gpio_num);
  eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);

  return ESP_OK;
}

////////////////////////////////////////

static esp_err_t emac_w6100_del(esp_eth_mac_t *mac)
{
  emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);

  vTaskDelete(emac->rx_task_hdl);
  vSemaphoreDelete(emac->spi_lock);
  free(emac);

  return ESP_OK;
}

////////////////////////////////////////

esp_eth_mac_t *esp_eth_mac_new_w6100(const eth_w6100_config_t *w6100_config, const eth_mac_config_t *mac_config)
{
  esp_eth_mac_t *ret = NULL;
  emac_w6100_t *emac = NULL;

  ESP_GOTO_ON_FALSE(w6100_config && mac_config, NULL, err, TAG, "Invalid argument");

  emac = calloc(1, sizeof(emac_w6100_t));
  ESP_GOTO_ON_FALSE(emac, NULL, err, TAG, "No mem for MAC instance");

  /* w6100 driver is interrupt driven */
  ESP_GOTO_ON_FALSE(w6100_config->int_gpio_num >= 0, NULL, err, TAG, "Invalid interrupt gpio number");

  /* bind methods and attributes */
  emac->sw_reset_timeout_ms = mac_config->sw_reset_timeout_ms;
  emac->int_gpio_num = w6100_config->int_gpio_num;
  emac->spi_hdl = w6100_config->spi_hdl;
  emac->parent.set_mediator = emac_w6100_set_mediator;
  emac->parent.init = emac_w6100_init;
  emac->parent.deinit = emac_w6100_deinit;
  emac->parent.start = emac_w6100_start;
  emac->parent.stop = emac_w6100_stop;
  emac->parent.del = emac_w6100_del;
  emac->parent.write_phy_reg = emac_w6100_write_phy_reg;
  emac->parent.read_phy_reg = emac_w6100_read_phy_reg;
  emac->parent.set_addr = emac_w6100_set_addr;
  emac->parent.get_addr = emac_w6100_get_addr;
  emac->parent.set_speed = emac_w6100_set_speed;
  emac->parent.set_duplex = emac_w6100_set_duplex;
  emac->parent.set_link = emac_w6100_set_link;
  emac->parent.set_promiscuous = emac_w6100_set_promiscuous;
  emac->parent.set_peer_pause_ability = emac_w6100_set_peer_pause_ability;
  emac->parent.enable_flow_ctrl = emac_w6100_enable_flow_ctrl;
  emac->parent.transmit = emac_w6100_transmit;
  emac->parent.receive = emac_w6100_receive;

  /* create mutex */
  emac->spi_lock = xSemaphoreCreateMutex();
  ESP_GOTO_ON_FALSE(emac->spi_lock, NULL, err, TAG, "Create lock failed");

  /* create w6100 task */
  BaseType_t core_num = tskNO_AFFINITY;

  if (mac_config->flags & ETH_MAC_FLAG_PIN_TO_CORE)
  {
    core_num = cpu_hal_get_core_id();
  }

  BaseType_t xReturned = xTaskCreatePinnedToCore(emac_w6100_task, "w6100_tsk", mac_config->rx_task_stack_size, emac,
                                                 mac_config->rx_task_prio, &emac->rx_task_hdl, core_num);
  ESP_GOTO_ON_FALSE(xReturned == pdPASS, NULL, err, TAG, "Create w6100 task failed");

  return &(emac->parent);

err:

  if (emac)
  {
    if (emac->rx_task_hdl)
    {
      vTaskDelete(emac->rx_task_hdl);
    }

    if (emac->spi_lock)
    {
      vSemaphoreDelete(emac->spi_lock);
    }

    free(emac);
  }

  return ret;
}

////////////////////////////////////////

