/****************************************************************************************************************************
  esp32_enc28j60.h

  For Ethernet shields using ESP32_ENC (ESP32 + ENC28J60)

  WebServer_ESP32_ENC is a library for the ESP32 with Ethernet ENC28J60 to run WebServer

  Based on and modified from ESP32-IDF https://github.com/espressif/esp-idf
  Built by Khoi Hoang https://github.com/khoih-prog/WebServer_ESP32_ENC
  Licensed under GPLv3 license

  Version: 1.5.3

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.5.1   K Hoang      28/11/2022 Initial coding for ESP32_ENC (ESP32 + ENC28J60). Sync with WebServer_WT32_ETH01 v1.5.1
  1.5.3   K Hoang      11/01/2023 Using built-in ESP32 MAC. Increase default SPI clock to 20MHz from 8MHz
 *****************************************************************************************************************************/
/*
  esp32_enc28j60.h - ETH PHY support for ENC28J60
  Based on ETH.h from arduino-esp32 and esp-idf
  and Tobozo ESP32-ENC28J60 library
*/

#ifndef _ESP32_ENC_H_
#define _ESP32_ENC_H_

#include "WiFi.h"
#include "esp_system.h"
#include "esp_eth.h"

#include <hal/spi_types.h>

////////////////////////////////////////

#if ESP_IDF_VERSION_MAJOR < 4 || ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,4,0)
  #error "This version of Arduino is too old"
#endif

////////////////////////////////////////

static uint8_t ENC28J60_Default_Mac[] = { 0xFE, 0xED, 0xDE, 0xAD, 0xBE, 0xEF };

////////////////////////////////////////

class ESP32_ENC
{
  private:
    bool initialized;
    bool staticIP;

    uint8_t mac_eth[6] = { 0xFE, 0xED, 0xDE, 0xAD, 0xBE, 0xEF };

#if ESP_IDF_VERSION_MAJOR > 3
    esp_eth_handle_t eth_handle;

  protected:
    bool started;
    eth_link_t eth_link;
    static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
#else
    bool started;
    eth_config_t eth_config;
#endif

  public:
    ESP32_ENC();
    ~ESP32_ENC();

    bool begin(int MISO, int MOSI, int SCLK, int CS, int INT, int SPICLOCK_MHZ = 8, int SPIHOST = SPI3_HOST,
               uint8_t *ENC28J60_Mac = ENC28J60_Default_Mac);

    bool config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dns1 = (uint32_t)0x00000000,
                IPAddress dns2 = (uint32_t)0x00000000);

    const char * getHostname();
    bool setHostname(const char * hostname);

    bool fullDuplex();
    bool linkUp();
    uint8_t linkSpeed();

    bool enableIpV6();
    IPv6Address localIPv6();

    IPAddress localIP();
    IPAddress subnetMask();
    IPAddress gatewayIP();
    IPAddress dnsIP(uint8_t dns_no = 0);

    IPAddress broadcastIP();
    IPAddress networkID();
    uint8_t subnetCIDR();

    uint8_t * macAddress(uint8_t* mac);
    String macAddress();

    friend class WiFiClient;
    friend class WiFiServer;
};

////////////////////////////////////////

extern ESP32_ENC ETH;

////////////////////////////////////////

#endif /* _ESP32_ENC_H_ */
