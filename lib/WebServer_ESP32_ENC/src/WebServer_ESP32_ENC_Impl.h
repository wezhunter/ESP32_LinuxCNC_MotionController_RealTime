/****************************************************************************************************************************
  WebServer_ESP32_ENC_Impl.h

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

#pragma once

#ifndef WEBSERVER_ESP32_ENC_IMPL_H
#define WEBSERVER_ESP32_ENC_IMPL_H

//////////////////////////////////////////////////////////////

bool ESP32_ENC_eth_connected = false;

void ESP32_ENC_onEvent()
{
  WiFi.onEvent(ESP32_ENC_event);
}

void ESP32_ENC_waitForConnect()
{
  while (!ESP32_ENC_eth_connected)
    delay(100);
}

bool ESP32_ENC_isConnected()
{
  return ESP32_ENC_eth_connected;
}

void ESP32_ENC_event(WiFiEvent_t event)
{
  switch (event)
  {
      //#if USING_CORE_ESP32_CORE_V200_PLUS
#if ( ( defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 2) ) && ( ARDUINO_ESP32_GIT_VER != 0x46d5afb1 ) )
    // For breaking core v2.0.0
    // Why so strange to define a breaking enum arduino_event_id_t in WiFiGeneric.h
    // compared to the old system_event_id_t, now in tools/sdk/esp32/include/esp_event/include/esp_event_legacy.h
    // You can preserve the old enum order and just adding new items to do no harm
    case ARDUINO_EVENT_ETH_START:
      ET_LOG(F("\nETH Started"));
      //set eth hostname here
      ETH.setHostname("ESP32-ENC");
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      ET_LOG(F("ETH Connected"));
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      if (!ESP32_ENC_eth_connected)
      {
        ET_LOG3(F("ETH MAC: "), ETH.macAddress(), F(", IPv4: "), ETH.localIP());

        if (ETH.fullDuplex())
        {
          ET_LOG0(F("FULL_DUPLEX, "));
        }
        else
        {
          ET_LOG0(F("HALF_DUPLEX, "));
        }

        ET_LOG1(ETH.linkSpeed(), F("Mbps"));

        ESP32_ENC_eth_connected = true;
      }

      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      ET_LOG("ETH Disconnected");
      ESP32_ENC_eth_connected = false;
      break;

    case ARDUINO_EVENT_ETH_STOP:
      ET_LOG("\nETH Stopped");
      ESP32_ENC_eth_connected = false;
      break;

#else

    // For old core v1.0.6-
    // Core v2.0.0 defines a stupid enum arduino_event_id_t, breaking any code for ESP32_ENC written for previous core
    // Why so strange to define a breaking enum arduino_event_id_t in WiFiGeneric.h
    // compared to the old system_event_id_t, now in tools/sdk/esp32/include/esp_event/include/esp_event_legacy.h
    // You can preserve the old enum order and just adding new items to do no harm
    case SYSTEM_EVENT_ETH_START:
      ET_LOG(F("\nETH Started"));
      //set eth hostname here
      ETH.setHostname("ESP32-ENC");
      break;

    case SYSTEM_EVENT_ETH_CONNECTED:
      ET_LOG(F("ETH Connected"));
      break;

    case SYSTEM_EVENT_ETH_GOT_IP:
      if (!ESP32_ENC_eth_connected)
      {
        ET_LOG3(F("ETH MAC: "), ETH.macAddress(), F(", IPv4: "), ETH.localIP());

        if (ETH.fullDuplex())
        {
          ET_LOG0(F("FULL_DUPLEX, "));
        }
        else
        {
          ET_LOG0(F("HALF_DUPLEX, "));
        }

        ET_LOG1(ETH.linkSpeed(), F("Mbps"));

        ESP32_ENC_eth_connected = true;
      }

      break;

    case SYSTEM_EVENT_ETH_DISCONNECTED:
      ET_LOG("ETH Disconnected");
      ESP32_ENC_eth_connected = false;
      break;

    case SYSTEM_EVENT_ETH_STOP:
      ET_LOG("\nETH Stopped");
      ESP32_ENC_eth_connected = false;
      break;
#endif

    default:
      break;
  }
}

#endif    // WEBSERVER_ESP32_ENC_IMPL_H
