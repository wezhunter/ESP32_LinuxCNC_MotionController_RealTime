/****************************************************************************************************************************
  WebServer_ESP32_ENC.h

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

#ifndef WEBSERVER_ESP32_ENC_H
#define WEBSERVER_ESP32_ENC_H

//////////////////////////////////////////////////////////////

//#if !defined(USING_CORE_ESP32_CORE_V200_PLUS)
#if ( ( defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 2) ) && ( ARDUINO_ESP32_GIT_VER != 0x46d5afb1 ) )
  #define USING_CORE_ESP32_CORE_V200_PLUS      true

  #if (_ETHERNET_WEBSERVER_LOGLEVEL_ > 2)
    #warning Using code for ESP32 core v2.0.0+ in WebServer_ESP32_ENC.h
  #endif

  #define WEBSERVER_ESP32_ENC_VERSION       "WebServer_ESP32_ENC v1.5.3 for core v2.0.0+"
#else
  #if (_ETHERNET_WEBSERVER_LOGLEVEL_ > 2)
    #warning Using code for ESP32 core v1.0.6- in WebServer_ESP32_ENC.h
  #endif

  #define WEBSERVER_ESP32_ENC_VERSION       "WebServer_ESP32_ENC v1.5.3 for core v1.0.6-"
#endif

#define WEBSERVER_ESP32_ENC_VERSION_MAJOR    1
#define WEBSERVER_ESP32_ENC_VERSION_MINOR    5
#define WEBSERVER_ESP32_ENC_VERSION_PATCH    3

#define WEBSERVER_ESP32_ENC_VERSION_INT      1005003

//////////////////////////////////////////////////////////////

#if ( ARDUINO_ESP32S2_DEV || ARDUINO_FEATHERS2 || ARDUINO_ESP32S2_THING_PLUS || ARDUINO_MICROS2 || \
      ARDUINO_METRO_ESP32S2 || ARDUINO_MAGTAG29_ESP32S2 || ARDUINO_FUNHOUSE_ESP32S2 || \
      ARDUINO_ADAFRUIT_FEATHER_ESP32S2_NOPSRAM )

  #error ESP32_S2 not supported. Use WebServer_ESP32_SC_ENC library

#elif ( defined(ARDUINO_ESP32C3_DEV) )

  #error ESP32_C3 not supported. Use WebServer_ESP32_SC_ENC library

#elif ( defined(ARDUINO_ESP32S3_DEV) || defined(ARDUINO_ESP32_S3_BOX) || defined(ARDUINO_TINYS3) || \
        defined(ARDUINO_PROS3) || defined(ARDUINO_FEATHERS3) )

  #error ESP32_S3 not supported. Use WebServer_ESP32_SC_ENC library

#elif ESP32

  #if (_ETHERNET_WEBSERVER_LOGLEVEL_ > 3)
    #warning Using ESP32 architecture for WebServer_ESP32_W5500
  #endif

#define BOARD_NAME      ARDUINO_BOARD

#else
  #error This code is designed to run on ESP32 platform! Please check your Tools->Board setting.
#endif

//////////////////////////////////////////////////////////////

#include <Arduino.h>

#include "WebServer_ESP32_ENC_Debug.h"

//////////////////////////////////////////////////////////////

#include "enc28j60/esp32_enc28j60.h"

#include "WebServer_ESP32_ENC.hpp"
#include "WebServer_ESP32_ENC_Impl.h"

#endif    // WEBSERVER_ESP32_ENC_H
